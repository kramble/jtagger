/* jtagger.c - JTAG driver for DE0-Nano (see usercode.c for customised functionality)

Client/Server (now depreciated, runs standalone by default) communicate using System V message queues.

See https://beej.us/guide/bgipc/html//index.html (easy examples, though POSIX is apparently preferred over System V
in more recent times)

NB to list the queues from shell use "ipcs" and "ipcrm" to delete them (usually only neccessary after terminating
the client or server abnormally, eg via CONTROL-C)

Uses ublast_access_ftdi.c from OpenOCD for communication with DE0-Nano onboard Altera USB Blaster, plus code from
usb_blaster.c thus including the original copyright notice...

 SPDX-License-Identifier: GPL-2.0-or-later

 *   Driver for USB-JTAG, Altera USB-Blaster and compatibles
 *
 *   Inspired from original code from Kolja Waschk's USB-JTAG project
 *   (http://www.ixo.de/info/usb_jtag/), and from openocd project.
 *
 *   Copyright (C) 2013 Franck Jullien franck.jullien@gmail.com
 *   Copyright (C) 2012 Robert Jarzmik robert.jarzmik@free.fr
 *   Copyright (C) 2011 Ali Lown ali@lown.me.uk
 *   Copyright (C) 2009 Catalin Patulea cat@vv.carleton.ca
 *   Copyright (C) 2006 Kolja Waschk usbjtag@ixo.de
 *

Actually I don't really use most of the usb_blaster.c code outside of ublast_initial_wipeout() which is easily
refactored to avoid these calls. Currently I'm using raw hex strings gleaned from a reverse engineered OpenFPGALoader
session (it was just the quickest way to get something that worked). So TODO, decide whether to convert that raw
stuff into calls into the OpenOCD stack, or go it my own way (see scan_dr_int() for example) and strip it all out.

*/

#include "common.h"

// BEWARE the server process and jtagger process have INDEPENDENT versions of info (since they
// are separate processes). So data must be copied between them at the message level.
// NB info.drv is used ONLY by server (jtagger does not communicate directly with FTDI)

//
/*
 * Global device control
 */
static struct ublast_info info = {
	.ublast_vid = 0x09fb, /* Altera */
	.ublast_pid = 0x6001, /* USB-Blaster */
	.lowlevel_name = NULL,
	.srst_asserted = false,
	.trst_asserted = false,
	.pin6 = FIXED_1,
	.pin8 = FIXED_1,
	// MJ the remaining members initialize to zero since info is in .bss (see objdump -t jtagger, though it
	// is also listed in .data). The above values are possibly stored in .data (see objdump -s -j.data jtagger
	// which shows fb090160 at offset 7040(YMMV))
};

// ----------------------------------
// Comment from openocd usb_blaster.c
// ----------------------------------

/*
 * Actually, the USB-Blaster offers a byte-shift mode to transmit up to 504 data
 * bits (bidirectional) in a single USB packet. A header byte has to be sent as
 * the first byte in a packet with the following meaning:
 *
 *   Bit 7 (0x80): Must be set to indicate byte-shift mode.
 *   Bit 6 (0x40): If set, the USB-Blaster will also read data, not just write.
 *   Bit 5..0:     Define the number N of following bytes
 *
 * All N following bytes will then be clocked out serially on TDI. If Bit 6 was
 * set, it will afterwards return N bytes with TDO data read while clocking out
 * the TDI data. LSB of the first byte after the header byte will appear first
 * on TDI.
 */

/* Simple bit banging mode:
 *
 *   Bit 7 (0x80): Must be zero (see byte-shift mode above)
 *   Bit 6 (0x40): If set, you will receive a byte indicating the state of TDO
 *                 in return.
 *   Bit 5 (0x20): Output Enable/LED.
 *   Bit 4 (0x10): TDI Output.
 *   Bit 3 (0x08): nCS Output (not used in JTAG mode).
 *   Bit 2 (0x04): nCE Output (not used in JTAG mode).
 *   Bit 1 (0x02): TMS Output.
 *   Bit 0 (0x01): TCK Output.
 *
 * For transmitting a single data bit, you need to write two bytes (one for
 * setting up TDI/TMS/TCK=0, and one to trigger TCK high with same TDI/TMS
 * held). Up to 64 bytes can be combined in a single USB packet.
 * It isn't possible to read a data without transmitting data.
 */

#define TCK		(1 << 0)
#define TMS		(1 << 1)
#define NCE		(1 << 2)
#define NCS		(1 << 3)
#define TDI		(1 << 4)
#define LED		(1 << 5)
#define READ		(1 << 6)
#define SHMODE		(1 << 7)
#define READ_TDO	(1 << 0)

static int ublast_buf_write(uint8_t *buf, int size, uint32_t *bytes_written)
{
	// NB this is a wrapper around respond() so we ASSUME success and set 
	// bytes_written and return value accordingly. This function is supplied
	// because other ublast functions call it, and I don't want to rewrite them all.

	if (size > BUF_LEN)			// Belt'n'Braces check
		return ERROR_FAIL;

	*bytes_written = size;		// Assume success

	char str[MSGBUFLEN];
	char *hex = hexdump(buf, size);
	strcpy (str, "WX");
	strcat (str, hex);
	if (hex)
		free(hex);
	strcat (str, "Z");
	respond(str);
	return ERROR_OK;			// NB caller must check message queue response to get actual status
}

static int nb_buf_remaining(void)
{
	return BUF_LEN - info.bufidx;
}

void ublast_flush_buffer(void)
{
	uint32_t retlen;
	int nb = info.bufidx, ret = ERROR_OK;

	while (ret == ERROR_OK && nb > 0) {
		ret = ublast_buf_write(info.buf, nb, &retlen);
		nb -= retlen;
		// MJ BUG? This does not update the ublast_buf_write() buf parameter, so won't work
		// properly if the loop executes more then once (which should not happen anyway)
	}
	info.bufidx = 0;
}

/**
 * ublast_queue_byte - queue one 'bitbang mode' byte for USB Blaster
 * @param abyte the byte to queue
 *
 * Queues one byte in 'bitbang mode' to the USB Blaster. The byte is not
 * actually sent, but stored in a buffer. The write is performed once
 * the buffer is filled, or if an explicit ublast_flush_buffer() is called.
 */
void ublast_queue_byte(uint8_t abyte)
{
	if (nb_buf_remaining() < 1)
		ublast_flush_buffer();
	info.buf[info.bufidx++] = abyte;
	if (nb_buf_remaining() == 0)
		ublast_flush_buffer();
//	LOG_DEBUG_IO("(byte=0x%02x)", abyte);
}

/**
 * ublast_compute_pin - compute if gpio should be asserted
 * @param steer control (ie. TRST driven, SRST driven, of fixed)
 *
 * Returns pin value (1 means driven high, 0 mean driven low)
 */
bool ublast_compute_pin(enum gpio_steer steer)
{
	switch (steer) {
	case FIXED_0:
		return 0;
	case FIXED_1:
		return 1;
	case SRST:
		return !info.srst_asserted;
	case TRST:
		return !info.trst_asserted;
	default:
		return 1;
	}
}

/**
 * ublast_build_out - build bitbang mode output byte
 * @param type says if reading back TDO is required
 *
 * Returns the compute bitbang mode byte
 */
static uint8_t ublast_build_out(enum scan_type type)
{
	uint8_t abyte = 0;

	abyte |= info.tms ? TMS : 0;
	abyte |= ublast_compute_pin(info.pin6) ? NCE : 0;
	abyte |= ublast_compute_pin(info.pin8) ? NCS : 0;
	abyte |= info.tdi ? TDI : 0;
	abyte |= LED;
	if (type == SCAN_IN || type == SCAN_IO)
		abyte |= READ;
	return abyte;
}

/**
 * ublast_clock_tms - clock a TMS transition
 * @param tms the TMS to be sent
 *
 * Triggers a TMS transition (ie. one JTAG TAP state move).
 */
void ublast_clock_tms(int tms)
{
	uint8_t out;

//	LOG_DEBUG_IO("(tms=%d)", !!tms);
	info.tms = !!tms;
	info.tdi = 0;
	out = ublast_build_out(SCAN_OUT);
	ublast_queue_byte(out);
	ublast_queue_byte(out | TCK);
}

/**
 * ublast_idle_clock - put back TCK to low level
 *
 * See ublast_queue_tdi() comment for the usage of this function.
 */
void ublast_idle_clock(void)
{
	uint8_t out = ublast_build_out(SCAN_OUT);

//	LOG_DEBUG_IO(".");
	ublast_queue_byte(out);
}

/**
 * ublast_tms_seq - write a TMS sequence transition to JTAG
 * @param bits TMS bits to be written (bit0, bit1 .. bitN)
 * @param nb_bits number of TMS bits (between 1 and 8)
 * @param skip number of TMS bits to skip at the beginning of the series
 *
 * Write a series of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=\<new_state>, TDI=\<???>
 *  - writing out TCK=1, TMS=\<new_state>, TDI=\<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */
void ublast_tms_seq(const uint8_t *bits, int nb_bits, int skip)
{
	int i;

//	LOG_DEBUG_IO("(bits=%02x..., nb_bits=%d)", bits[0], nb_bits);
	for (i = skip; i < nb_bits; i++)
		ublast_clock_tms((bits[i / 8] >> (i % 8)) & 0x01);
	ublast_idle_clock();
}

static void ublast_initial_wipeout(void)
{
	// openocd usb_blaster.c does ublast_initial_wipeout() on initial call to ublast_execute_queue()
	// At that point info.pin6 = FIXED_1, info.pin8 = FIXED_1 (see info initialization above), also
	// info.tms = 0 since info is a bss global, so has initialized to 0.

	// This writes the full BUF_LEN of TCK + TMS=0 (see above)
	// As stated below, the purpose is to flush the USB-Blaster queue fifos
	// Unsure of the effect on the actual scan chain. See p152 of xilinx-configuration-ug380.pdf ...
	// If the FPGA TAP starts out in reset state, it should got to RUN/IDLE and remain there

	// NB respond() gives EINVAL if len > MSGMAX which is 8192 ... oops it's 8196 for ublast_initial_wipeout()
	// So I'm going to cheat and send a half sized buffer twice
	int bytes_to_send = BUF_LEN / 2;

	if (!g_silent)
		printf("wipeout...\n");

	// These are the initial values (since info is .bss). The original openocd did not set them here (likely
	// since it only called ublast_initial_wipeout() ONCE from ublast_execute_queue() via first_call static int.
	// HOWEVER in jtagger ublast_initial_wipeout() can be called again on a failure (after FTDI close/reopen)
	// so ensure they are set correctly each time.

	// printf("wipeout initial tdi %d tms %d\n", info.tdi, info.tms);	// Check them
	info.tdi = 0;
	info.tms = 0;

	static uint8_t tms_reset = 0xff;
	uint8_t out_value;
	uint32_t retlen;
	int i;

	out_value = ublast_build_out(SCAN_OUT);
	for (i = 0; i < bytes_to_send; i++)
		info.buf[i] = out_value | ((i % 2) ? TCK : 0);

	/*
	 * Flush USB-Blaster queue fifos
	 *  - empty the write FIFO (128 bytes)
	 *  - empty the read FIFO (384 bytes)
	 */
	
	// Send it twice (see note above) ... NB ublast_buf_write() calls respond() so sequence prefix is applied
	ublast_buf_write(info.buf, bytes_to_send, &retlen);

	if (!g_standalone)
		JTAGGER_SLEEP(100 * MILLISECONDS);	// allow time for the FTDI I/O operation

	ublast_buf_write(info.buf, bytes_to_send, &retlen);
	if (!g_standalone)
		JTAGGER_SLEEP(100 * MILLISECONDS);

	/*
	 * Put JTAG in RESET state (five 1 on TMS)
	 */
	ublast_tms_seq(&tms_reset, 5, 0);
	// tap_set_state(TAP_RESET);

	ublast_flush_buffer();	// Added this to write the TAP_RESET
    jflush();				// Ensure server.c buffer is flushed

	if (!g_standalone)
		JTAGGER_SLEEP(100 * MILLISECONDS);

	// After physically connecting the USB device cable, the first attempt to access the device
	// returns the device ID mis-aligned. Flushing the read buffer TWICE fixes this (alternatively
	// closing and reopening the FTDI device works, but this is cleaner). Note that the FTDI status
	// indicates that the read buffer is empty on startup, but it does return data! WERID.

	clientflushrx();

	respond("RX80Z");	// Read 128 bytes to clear read buffer
	if (!g_standalone)
		JTAGGER_SLEEP(500 * MILLISECONDS);	// NB there normally is nothing to read, so FTDI I/O will timeout

	int retmsg = io_check();

	// INFO I saw the following in server log for the first read after cable connect: 111 bytes
	if (!g_silent)
		printf("flush read buffer retmsg %08X\n", retmsg);

	if (retmsg != 0x5A58524B)	// Indicates zero bytes read which is true for all except initial cable connection
	{
		respond("RX80Z");	// Read another 128 bytes (seems to be necessary when previous read was non-zero bytes)
		if (!g_standalone)
			JTAGGER_SLEEP(500 * MILLISECONDS);
		int retmsg = io_check();
		if (!g_silent)
			printf("second flush of read buffer retmsg %08X\n", retmsg);
	}
}

// These are NOT static since called from program() and usercode()

void jflush(void)
{

	respond("PZ");	// Flush server.c write buffer (NB flushes automatically on read)
}

int tap_reset(void)
{
	// Enter TAP_RESET state (valid from any state)
    respond("WX2e2f2e2f2e2f2e2f2e2f2eZ");
    // TMS        1   1   1   1   1

	return io_check();
}

int runtest5(void)
{
	// Move from TAP_RESET to RUN/IDLE (NB all openocd sequences assume RUNIDLE as start point)
    respond("WX2c2d2c2d2c2d2c2d2c2d2cZ");
    // TMS        0   0   0   0   0
	return io_check();
}

void IRSHIFT_USER0(void)	// Select VDR, no readback, entry RUNIDLE exit PAUSEIR 
{
    respond("WX2e2f2e2f2c2d2c2d2c810c2c2d2e2f2c2d2cZ");	// IRSHIFT USER0 0x0c
    // TMS        1   1   0   0 data    0   1   0 (NB 2 extra tdi=0 shifts after data)
}

void IRSHIFT_USER1(void)	// Select VIR, no readback, entry RUNIDLE exit PAUSEIR
{
    respond("WX2e2f2e2f2c2d2c2d2c810e2c2d2e2f2c2d2cZ");	// IRSHIFT USER1 0x0e
    // TMS        1   1   0   0 data    0   1   0 (NB 2 extra tdi=0 shifts after data)
}

int runtest(int n)	// NOT static since called from program()
{
	char buf[256];

	if (n < 1)
		DOABORT("n < 1");

	// Check for bits that won't fit into byte packet
	int residual = (n & 7);

	if (n < 16)			// Always bitbang if < 2 bytes
		residual = n;

	if (residual)
	{
		strcpy(buf, "WX");
		int i = residual;
		while (i--)
			strcat(buf, "2c2d");
		strcat(buf, "Z");	// uses max 64 chars, inc null
		respond(buf);
	}

	if (0 == (n -= residual))
		return 0;

	// Construct buffer for bulk write
	strcpy(buf, "WXbf");	// 63 bytes in byte mode
	for (int i=0; i<63*2; i++)
		buf[4+i] = '0';		// char '0' giving 63 hex pairs "00"
	buf[130] = 'Z';
	buf[131] = 0;			// terminate

	// TODO pack multiple of these into a max size message (NB use respond so as to prefix sequence)

	while (n > 0)
	{
		int bytes = 63;
		if (n < bytes * 8)
			bytes = n / 8;

		// printf("bytes %d\n", bytes);

		if (bytes < 63)
		{
			// Last use of buf, so ok to overwrite
			sprintf(buf,"WX%02x", 0x80 + bytes);
			buf[4] = '0';			// printf terminated it, so fix
			buf[4+bytes*2] = 'Z';
			buf[5+bytes*2] = 0;
		}

		// printf("runtest <%s>\n", buf);
		respond(buf);
			
		n -= bytes * 8;
		if (n < 0)		// DEBUG check
		{
			printf("runtest: runtime bad n = %d\n", n);
			exit(1);
		}
	}

	return 0;
}

int scan_dr_int(unsigned int val, int bits, int read)
{
	// Requires entry from PAUSEIR, exits in RUNIDLE
	// Require a minimum of 4 bits since using hub scan as template, the final two
	// bits needing special handing due to JTAG mode change
	// NB all done in bitbang mode (TODO use byte mode)

	if (bits < 4 || bits > 64)
	{
		printf("scan_dr ERROR invalid bits %d\n", bits);
		DOABORT ("invalid bits");
	}

    // Construct (5 bit example) "WX2e2f2e2f2e2f2c2d2c2d2c2c6d2c6d2c6d2c6d2e7f3eZ");
    //                       TMS       1   1   1   0   0     0   0   0   0   1
    //                                  capturedr->^   ^<-shiftdr   exit1dr->^ (needs extra TMS=1,0 to RUNIDLE)

	char str[1024];	// Plenty for 64 bits (4 char per bit, plus leadin/tailout)

	char s[5];		// Construction zone
	s[4] = 0;

    strcpy (str, "WX2e2f2e2f2e2f2c2d2c2d2c");	// PAUSEIR/DR to SHIFTDR
    // TMS             1   1   1   0   0

	for (int i=0; i<bits; i++)
	{
		char c1 = read ? '6' : '2';
		char c2 = 'd';
		char c3 = '2';
		char c4 = 'c';

		if (val & 1)		// Set TDI
		{
			c1 |= 1;
			c3 |= 1;
		}

		if (bits - i < 3 )	// Set TMS
			c4 += 2;		// NB add, "or" won't work on 'c' -> 'e' since 'c' = 0x63 so bit is already set

		if (bits - i == 1)	// Set TMS
			c2 += 2;

		val >>= 1;

		s[0] = c1; s[1] = c2; s[2] = c3; s[3] = c4;
		strcat (str, s);
	}

	strcat (str, "2e2f2c2d2cZ");	// end at RUNIDLE

		respond(str);

	return 0;
}

long long unsigned scan_vir_vdr(unsigned int irlen, unsigned int vrlen, unsigned int vir, unsigned int vdr, int read)
{
	long long unsigned vdr_ret = 0;

	// TODO speed this up

	tap_reset();
	runtest5();

	IRSHIFT_USER1();	// entry RUNIDLE exit PAUSEIR

#if 0	// We don't need this result and NOREADMODE is significantly faster

	// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
	scan_dr_int(0x10 | vir, irlen+1, READMODE);	// PAUSEIR to SHIFTDR (addr=1 is MSB, hence 0x10)

	clientflushrx();

	respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
	io_check();
#else
	// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
	scan_dr_int(0x10 | vir, irlen+1, NOREADMODE);	// PAUSEIR to SHIFTDR (addr=1 is MSB, hence 0x10)
#endif

	IRSHIFT_USER0();

	if (read)
	{
		scan_dr_int(vdr, vrlen, READMODE);

		clientflushrx();
		char str[256];
		sprintf(str, "RX%02xZ", vrlen);
		respond(str);	// read bitbangs
		io_check();

		vdr_ret = get_bitbang(vrlen, 0);

		if (!g_silent)
			printf("scan_vir_vdr returned %08" PRIx64 "\n", (int64_t)vdr_ret);
	}
	else
		scan_dr_int(vdr, vrlen, NOREADMODE);

	return vdr_ret;
}

int print_nibble(void)
{
	// eg g_clientmsg.mtext == "9KRX02020203Z" is binary 1000 (ie reversed)
	char *p = g_clientmsg.mtext + 4;
	int n = 0;
	for (int i=0; i<4; i++)
	{
		n >>= 1;
		if (*p != '0')
			printf("print_nibble: ERROR expected '0' got '%c'\n", *p);
		int ch = *++p;
		if (ch == '3')
			n |= 8;
		else if (ch != '2')
			printf("print_nibble: ERROR expected '2|3' got '%c'\n", ch);
		p++;
	}
	if (!g_silent)
		printf("nibble 0x%x (%d)\n", n, n);
	return n;
}

unsigned long long get_bitbang(unsigned int len, unsigned int shift)
{
	// same as print_nibble but for variable length
	char *p = g_clientmsg.mtext + 4;
	unsigned long long n = 0;
	for (int i=0; i<len; i++)
	{
		n >>= 1;
		if (*p != '0')
			printf("print_bits: ERROR expected '0' got '%c'\n", *p);
		int ch = *++p;
		if (ch == '3')
			n |= (1LL << (len-1+shift));
		else if (ch == '2')
			n &= ~(1LL << (len-1+shift));
		else
			printf("print_bits: ERROR expected '2|3' got '%c'\n", ch);
		p++;
	}
	if (!g_silent)
		printf("value 0x%" PRIx64 "\n", (int64_t) n);
	return n;
}

// NOTE the bulk transfer functions may appear to belong in jtagger.c BUT they rely on user #defines IRDATA and IWDATA
// which do NOT belong in jtagger.c so they will have to stay in uercode.c (at least until I think of a better way)
// UPDATE added vir parameter, so now moved them to jtagger.c

void bulk_readbuf(char *mem, unsigned int len)
{
	// Processing g_clientmsg.mtext so no point having an error return
	// NB A length mismatch would result in an error in the server.c FTDI read, so handled there
	// Unlike get_bitbang() len is bytes, not bits

	char *p = g_clientmsg.mtext + 4;

	// printf("rbuf = [%s]\n", p-4);

	for (int i=0; i<len; i++)
	{
		// TODO make efficient
		unsigned char b, c=0;
		b = *p++;	// BEWARE do not use UNHEX() directly on p++ (else ++ will happen 6 times)
		c = UNHEX(b);
		b = *p++;
		c = c<<4 | UNHEX(b);
		*mem++ = c;
	}
}

static void bulk_residual_write(char *packbuf, char *dst, int mode)
{
	// Based on program.c/send_residual() with addition of mode
	int debug = 0;
	int lastlen = 0;

	if (dst > packbuf+2)	// Check for residual data (ie packbuf contains more than "WX")
	{
		if (debug)
			printf("dst %p *dst=%d packbuf %p ...\n%s\n", dst, *dst, packbuf, packbuf);	// *dst should (now) be NULL

		// TESTED by changing postamble in devices.c - use parameters preamble=3192, postamble=401 for max
		// packet "bff..ffZ", then postamble=409 for rollover "81ffZ"

		int numpackets = (dst - packbuf - 2) / 128;		// 128 for (header byte + 63 bytes data) * 2 for hex
		char* last = packbuf + numpackets * 128 + 2;	// The -2 above / + 2 here allows for "WX"

		if (debug)
		{
			lastlen = dst - last;	// DEBUG
			printf("numpackets = %d packbuf = %p last = %p lastlen = %d\n", 
				numpackets, packbuf, last, lastlen);
			printf("packlen = %" PRIuPTR " lastoffset = %" PRIuPTR "\n", dst - packbuf, last - packbuf);
		}

		if (last < dst)
		{
			// update header for last packet
			unsigned char header = 0x80 | ((dst - last) / 2 - 1);	// -1 allows for the header byte itself
			if (header > 0xbf)	// max byte packet
				DOABORT("header > 0xbf");
			header |= mode ? 0 : 0x40;		// Set read flag
			*last = TOHEX(header>>4);
			*(last+1) = TOHEX(header);
		}
		else if (last == dst)
		{
			// Last packet was already complete (seen with preamble=3192, postamble=401), this is OK
			if (debug)
				printf("INFO last == dst\n");
		}
		else
			// DOABORT("last > dst");						// This should not happen
			printf("%s: ERROR last > dst\n", __func__);		// But don't abort (programming may succeed anyway)

		// Write residual
		*dst++ = 'Z';
		*dst = 0;

		if (debug)
			puts(packbuf);

		respond(packbuf);
	}
}

static void bulk_residual_read(char *packbuf, char *dst, char *result, int mode)
{
	// Separate for now until debugged, TODO combine with write

	// Based on program.c/send_residual() with addition of mode
	int debug = 0;
	int lastlen = 0;

	int packlen = mode ? 128 : 30;	// NB read packet includes 20 byte TMS sequence hence (2 + 8 + 20)

	if (dst > packbuf+2)	// Check for residual data (ie packbuf contains more than "WX")
	{
		if (debug)
			printf("dst %p *dst=%d packbuf %p ...\n%s\n", dst, *dst, packbuf, packbuf);	// *dst should (now) be NULL

		int numpackets = (dst - packbuf - 2) / packlen;		// 128 for (header byte + 63 bytes data) * 2 for hex
		char* last = packbuf + numpackets * packlen + 2;	// The -2 above / + 2 here allows for "WX"

		if (debug)
		{
			lastlen = dst - last;	// DEBUG
			printf("numpackets = %d packbuf = %p last = %p lastlen = %d\n", 
				numpackets, packbuf, last, lastlen);
			printf("packlen = %" PRIuPTR " lastoffset = %" PRIuPTR "\n", dst - packbuf, last - packbuf);
		}

		if (last < dst)
		{
			// update header for last packet
			int adj = mode ? 0 : 20;
			unsigned char header = 0x80 | ((dst - last - adj) / 2 - 1);	// -1 allows for the header byte itself
			if (header > 0xbf)	// max byte packet
				DOABORT("header > 0xbf");
			header |= mode ? 0 : 0x40;		// Set read flag
			*last = TOHEX(header>>4);
			*(last+1) = TOHEX(header);
		}
		else if (last == dst)
		{
			// Last packet was already complete
			if (debug)
				printf("INFO last == dst\n");
		}
		else
			// DOABORT("last > dst");						// This should not happen
			printf("%s: ERROR last > dst\n", __func__);		// But don't abort (programming may succeed anyway)

		// Write residual
		*dst++ = 'Z';
		*dst = 0;

		if (debug)
			puts(packbuf);

		respond(packbuf);

		if (lastlen)
			DOABORT("lastlen");		// Read should not have any partial packets

		int rcount = numpackets;

		// printf("Readback %d ints %d bytes\n", rcount, rcount*4);

		char tmp[64];
		// BEWARE server.c only allows 2 byte (4 hex char) size
		sprintf(tmp,"RX%02x%02xZ", (rcount*4)&0xff, (rcount/64)&0xff);
		respond(tmp);

		io_check();
		bulk_readbuf(result, rcount*4);

		rcount = 0;
	}
}

static void bulk_transfer(char *mem, unsigned int len, int mode, int vir)
{
	// Setup is same as scan_vir_vdr(4, 32, ...)

	// int vir = mode ? IWDATA : IRDATA	// Now passed as parameter
	int irlen = 4;

	tap_reset();
	runtest5();

	IRSHIFT_USER1();

	// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
	scan_dr_int(0x10 | vir, irlen+1, NOREADMODE);	// hub addr=1 is MSB, hence 0x10

	IRSHIFT_USER0();	// exits in PAUSEIR

	// Based on scan_dr_int(vdr, vrlen) and parse_rbf()
	// Write scans data non-stop (does not pass through UPDATEDR, see jtag_vdr.v)
	// Read goes though the normal capture/update loop

	char packbuf[BUF_LEN];	// NB BUF_LEN is message buffer
	*packbuf = 0;			// Set as empty

    respond("WX2e2f2e2f2e2f2c2d2c2d2cZ");	// PAUSEIR to SHIFTDR
    // TMS        1   1   1   0   0

	char *dst = packbuf;
	strcpy(dst, "WX");
	dst += 2;

	char *p = mem;
	char *r = mem;	// FTDI read data is written here
	int rcount = 0;
	int byte = 0;
	while (p < mem + len)
	{
		if (byte == 0)
		{
			// packet header
			if (mode)
			{
				*dst++ = 'b';	// Write 63 bytes
				*dst++ = 'f';
			}
			else
			{
				*dst++ = 'c';	// Read 4 bytes
				*dst++ = '4';
			}
		}

		unsigned char ch = *p++;

		// Two hex chars (bytes)
		// NB these values are redundant when reading, but still need to be sent over JTAG (as a bonus
		// this could implement a simultaneous read/write operation with appropriate verilog support)
		*dst++ = TOHEX(ch>>4);
		*dst++ = TOHEX(ch);
		*dst = 0;	// Not necessary but makes debugging easier since can see end of data when printing buffer

		int packlen = mode ? 62 : 3;	// Write mode sends 63 byte packets, read is 4 bytes
		if (++byte > packlen)
		{
			byte = 0;

			if (mode)
			{
				// Writing

				size_t pblen = dst - packbuf;
				if (pblen > sizeof(packbuf) - 132)	// Tweak so it just fits
				{
					*dst++ = 'Z';
					*dst = 0;

					respond(packbuf);

					dst = packbuf;
					strcpy(dst, "WX");
					dst += 2;
					*dst = 0;	// Not neccessary (but eases debug)
				}
			}
			else
			{
				// Reading
				// Cycle through UPDATEDR/CAPTUREDR back to SHIFTDR
				strcpy(dst, "2e2f2e2f2e2f2c2d2c2d");	// TMS 1 1 1 0 0
				dst += 20;
				*dst = 0;
				rcount++;

				size_t pblen = dst - packbuf;
				if (pblen > sizeof(packbuf) - 132)	// Tweak so it just fits (this is PLENTY for read)
				{
					// This does not get used (read exceeds max allowed) so may be buggy

					printf("Readback full buffer ints %d bytes %d\n", rcount, rcount*4);
					*dst++ = 'Z';
					*dst = 0;

					respond(packbuf);

					char tmp[64];
					// BEWARE server.c only allows 2 byte (4 hex char) size
					sprintf(tmp,"RX%02x%02xZ", (rcount*4)&0xff, (rcount/64)&0xff);
					respond(tmp);

					io_check();
					bulk_readbuf(r, rcount*4);
					r += rcount + 4;

					dst = packbuf;
					strcpy(dst, "WX");
					dst += 2;
					*dst = 0;

					rcount = 0;
				}
			}
		}
	}

	// TODO combine
	if (mode)
		bulk_residual_write(packbuf, dst, mode);
	else
		bulk_residual_read(packbuf, dst, r, mode);

	tap_reset();
	jflush();
}

void bulk_write(char *mem, unsigned int len, int vir)
{
	bulk_transfer(mem, len, 1, vir);
}

void bulk_read(char *mem, unsigned int len, int vir)
{
	if (len % 4)
		DOABORT("length must be a multiple of 4");

	if (len > 72 * 4)
		DOABORT("length cannot exceed 288 bytes");	// Seems to be a FTDI limit, TODO check documentation

	bulk_transfer(mem, len, 0, vir);
}

static int find_device(unsigned int device_id)
{
	for (int i=1; /* empty */; i++)	// NB first array entry is for defaults, so search from 1
	{
		int j;	// Check for all entries zero (end of device_params[] array). Risks a segfault
				// if device_params[] is misconfigured, but I did warn quite firmly in devices.c
		for (j=0; j<=DEVICE_PARAMS_MAXINDEX; j++)
			if (device_params[i][j])
				break;
		if (j > DEVICE_PARAMS_MAXINDEX)
			break;		// return 0

#if 0	// DEBUG
		printf("device_params %d %d %d %d %d %d\n", device_params[i][0], device_params[i][1], device_params[i][2],
			device_params[i][3], device_params[i][4], device_params[i][5]);

		printf("device_params[%d][0] %08x compare device_id %08x\n", i, device_params[i][0], device_id);
#endif
		if (device_params[i][DEVICE_PARAMS_CHIP_ID] == device_id)
			return i;	// found
	}
	return 0;		// NB device_index=0 explicitly means not found
}

int init_fpga(int *device_index)
{
	// Checks server status, opens FTDI if needed, and checks jtag functionality (queries fpga chip id)

	printf("init_fpga\n");
	g_ftdi_ok = 0;	// Assume NOT connected OK - BEWARE main() may attempt connection but status is checked here

	if (!device_index)
		DOABORT("device_index");

	// Discard any pending messages
	int retmsg = clientflushrx();
	if (!g_silent)
		printf("client flush retmsg %08X\n", retmsg);

	respond("SZ");	// Get status

	// Allow time for server to respond
	if (!g_standalone)
		JTAGGER_SLEEP(10 * MILLISECONDS);	// 10 milliseconds should be sufficient for status, FTDI I/O will take longer.
	retmsg = clientflushrx();
	if (!g_silent)
		printf("status retmsg %08X\n", retmsg);

	if ((retmsg & 0xFF) != JMSG_OK)
		return 1;

	retmsg &= 0xF2FFFFFF;		// Mask off the error bits (TODO rework the error bit handling)
								// as we only want the connection status (0x32)

	if (retmsg == 0x3058534B)	// status is zero
	{
		// Connect
		printf("attemting to open FTDI...\n");
		respond("JZ");
		if (!g_standalone)
			JTAGGER_SLEEP(500 * MILLISECONDS);	// 500 milliseconds since JCMD_OPEN takes longer
		retmsg = clientflushrx();
		if (!g_silent)
			printf("ftdi open retmsg %08X\n", retmsg);

		if ((retmsg & 0xFF) != JMSG_OK)
		{
			printf("FTDI open failed\n");
			return 2;
		}
	}
	else if (retmsg != 0x3258534B)	// status is connected (and no error flags)
	{
		printf("bad status\n");
			return ERROR_BADCHIP;	// caller will retry
	}

	g_ftdi_ok = 1;	// Used at final exit from main() to send TAP_RESET. NB set to 0 above at entry to jtagger()

	// Get status again (just for info)
	respond("SZ");
	if (!g_standalone)
		JTAGGER_SLEEP(10 * MILLISECONDS);
	retmsg = clientflushrx();
	if (!g_silent)
		printf("status retmsg %08X\n", retmsg);

	if ((retmsg & 0xFF) != JMSG_OK)
		return 4;

	if (!g_silent)
	{
		// Get FTDI status
		respond("HZ");
		if (!g_standalone)
			JTAGGER_SLEEP(10 * MILLISECONDS);
		retmsg = clientflushrx();
		printf("ftdi status retmsg %08X\n", retmsg);
		printf("message: %s\n", g_clientmsg.mtext);	// Demonstrate access to last message
	}

	ublast_initial_wipeout();

	// NB ublast_initial_wipeout() has done clientflushrx() so this should return 00000000

	retmsg = clientflushrx();
	if (!g_silent)
		printf("wipeout retmsg %08X\n", retmsg);

	// TODO use the ublast_* functions, but for now just cheat and write hex strings directly
	// see ../../openocd/zdebug.log.2 for the expected behavoir

	// Next read the chip ID. We have already sent TAP_RESET in ublast_initial_wipeout(), so the chip IR
	// will have been loaded with the IDCODE instruction (see https://www.fpga4fun.com/JTAG3.html)
	// So all we have to do is shift out the chip ID via DRSCAN...

	// The following is cribbed from the openocd zdebug.log2 which has...
	// ublast_scan(scan=DRSCAN, type=SCAN_IO, bits=672, buf=[ff..ff],end_state=3) HOWEVER the first sequence
	// actually reads 384 bits (the followup "e3ff..ff" has 0xE3 which reads 35 bytes = 280 bits, total=664),
	// then a bitbang sequence to read the final 8 bits (packed in 8 bytes). Seems inefficient. I have only
	// implemented the first sequence here, and that is still more than sufficent for the DE0_NANO which
	// has only one device in the TAP chain. NB the read returns ff..ff for the trailing bits (after the chip ID)
	// because we sent ff..ff for the bulk write data below (TODO test this by replacing some with 0's).

	// Send 64 bytes. The first 15 setup the DRSCAN mode (1110100) which leaves us in state DRSHIFT, then a mode
	// change byte 0xF0 (see detailed note above) which has bits 0x80 (set byte mode) 0x40 (read flag) and size
	// 0x30 (48 bytes = 384 bits)
	// NB 2C2D clocks TMS=0, 2E2F clocks TMS=1, the final 2C (before F0) sets TCK low again.

	respond("WX2e2f2e2f2e2f2c2d2e2f2c2d2c2d2cf0" // NB strings concatenated
			"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffZ");

	// Now read 48 (0x30) bytes matching the read count for the above write. NB we're still in byte mode,
	// so 8 bits are returned per byte. It's a bit overkill as the ID register is only 4 bytes, but I guess
	// it's allowing for a longer TAP chain (see above, the full scan of 672/32 implies up to 21 devices)

	clientflushrx();
	respond("RX30Z");
	io_check();

	// Confirm the ID in the message

	char idstr[16] = { 0 };
	unsigned int device_sc = 0, device_id;
	strncpy(idstr, g_clientmsg.mtext+4, 8);
	sscanf(idstr, "%x", &device_sc);

#ifndef __MINGW32__
	device_id = bswap_32(device_sc);
#else
	// MinGW does not have bswap_32() and I don't fancy including the winsock stuff just for htonl()
#define BSWAP32(n) (((n)>>24 & 0xff) | ((n)>>8 & 0xff00) | ((n)<<8 & 0xff0000) | ((n)<<24 & 0xff000000))
	device_id = BSWAP32(device_sc);
#endif

	// device_id = 0x12345678;	// TEST non-match

	*device_index = find_device(device_id);

	// printf("device_id [%s] %08x index %d\n", idstr, device_id, *device_index);

	if (*device_index)	// NB device_index=0 explicitly means not found
	{
//		printf("\n===================================\n");
		printf("FPGA identity MATCH OK ... %08x\n", device_id);
//		printf("===================================\n");
	}
	else
	{
		printf("\n===========================================================================\n");
		printf("WARNING FPGA identity CHIP ID %08x is not recognised (update devices.c)\n", device_id);
		printf("RBF programming will use default parameters which may not be appropriate\n");
		printf("===========================================================================\n");
		// return ERROR_BADCHIP;	// No, this would restrict us to ONLY the EP4CE22 chip
	}

	tap_reset();

	// Openocd now performs "core.c:1364 jtag_validate_ircapture(): IR capture validation scan"
	// ublast_state_move(): (from RESET to IRSHIFT)
	// ublast_scan(scan=IRSCAN, type=SCAN_IO, bits=12, buf=[ff0f], end_state=12)
	// NB 12 bits, even though openocd knows IR is only 10 bits

	// Set IRSCAN -> IRCAPTURE -> IRSHIFT then shift 8 bits in bytemode (with read flag)
	respond("WX2e2f2e2f2c2d2e2f2e2f2c2d2c2d2cc1ffZ");	// 1101100 then c1ff for bytmode/read 8 bits ff

	clientflushrx();
	respond("RX01Z");	// read a single byte
	io_check();

	// These are the low bits of the IR

	if (strncmp(g_clientmsg.mtext+1, "KRX55Z", 6))
	{
		printf("FPGA IR validate ERROR expected 0x55 (KRX55Z) got %s\n", g_clientmsg.mtext+1);
		return ERROR_IRVALIDATE;
	}

	// ublast_read_bitbang_tdos(buf=<pointer>, num_bits=4)
	// Bitbang TDI=1111 with TMS=0 (+read) then TDI=1 with TMS=1 (no read) NB TDI is low bit of high nibble

	clientflushrx();
	respond("WX3c7d3c7d3c7d3e7f3eZ");	// NB read flag is set on the TCK=high byte (7d), but zero for TCK=low (3c)
	io_check();

	// Now get the high bits of the IR (returning 4, even though register is 10 bits not 12)

	// ublast_buf_read(): (size=4, buf=[03020303]) -> 4

	clientflushrx();
	respond("RX04Z");	// read 4 bytes (NB bitbang, 03=one, 02=zero)
	io_check();

	if (strncmp(g_clientmsg.mtext+1, "KRX03020303Z", 12))	// NB LSB is on LEFT
	{
		printf("\n=========================\n");
		printf("FPGA IR validate ERROR expected 1011 (KRX03020303Z) got %s\n", g_clientmsg.mtext);
		return ERROR_IRVALIDATE;
	}

	// The validated pattern is (11) 0101010101 (msb to lsb) 0x155
	printf("FPGA IR validate 0x155 OK\n");
//	printf("=========================\n\n");

	return 0;
}

