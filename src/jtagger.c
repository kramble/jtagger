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

#define MILLISECONDS 1000	// for JTAGGER_SLEEP (calls usleep)

int ftdi_ok;

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

	ublast_flush_buffer();	// MJ added this to write the TAP_RESET

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
			printf("seconf flush of read buffer retmsg %08X\n", retmsg);
	}
}

int tap_reset(void)	// NOT static since called from program()
{
	// Enter TAP_RESET state (valid from any state)
	respond("WX2e2f2e2f2e2f2e2f2e2f2e2eZ");
	return io_check();
}

int runtest5(void)	// NOT static since called from program()
{
	// Move from TAP_RESET to RUN/IDLE (NB all openocd sequences assume RUN/IDLE as start point)
	respond("WX2c2d2c2d2c2d2c2d2c2d2cZ");
	return io_check();
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

// #define TEST_SCAN_DR	// Enable test

#ifdef TEST_SCAN_DR		// DEBUG function, call from top of main() then exit
static char *test_scan_dr_int_expect;	// TESTING see test_scan_dr_int()
#endif

int scan_dr_int(unsigned int val, int bits)
{
	// Requires entry from RUN/TEST, exits in DRSHIFT mode (I think, TODO CHECK)
	// Require a minimum of 4 bits since using hub scan as template, the final two
	// bits needing special handing due to JTAG mode change

	if (bits < 4 || bits > 64)
	{
		printf("scan_dr ERROR invalid bits %d\n", bits);
		DOABORT ("invalid bits");
	}

	// Construct (5 bit example) "WX2e2f2e2f2e2f2c2d2c2d2c2c6d2c6d2c6d2c6d2e7f3eZ");

	char str[1024];	// Plenty for 32 bits (4 char per bit, plus leadin/tailout)

	char s[5];		// Construction zone
	s[4] = 0;

	strcpy (str, "WX2e2f2e2f2e2f2c2d2c2d2c2c");

	for (int i=0; i<bits; i++)
	{
		char c1 = '6';
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

	strcat (str, "Z");

#ifdef TEST_SCAN_DR		// TESTING, see test_scan_dr_int()
	if (test_scan_dr_int_expect)
	{
		printf("test %s\n", str);
		return strcmp(str, test_scan_dr_int_expect);
	}
	else
#endif
		respond(str);

	return 0;
}

#ifdef TEST_SCAN_DR		// DEBUG function, enabled via #define TEST_SCAN_DR (see above), but need to call it from main()
static int test_scan_dr_int()
{
	test_scan_dr_int_expect = "WX2e2f2e2f2e2f2c2d2c2d2c2c6d2c6d2c6d2c6d2e7f3eZ";
	if (scan_dr_int(0x10, 5))
		printf("FAIL %s\n", test_scan_dr_int_expect);

	test_scan_dr_int_expect = "WX2e2f2e2f2e2f2c2d2c2d2c2c6d2c6d2c6d2e6f2eZ";	// IRPAUSE to DRSCAN 4 bits
	if (scan_dr_int(0, 4))
		printf("FAIL %s\n", test_scan_dr_int_expect);

	// TODO add some more to be sure

	return 0;
}
#endif

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

static int init_fpga(int *device_index)
{
	// Checks server status, opens FTDI if needed, and checks jtag functionality (queries fpga chip id)

	printf("init_fpga\n");
	ftdi_ok = 0;	// Assume NOT connected OK - BEWARE main() may attempt connection but status is checked here

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

	ftdi_ok = 1;	// Used at final exit from main() to send TAP_RESET. NB set to 0 above at entry to jtagger()

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
		printf("\n===================================\n");
		printf("FPGA identity MATCH OK ... %08x\n", device_id);
		printf("===================================\n");
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

	printf("\n=========================\n");
	if (strncmp(g_clientmsg.mtext+1, "KRX03020303Z", 12))	// NB LSB is on LEFT
	{
		printf("FPGA IR validate ERROR expected 1011 (KRX03020303Z) got %s\n", g_clientmsg.mtext);
		return ERROR_IRVALIDATE;
	}

	// The validated pattern is (11) 0101010101 (msb to lsb) 0x155
	printf("FPGA IR validate 0x155 OK\n");
	printf("=========================\n\n");

	return 0;
}

static void usage(void)
{
	printf("Usage: jtagger --help -v -p filename.svf -r filename.rbf\n");
}

static void help(void)
{
	printf(
"Usage: jtagger --help -v -s -p filename.svf -r filename.rbf\n\n"
"A standalone jtag driver for the DE0-Nano (Quartus is not required).\n"
"Without options, jtagger prints the chip id and checks for a virtual jtag hub.\n"
"If a hub is found, the first instance is listed and some I/O is attempted.\n"
"See the vjtag verilog project for details (this is just an example, the jtagger\n"
"source should be modified to support your own system requirements).\n\n"

"OPTIONS\n"
"-v sets verbose mode.\n"
"-s communicates with a separate jtag server (see below).\n"
"-p will program a .svf file (default %s), likely BUGGY (use -r instead)\n"
"-r will program a .rbf file (default %s), must not be compressed.\n"
"NB only Altera/Intel Quartus .svf programming files are supported as the svf\n"
"parsing is very crude, tested on Quartus 10.1 (other versions may not work).\n\n"

"Jtagger was originally designed as a client/server on the presumpton of\n"
"improved performance, however that is now deemed unneccessary. It can still be\n"
"operated that way by starting \"jtagserver\" in a separate terminal window then\n"
"running jtagger with the -s switch. Programming is considerably slower in this\n"
"mode. The client/server employs SYSTEM V message queues (sorry) so use \"ipcs\"\n"
"for troubleshooting. The message PID/lock file is %s (may need\n"
"deleting to fix problems). It's really NOT worth bothering with client/server.\n\n"
"Employs code from OpenOCD and OpenFPGALoader under the GPL license.\n"
"You may find those projects more useful than jtagger which was written as a\n"
"personal project to drive a vitual jtag hub without needing Quartus installed.\n"
"Nevertheless, you may be pleasantly surprised at just how FAST it programs!\n"
, PROGRAMFILE_S, PROGRAMFILE_R, SOCKFILE);
}

int main (int argc, char **argv)
{
	// NB g_spoofprog is for DEBUG of program_fpga() without writing FTDI output
	// It skips init_fpga and disables respond() ... no messages are sent to server
	g_spoofprog = 0;

	if (g_spoofprog)
	{
		printf("==============================================================\n");
		printf("****** SPOOFING  SPOOFING  SPOOFING  SPOOFING  SPOOFING ******\n");
		printf("==============================================================\n");
	}

	g_strictrx = 1;		// Always use this now (non-strict is bad, keep option for debugging only)

	int device_index = 0;	// Holds index into device_params[] after fpga_init(), 0 means not found (uses default)

	// Process command line. TODO use getopt
	int verbose = 0;
	int filetype = FILETYPE_NONE;
	char *fname = NULL;

	while (argc > 1)
	{
		if (!argv || !argv[1])
			DOABORT ("argv");	// Should not happen

		if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "-?") || !strcmp(argv[1], "--help"))
		{
			// Print help and exit, other options are ignored (unless a prior was invalid)
			help();
			return 0;
		}

		if (!strcmp(argv[1], "-v"))
		{
			verbose = 1;
			argc--;
			argv++;
			continue;
		}

		if (!strcmp(argv[1], "-s"))
		{
			g_standalone = 0;
			argc--;
			argv++;
			continue;
		}

		if (!strncmp(argv[1], "-p", 2) || !strncmp(argv[1], "-r", 2))
		{
			if (filetype)
			{
				printf("ERROR multiple program options supplied\n");
				usage();
				return 1;
			}

			filetype = FILETYPE_SVF;
			fname = PROGRAMFILE_S;

			if (argv[1][1] == 'r')
			{
				filetype = FILETYPE_RBF;
				fname = PROGRAMFILE_R;
			}

			if (argv[1][2])			// a character following p/v indicates a concatenated filename
				fname = argv[1]+2;
			else if (argc > 2)		// filename is next option
			{
				fname = argv[2];
				argc--;
				argv++;
				// decrement again below
			}
			argc--;
			argv++;
			continue;
		}

		printf("option(s) not recognised or invalid\n");
		usage();
		return 1;
	}

	if (verbose)
		printf("verbose %d filetype %d fname [%s]\n", verbose, filetype, fname ? fname : "NULL");	// DEBUG

	if (!g_standalone)
	{
		if (initmessage())
			return 1;
	}

	if (filetype != FILETYPE_NONE)
	{
		if (!fname)
			DOABORT ("fname");

		if (!verbose)
			g_silent = 1;		// BEWARE may be overriden in program.c/parse()

		int ret = 0;

		if (g_spoofprog)
		{
			// NB skips init since just testing the program_fpga() without writing FTDI output
			// also disables respond() ... no messages are sent to server, but to be sure do this too
			g_spoofprog = 0;
			respond("MX07Z");	// Set g_mode = 0x03 (spoof actions 0x01 | spoof connected status 0x02) | 0x04 (silent)
			g_spoofprog = 1;
		}
		else
		{
			if (!verbose)
				respond("MX04Z");			// Set g_mode = 0x04 (silent)
			else
				respond("MX00Z");			// Set g_mode = 0x00 (normal) in case it was left set to debug mode
			ret = init_fpga(&device_index);
		}

		if (!ret)
			return (program_fpga(fname, filetype, device_index));

		return ret;
	}

	// NB program_fpga() does not reach here (always returns above)

	if (g_spoofprog)
	{
		// Not generally a good idea here as only intended for testing program_fpga()
		printf("====================================\n");
		printf("**** WARNING g_spoofprog is set ****\n");
		printf("This will likely hang at init_fpga()\n");
		printf("Mode is only useful with -p or -x   \n");
		printf("====================================\n");
	}

	if (!verbose)
		g_silent = 1;

	if (verbose)
		respond("MX00Z");		// Set g_mode = 0x00 (normal) in case it was left set to debug mode
	else
		respond("MX04Z");		// Set g_mode = 0x04 (silent)

	// The first run after connecting the DE0_NANO often fails, but this is taken care of in init_fpga()
	// by ublast_initial_wipeout(). However disconnecting and reconnecting the USB cable without stopping
	// and restarting the server can be problematic. Hence this loop attempts reconnection on errors.

	int ret = 0;
	int err = 0;
	for (int i=0; i<2; i++)		// Retry several times (currently one retry) on ERROR_BADCHIP
	{
		if (err)
		{
			// Try closing and reopening FTDI. This is no longer necessary since fixed in the ublast_initial_wipeout()
			// call, but no harm in keeping it. May be required in some circumstances, eg if disconnect and
			// reconnect without stopping server (so get FTDI errors on first try, second should reconnect driver)
			printf("Retrying close/open FTDI device\n");
			respond("UZ");
			if (!g_standalone)
				JTAGGER_SLEEP(1000 * MILLISECONDS);
			clientflushrx();
			respond("JZ");
			if (!g_standalone)
				JTAGGER_SLEEP(1000 * MILLISECONDS);
			clientflushrx();
			// NB ftdi_ok status is updated in jtagger()
		}
		ret = init_fpga(&device_index);
		if (ret)
		{
			printf("jtagger exit with ERROR %d\n", ret);
			if (ret != ERROR_BADCHIP && ret != ERROR_IRVALIDATE)	// Retry by closing/reopening FTDI
				break;	// Quit on any other errors
			err = 1;
		}
		else
		{
			ret = usercode();
			// printf("jtagger exit with status %d\n", ret);
			break;	// Omit this break in order to to loop regardless of successful result
		}
		JTAGGER_SLEEP (2000 * MILLISECONDS);	// 2 seconds between retries
	}

	// Finish with TAP_RESET (safe since server will reject if FTDI not open, but check ftdi_ok anyway
	// as server does complain which could be distracting)
	if (ftdi_ok)
		tap_reset();

	// Print timeout stats
	// NB if g_flushrx_timeout > 0, then g_flushrx_maxdelay will always be 99
	// printf("io_check timeouts %d maxdelay %d\n", g_flushrx_timeout, g_flushrx_maxdelay);

	printf("jtagger exit with status %d\n", ret);
	return ret;
}
