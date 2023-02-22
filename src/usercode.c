/* usercode.c - custom interface to fpga

TODO ...
Write (more) functions for common steps.
Add a softcore CPU and control it via flags.
Add jtag uart functionality (hosting).

*/

#include "common.h"

// Bitstream ientifiers
#define CHIPID_v1 0x97d2f9ce
#define CHIPID_v2 0x97d2f9cf

// Instruction Register opcodes, see fpga/txrxmem/defines.v
#define IIDENT	0
#define IRADDR	1
#define IWADDR	2
#define IRDATA	5
#define IWDATA	6
#define IFLAGS	14
#define IBYPASS	15	(actually anything except the above acts as a bypass opcode)

// Debug flags (sets LED source), see fpga/txrxmem/system.v
// This is clunky due to lazy decoding in system.v, saved time then, pay for it now :-(

#define FLAGS_v1_DEBUG  0x100
#define FLAGS_v1_RADDR  0x200
#define FLAGS_v1_WADDR  0x400
#define FLAGS_v1_WDATA  0x800	// yeah, the order was a bit wrong too
#define FLAGS_v1_RDATA  0x1000

#define FLAGS_v2_DEBUG  0x100
#define FLAGS_v2_RADDR  0x200
#define FLAGS_v2_WADDR  0x300
#define FLAGS_v2_RDATA  0x400
#define FLAGS_v2_WDATA  0x500

// These are now variables (assigned later based on returned chip id)
unsigned int FLAGS_DEBUG;
unsigned int FLAGS_RADDR;
unsigned int FLAGS_WADDR;
unsigned int FLAGS_WDATA;
unsigned int FLAGS_RDATA;

#define FLAGS_TMODE 0x00010000

#define MEMSIZE 16384	// 32 bit words

#if MEMSIZE & (MEMSIZE-1)
#error "MEMSIZE must be a power of 2"	// This is relied on for address validation
#endif

static int get_hub_info(void)
{
	// Now exercise the Virtual Jtag, note the USER0 = 0xE and USER1 = 0xC values (use ONLY these)
	// NB USER1 (VIR) = 0x00E and USER0 (VDR) = 0x00C (10 bits each)

	// Specification for reading hubinfo is Altera manual ug_virtualjtag-683705-666577.pdf pages 33-34

	unsigned int buildinfo = 0;
	unsigned int hubinfo = 0;
	unsigned int nodeinfo = 0;

	tap_reset();	// Send TAP_RESET
	runtest5();		// Move from TAP_RESET to RUN/IDLE

	IRSHIFT_USER1();

	// Write 8 zeros to VIR (NB it's actually 5 bits but it is permissible to overscan when fetching hub info)
    respond("WX2e2f2e2f2e2f2c2d2c2d2c2c6d2c6d2c6d2c6d2c6d2c6d2c6d2e6f2eZ");	// PAUSEIR to SHIFTDR 8 zeros
    // TMS        1   1   1   0   0    data+read                    1  // ends at EXIT1DR

    respond("WX2e2f2c2d2cZ");	// from EXIT1DR to RUNIDLE
    // TMS        1   0

	respond("RX08Z");	// read 8 bytes (expect 0202020202020202, also 8 zeros)
	io_check();

	// Now scan out 32 bits from VDR as 8 nibbles

	// BEWARE Altera manual ug_virtualjtag-683705-666577.pdf specifies DRSCAN of one nibble at a time.
	// I tried scanning 32 bits in openocd and it FAILED! So the requirement is neccessary. AHA the
	// need for UPDATE_DR is mentioned, which explains it (the sequences below go via RUN/TEST so
	// this requirement is satisfied) ...

/* QUOTE: The HUB IP configuration register is shifted out using eight four-bit nibble scans of the
DR register. Each four-bit scan must pass through the UPDATE_DR state before the
next four-bit scan. The 8 scans are assembled into a 32-bit value with the definitions
shown in the table below */

	IRSHIFT_USER0();				// RUNIDLE to PAUSEIR

	scan_dr_int(0, 4, READMODE);	// PAUSEIR to RUNIDLE

	respond("RX04Z");	// read 4 bytes
	io_check();
	buildinfo = (buildinfo >> 4) | (print_nibble() << 28);

	// The remaining nibbles can loop (but not the first as the DRSCAN is different)
	// Also scan out the node info for the first node (an extra 8 nibbles). If there were
	// more nodes they would be scanned out in the same way (just increase the loop end value)

	for (int i=1; i<16; i++)
	{
        // TMS        1   0
        respond("WX2e2f2c2d2c2d2c2c6d2c6d2c6d2e6f2eZ");	// RUNIDLE to DRSCAN 4 bits
        // TMS        1   0   0    data+read    1  // ends at EXIT1DR

        respond("WX2e2f2c2d2cZ");	// from EXIT1DR to RUNIDLE
        // TMS        1   0

		respond("RX04Z");	// read 4 bytes
		io_check();
		buildinfo = (buildinfo >> 4) | (print_nibble() << 28);

		if (i==7)
		{
			hubinfo = buildinfo;
			buildinfo = 0;
		}
	}

	nodeinfo = buildinfo;

	// NB a FPGA without a Virtual Jtag module will return 0xffffffff for both hubinfo and nodeinfo

	printf("====================\n");
	printf("hubinfo = 0x%08x\n", hubinfo);
	printf("====================\n");

	unsigned int expected_hubinfo = 0x08086e04;
	if (hubinfo != expected_hubinfo)
	{
		// Value depends on the FPGA virtual jtag configuration vis VIR size, address size (number of instances)
		printf("\n==================================================\n");
		printf("WARNING hubinfo does not match expected %08x\n", expected_hubinfo);
		printf("Check and update expected value if this is correct\n");
		printf("==================================================\n");
	}

	printf("\n=====================\n");
	printf("nodeinfo = 0x%08x\n", nodeinfo);
	printf("=====================\n");

	// Decode the hub and node info

	printf("\nhub info m = %d mfg = 0x%x n = %d version = %d\n",
				hubinfo & 0xff, (hubinfo>>8) & 0x1ff, (hubinfo>>19) & 0xff, (hubinfo>>27) & 0x1f);

	printf("\nnode info inst = %d mfg = 0x%x id = %d version = %d\n\n",
				nodeinfo & 0xff, (nodeinfo>>8) & 0x1ff, (nodeinfo>>19) & 0xff, (nodeinfo>>27) & 0x1f);

	if (hubinfo != expected_hubinfo)
		return ERROR_FAIL;

	return 0;

}	// end get_hub_info()

static int vjtag_test(int vir, int vdr)
{
	printf("Starting vjtag_test vir %d vdr %d\n", vir, vdr);

	tap_reset();	// Send TAP_RESET
	runtest5();		// Move from TAP_RESET to RUN/IDLE

	IRSHIFT_USER1();

	// Write 5 bits VIR, address bit = 1 (msb) plus 4 instuction code bits.
	// For system_basic.rbf these 4 bits will display one LEDS MSB.

	// TDI is lsb of top nibble, vis 0x10 (so even nibbles are 0, odd are 1)
	// Together with the read bit, this encodes as 6 or 7 followed by 2 or 3 in next byte
	// TODO encode these via function (for now just hardcode a few)...

	scan_dr_int(0x10 | vir, 5, READMODE);	// PAUSEIR to SHIFTDR 00001 (addr=1 is MSB, hence 0x10)

	respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
	io_check();

	// Now load 5 bits into vdr - NB 4 bits are LEDs, the top bit is an enable so must be set.
	/*
		reg [NR_GPIOS:0]  gpio_dr;				// 5 bit DR

		if (update_dr)
			begin
			if (gpio_dr[NR_GPIOS]) begin		// Test MSB for enable
				gpio_outputs    <= gpio_dr[NR_GPIOS-1:0];
			end
		end
	*/

	IRSHIFT_USER0();

#if 0 	// KEEP this as a comment since referenced in io_check() as an example of how to cause a TIMEOUT

	// This is WRONG, do not read IR... causes io_check() timeout since was no read data available
	respond("RX04Z");	// read 4 bytes (captured IR)
	io_check();
#endif

	scan_dr_int(0x10 | vdr, 5, READMODE);	// PAUSEIR to SHIFTDR, top bit is enable so set 0x10, bottom 4 are leds

	respond("RX05Z");	// read 5 bytes (should be 0302020302 = 9 as verilog DRCaptures a const)
	io_check();
	return 0;

}	// end vjtag_test ()

int fpga_vjtag(void)
{
	printf("\nExercising fpga/vjtag\n\n");

	int dsec = 1000 * 1000;

	// test vir
	int startat = 0;
	// startat = 12;	// DEBUG shorten tests by setting higher start value (up to 15)
	for (int i=startat; i<16; i++)
	{
		vjtag_test(i,0);
		JTAGGER_SLEEP(dsec);
	}

	// test vdr ... NB need VIR=3 (see verilog EXTEST mode) to enable VDR loading to LEDs
	// plus the MSB set to enable LED output (this is added in vjtag_test() so no need here)
	for (int i=startat; i<16; i++)	
	{
		vjtag_test(3,i);
		JTAGGER_SLEEP(dsec);
	}

	vjtag_test(3,0);	// DR Leds off
	vjtag_test(0,0);	// IR Leds off

	tap_reset();
	jflush();

	return 0;
}

// NOTE the bulk transfer functions may appear to belong in jtagger.c BUT they rely on user #defines IRDATA and IWDATA
// which do NOT belong in jtagger.c so they will have to stay here (at least until I think of a better way)

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

static void bulk_transfer(char *mem, unsigned int len, int mode)
{
	// Setup is same as scan_vir_vdr(4, 32, ...)

	int vir = mode ? IWDATA : IRDATA, irlen = 4;

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

void bulk_write(char *mem, unsigned int len)
{
	bulk_transfer(mem, len, 1);
}

void bulk_read(char *mem, unsigned int len)
{
	if (len % 4)
		DOABORT("length must be a multiple of 4");

	if (len > 72 * 4)
		DOABORT("length cannot exceed 288 bytes");	// Seems to be a FTDI limit, TODO check documentation

	bulk_transfer(mem, len, 0);
}

int fpga_txrxmem(char *uparams, unsigned int chipid)
{
	printf("\nExercising fpga/txrxmem\n\n");

	tap_reset();
	runtest5();

	unsigned long long vdr_ret = 0;

	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_DEBUG, NOREADMODE);		// Set LEDs to debug (4 bits waddr, wdata)

	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0x0, NOREADMODE);	// Set read address 0
	vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0, NOREADMODE);	// Set write address 0

	// Use current time as prefix to debug what data is written and when
	int prefix = (time(NULL) & 0xffff) << 16;

	int count = 9;
	for (int i=0; i<count+1; i++)
	{
		printf("write addr %d val %08x", i, prefix+i);
		vdr_ret = scan_vir_vdr(4, 32, IWDATA, prefix+i, READMODE);	// Write values (address auto increments)
		printf(" returned vdr %08" PRIx64 "\n", (int64_t)vdr_ret);
		if (uparams && strchr(uparams, 'w'))
			JTAGGER_SLEEP(1000 * 1000);	// watch addr/data on LEDS (needs FLAGS_DEBUG, see above)
	}

	printf("\n");

	int addr = 0;

	// Readback the ram values ...
	for (int i=0; i<=count+2; i++)
	{
		vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0, READMODE);	// Read values (address auto increments)
		printf("addr %08x returned vdr %08" PRIx64 "\n", addr, (int64_t)vdr_ret);
		addr++;
	}

	printf("\nRANDOM numbers ...\n");

	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0x0, NOREADMODE);	// Set read address 0
	vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0, NOREADMODE);	// Set write address 0

	srand(1);

	for (int i=0; i<=count; i++)
	{
		unsigned int n = rand();
		printf("write addr %d val %08x", i, n);
		vdr_ret = scan_vir_vdr(4, 32, IWDATA, n, READMODE);	// Write values (address auto increments)
		printf(" returned vdr %08" PRIx64 "\n", (int64_t)vdr_ret);
	}

	printf("\n");

	srand(1);

	for (int i=0; i<=count; i++)
	{
		unsigned int n = rand();
		vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0, READMODE);	// Read values (address auto increments)
		printf("addr %d returned %08" PRIx64 " expected %08x %s\n", i, (int64_t)vdr_ret, n,
				vdr_ret == n ? "MATCH" : "BAD");
	}

	// vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0, READMODE);
	// printf("post read VDR returned %08" PRIx64 "\n", (int64_t)vdr_ret);

	// COMMENT it's slow, but that's because we're bitbanging and not buffering. Fixing that should
	// give speeds similar to the RBF programming, vis 700kB in 2 seconds... and it DOES !!!!

	// Test bulk uploading
	unsigned int mem[MEMSIZE];

	srand(2);	// NB different since we alredy have srand(1) data loaded

	for (int i=0; i<MEMSIZE; i++)
		mem[i] = rand();

	vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0, NOREADMODE);	// Set write address 0

	int membytes = MEMSIZE * sizeof(int);	// BEWARE is int always 4 bytes? Perhaps not in windows bit? TODO CHECK.

	// Upload is FAST as we're continuously shifting (EXACTLY the same as programming the bitstream)

	if (uparams && strchr(uparams, 's'))
	{
		int iter = 100;
		printf("\nBulk upload %d bytes speedtest %d iterations...\n", membytes, iter);
		time_t tstart, tfinish;
		time(&tstart);
		for (int i=0; i<iter; i++)	// NOT while() since want iter below
		{
			vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0, NOREADMODE);	// Set write address 0
			bulk_write((char*)mem, membytes);	// NB size is in bytes
			if (i%10 == 9)
				printf("done %d iterations of %d\n", i+1, iter);
		}
		time(&tfinish);
		printf("... done %" PRId64 " bytes in %" PRId64 " seconds = %"  PRId64 " bytes/sec\n\n",
			 iter * (int64_t) membytes, (int64_t)(tfinish - tstart),
			 iter * (int64_t) membytes / (tfinish - tstart));
	}
	else
	{
		printf("\nBulk upload %d bytes...\n", membytes);
		bulk_write((char*)mem, membytes);	// NB size is in bytes
		printf("... done\n\n");
	}
	
	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0, NOREADMODE);	// Set read address 0

	// Readback is SLOW as we're bitbanging, so just do random locations (not so SLOW any more)
	printf("Testing random locations...\n");	//  Not quite so SLOW any more

	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_RADDR, NOREADMODE);	// Set LEDs to read address for blinkenlights

	if (uparams && strchr(uparams, 's'))
		count = 16;		// Reduced so as to leave the speed test result on the screen
	else
		count = 64;

	while (count > 0)
	{
		// sanitize addr since using as index to mem[MEMSIZE] (BEWARE MEMSIZE must be power of 2)
		int addr = rand() & (MEMSIZE-1);

		int psize = 72;	// Packet size, see bulk_transfer(), counting by ints (so psize=60/4=15)
						// NB anything above 72 returns a FTDI error, possibly a limit TODO check documentation.
		unsigned int tmem[psize+1];	// Temp buffer to hold result

		// for (int i=0; i<psize; i++)	// DEBUG init to 0
		//	tmem[i] = 0;

		if (count < psize)
			psize = count;

		// This is a bit lazy as I don't handle wrapping, instead just defer to single mode. TODO fix.
		if (addr < MEMSIZE - 1 - psize)
		{
			vdr_ret = scan_vir_vdr(4, 32, IRADDR, addr, NOREADMODE);// Set read address
			bulk_read((char*)tmem, psize * 4);
			for (int i=0; i<psize; i++)
			{
				// NB this is a contiguous block of psize addresses
				unsigned int vdr_ret = tmem[i];
				unsigned int n = mem[addr + i];
				// Keep int64_t / PRIx64 for consistency with the one below
				printf("addr %04x returned %08" PRIx64 " expected %08x %s\n", addr+i, (int64_t)vdr_ret, n,
					vdr_ret == n ? "MATCH" : "BAD");
			}
			count -= psize;
		}
		else
		{
			// Address near end of mem[], do singly
			int addr = rand() & (MEMSIZE-1);
			unsigned int n = mem[addr];
			vdr_ret = scan_vir_vdr(4, 32, IRADDR, addr, NOREADMODE);// Set read address
			vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0, READMODE);	// Read value
			printf("addr %04x returned %08" PRIx64 " expected %08x %s\n", addr, (int64_t)vdr_ret, n,
					vdr_ret == n ? "MATCH" : "BAD");
			count--;
		}	
	}

	if (uparams && strchr(uparams, 'x'))
	{
	    printf("FTDI read cumulative %d.%.9" PRId64 " seconds\n", (int)g_cumulative_read_time.tv_sec,
				 (int64_t)g_cumulative_read_time.tv_nsec);

		printf("\nStress test, CONTROL-C to quit\n");	// TODO add signal handler and tap_reset() on quit

		int coverage[MEMSIZE] = { 0 };

		// int64_t elapsed_sec = 0;	// Now using clock_gettime() instead
		// time_t tstart, tfinish;
		struct timespec telapsed;

		for (int iter = 1; /* empty */ ; iter++)
		{
			// uncomment if NOT using chatty coverage print below
			// printf("Iteration %d\r", iter);	// \r not \n is deliberate
			// fflush(stdout);

			// time(&tstart);		// Now using clock_gettime() instead
		    struct timespec tstart, tfinish, tdelta;
		    clock_gettime(CLOCK_MONOTONIC, &tstart);

			srand(iter);	// Send different data each time

			for (int i=0; i<MEMSIZE; i++)
				mem[i] = rand();

			vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0, NOREADMODE);	// Set write address 0

			bulk_write((char*)mem, membytes);	// NB size is in bytes
			
			vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0, NOREADMODE);	// Set read address 0

			int psize = 72;	// Packet size, see bulk_transfer(), counting by ints (so psize=60/4=15)
							// NB anything above 72 returns a FTDI error, possibly a limit TODO check documentation.
			unsigned int tmem[psize+1];	// Temp buffer to hold result

			int golarge = 0;	// Yeah, it's now fast enough to do it all at once (approx 4 seconds per iteration)
								// But I'll leave it off by default as the other one looks more impressive

			if (uparams && strchr(uparams, 'L'))
				golarge = 1;	// If you really want it

			// count = 1024;	// OLD - Reads per iteration (checking count/MEMSIZE = 1/16 of total locations)
			count = (1024 / 72) * 72 + 72;	// Round it up to nearest psize should be marginally more efficient

			if (golarge)
				count = MEMSIZE;

			int addr = 0;

			while (count)
			{
				int nreads = psize;
				if (count < nreads)
					nreads = count;

				if (!golarge)
				{
					// random address (different each iteration since seed changes above)
					addr = rand() & (MEMSIZE-1);	// sanitize since using as index to mem[MEMSIZE]
													// BEWARE MEMSIZE must be power of 2 (checked above via #error)
					if (addr == 1)
						addr = 0;	// rand() does not return 0, so cover it at 1
				}

				if (addr >= MEMSIZE)
					DOABORT("bad address");

				// This is a bit lazy as I don't handle wrapping, instead just read to end of memory. TODO fix.
				int excess = addr + nreads - MEMSIZE;
				if (excess > 0)
					nreads -= excess;
				if (nreads < 1 || addr + nreads > MEMSIZE)	// Yeah, a bit belt&braces but brain fuzz has set in
				{
					printf("addr %x excess %d nreads %d\n", addr, excess, nreads);
					DOABORT("bad excess");
				}

				if (addr >= MEMSIZE)
					DOABORT("bad address");	// Just to be absolutely sure!

				vdr_ret = scan_vir_vdr(4, 32, IRADDR, addr, NOREADMODE);// Set read address
				bulk_read((char*)tmem, nreads * 4);

				for (int i=0; i<nreads; i++)
				{
					// NB this is a contiguous block of nreads addresses
					unsigned int vdr_ret = tmem[i];
					unsigned int n = mem[addr + i];
					coverage[addr + i]++;

					// if (iter==3 && i==nreads/2)	// Force a failure to confirm check is working
					//	vdr_ret++;
					if (vdr_ret != n)
					{
						printf("\nFAIL at addr %04x returned %08" PRIx64 " expected %08x %s\n", addr+i, (int64_t)vdr_ret, n,
								vdr_ret == n ? "MATCH" : "BAD");
						tap_reset();
						exit(1);
					}
				}

				count -= nreads;
				if (golarge)
					addr += nreads;
			}

			// time(&tfinish);
		    clock_gettime(CLOCK_MONOTONIC, &tfinish);

			// if (!g_silent)	// not appropriate as -v is way too verbose
			// quiet ie print if no uparams (redundant since needs 'x') or no 'q'
			if (golarge || !(uparams && strchr(uparams, 'q')))
			{
#if 0	// OLD time_t version
				elapsed_sec += tfinish - tstart;
				printf("Elapsed %" PRId64 " seconds cumulative %" PRId64 " sec", (int64_t)(tfinish - tstart), elapsed_sec);

		    	printf(" FTDI read cumulative %d.%.9" PRId64 " sec\n",
					(int)g_cumulative_read_time.tv_sec, (int64_t)g_cumulative_read_time.tv_nsec);
#else	// NEW clock_gettime version
   				sub_timespec(tstart, tfinish, &tdelta);
   				add_timespec(telapsed, tdelta, &telapsed);
			    printf("Elapsed %.2f seconds cumulative %.2f sec",
							(int)tdelta.tv_sec + (double)tdelta.tv_nsec / (double)NS_PER_SECOND,
							(int)telapsed.tv_sec + (double)telapsed.tv_nsec / (double)NS_PER_SECOND);
		    	printf(" FTDI read cumulative %.2f sec\n",
					(int)g_cumulative_read_time.tv_sec + (double)g_cumulative_read_time.tv_nsec / (double)NS_PER_SECOND);
#endif
			}

			// if (iter == 1 || iter%10 == 0)	// uncomment for less chat
			{
				// NOTE coverage just indicates what percentage of memory locations have been read back.
				// It is not an indication of the "fullness" of the stress test as we're testing JTAG I/O
				// and not the integrity of the block RAM! It takes 38 iterations for 90%, 49 for 95%

				int count = 0;
				for (int i=0; i<MEMSIZE; i++)
					if (coverage[i])
						count++;
				// if (count < MEMSIZE) || !golarge)	// No point as it's 100% with golarge anyway
				if (count < MEMSIZE)
					printf("Coverage at iteration %d = %3.2f%%\n", iter, (double)((100.0*count)/MEMSIZE));

#if 0
				// Find the bad'un 'cos without golarge it doesn't reach 100%
				if (count > MEMSIZE-2)
					for (int i=0; i<MEMSIZE; i++)
						if (!coverage[i])
							printf("Bad'un %x\n", i);	// It's zero! I suppose rand() don't give that one.
#endif
			}
		}
	}

	printf("FTDI read cumulative %d.%.9" PRId64 " sec\n",
		(int)g_cumulative_read_time.tv_sec, (int64_t)g_cumulative_read_time.tv_nsec);

	// At exit...
	tap_reset();
	jflush();

	return 0;
}

int fpga_txrxmem_timing(char *uparams, unsigned int chipid)
{
	printf("\nRunning fpga/txrxmem TIMING TEST\n");

	if (uparams && strcmp(uparams,"t"))	// Anything except exactly one 't' in uparams string
		printf("WARNING all other test options are ignored in this mode\n");

	printf("\n");

	if (chipid != CHIPID_v2)
		DOABORT("unexpected chipid");

	time_t tstart, tfinish;
	time(&tstart);

	tap_reset();
	runtest5();

	unsigned long long vdr_ret = 0;

	// Clear FLAGS_TMODE to ensure address is reset after a previous run
	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_DEBUG, NOREADMODE);

	// Set the timimg test mode and LEDs to debug (4 bits waddr, wdata)
	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_TMODE | FLAGS_DEBUG, NOREADMODE);

	// All JTAG operations are now logged to ram
	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0, NOREADMODE);	// Set read address 0

	// TODO Add more operations here that we want to see logged eg a short bulk_write()

	// Readback the ram

	// This is so very old-skool output compared to ModelSim/GtkWave, best to redirect this to a file.
	// BEWARE the current verilog wraps memory from 0x3fff to 0x0000. TODO add a flag to optionally inhibit this.

	printf("\ntime 50MHz/20nS     delta cdr sdr pdr udr tdo tdi tms tck\n");

	unsigned int mem[MEMSIZE];	// Save the result in case we want to do more later
	int addr = 0;
	int count = 500;
	int prev_time = 0;

	for (int i=0; i<=count; i++)
	{
		addr &= MEMSIZE-1;	// sanitize since using as index to mem[MEMSIZE] (BEWARE MEMSIZE must be power of 2)

		vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0, READMODE);	// Read values (address auto increments)
		// printf("addr %08x returned vdr %08" PRIx64 "\n", addr, (int64_t)vdr_ret);
		mem[addr] = vdr_ret;

		// Analyse the results (which are actually logging this readback, which is a bit meta)
		// Write as we go, else get a looooong pause while reading from jtag

		int time = mem[addr] >> 8;
		int sigs = mem[addr] & 0xff;
		int delta = time - prev_time;
		if (prev_time == 0)
			delta = 0;				// Startup (and a tiny chance of wrap around)
		if (delta < 0)
			delta += 0x01000000 ;	// Wrapped

		printf("       %8d  %8d   %d   %d   %d   %d   %d   %d   %d   %d\n", time, delta,
					 (sigs&(1<<7)) ? 1 : 0,
					 (sigs&(1<<6)) ? 1 : 0,
					 (sigs&(1<<5)) ? 1 : 0,
					 (sigs&(1<<4)) ? 1 : 0,
					 (sigs&(1<<3)) ? 1 : 0,
					 (sigs&(1<<2)) ? 1 : 0,
					 (sigs&(1<<1)) ? 1 : 0,
					 (sigs&1) ? 1 : 0);

		prev_time = time;
		addr++;
	}

	/* CONCLUDE there are some horrible pauses, eg

        2848637        21   0   1   0   0   1   0   0   0
        2848657        20   0   1   0   0   1   0   0   1
        2848678        21   0   1   0   0   1   0   0   0
        2848699        21   0   1   0   0   1   0   0   1
       12728502   9879803   0   0   0   0   0   0   1   1
       12728523        21   0   0   0   0   0   0   1   0
       12728565        42   0   0   0   0   0   0   0   0
       12728586        21   0   0   0   0   0   0   0   1

	9879803 * 20nS = 198mS. It's NOT caused by usleep (see common.h for debug JTAGGER_SLEEP macro) and not terminal
	I/O since I redirected to a file. Could it be FTDI read timeout? Other than those delays, tck runs at cycle time
	of approx 8 * 20nS = 160nS = 6MHz in bulk transfer mode, 42 * 20nS = 840nS = 1.1MHz in bitbang mode.

	The long pauses are the cause of the slow read performance. Need to get to the bottom of this...
	I think it's the FTDI driver buffering reads, TODO check the libftdi documentation for any tunable parmeters.

	*/

	time(&tfinish);
	printf("Elapsed %" PRId64 " seconds", (int64_t)(tfinish - tstart));

	printf(" FTDI read cumulative %d.%.9" PRId64 " sec\n",
		(int)g_cumulative_read_time.tv_sec, (int64_t)g_cumulative_read_time.tv_nsec);

	// At exit...
	tap_reset();
	jflush();

	return 0;
}

int usercode(char *uparams)
{
	printf("Starting usercode\n\n");

	if (get_hub_info())
	{
		printf("Unrecognised FPGA configuration, NOT proceeding with testing.\n");
		printf("You should load a supported bitstream.\n");
		return 5;	// TODO #define it
	}

	tap_reset();	// Send TAP_RESET
	runtest5();		// Move from TAP_RESET to RUN/IDLE

	// Determine which bitstream has been loaded, assumes 4 bit VIR
	// Select VIR opcode 0 (IIDENT in fpga/txrxmem, does nothing in fpga/vjtag)

	unsigned long long id = 0;

	IRSHIFT_USER1();

	// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
	scan_dr_int(0x10 | IIDENT, 5, READMODE);	// PAUSEIR to SHIFTDR (addr=1 is MSB, hence 0x10)

	respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
	io_check();

	IRSHIFT_USER0();

	int len = 32;
	scan_dr_int(0, len, READMODE);	// PAUSEIR to SHIFTDR

	char str[256];
	sprintf(str, "RX%02xZ", len);
	respond(str);	// read bitbangs
	io_check();

	id = get_bitbang(len,0);

	printf("VDR ident %08" PRIx64 "\n", (int64_t) id);

	if (id == CHIPID_v1 || id == CHIPID_v2)
	{
		printf("Found txtxmem OK\n");

		// chip id determines flag versions

		if (id == CHIPID_v1)
		{
			FLAGS_DEBUG = FLAGS_v1_DEBUG;
			FLAGS_RADDR = FLAGS_v1_RADDR;
			FLAGS_WADDR = FLAGS_v1_WADDR;
			FLAGS_WDATA = FLAGS_v1_WDATA;
			FLAGS_RDATA = FLAGS_v1_RDATA;

			if (uparams && strchr(uparams, 't'))
			{
				printf("WARNING timimg mode (-ut) is not supported for this bitstream\n");
				printf("Please load the newer version from fpga/txrxmem/system.rbf\n");
				JTAGGER_SLEEP(2000 * 1000);
				// But we run the tests anyway
			}
			return fpga_txrxmem(uparams, id);
		}

		if (id == CHIPID_v2)
		{
			FLAGS_DEBUG = FLAGS_v2_DEBUG;
			FLAGS_RADDR = FLAGS_v2_RADDR;
			FLAGS_WADDR = FLAGS_v2_WADDR;
			FLAGS_WDATA = FLAGS_v2_WDATA;
			FLAGS_RDATA = FLAGS_v2_RDATA;

			if (uparams && strchr(uparams, 't'))
				return fpga_txrxmem_timing(uparams, id);
			else
				return fpga_txrxmem(uparams, id);
		}

		DOABORT("notreached");
	}
	else
	{
		// Check for vjtag

#define EXTEST 3	// see fpga/vjtag/jtag_tap_defines.v

		tap_reset();
		runtest5();

		IRSHIFT_USER1();

		// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
		scan_dr_int(0x10 | EXTEST, 5, READMODE);	// PAUSEIR to SHIFTDR (addr=1 is MSB, hence 0x10)

		respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
		io_check();

		IRSHIFT_USER0();

		int len = 5;
		scan_dr_int(0, len, READMODE);	// PAUSEIR to SHIFTDR

		char str[256];
		sprintf(str, "RX%02xZ", len);
		respond(str);	// read bitbangs
		io_check();

		id = get_bitbang(len,0);

		printf("VDR ident %08" PRIx64 "\n", (int64_t) id);

		if (id == 0x9)	// Yeah, not very specific, but it is what is is
		{
			printf("Found vjtag OK\n");
			return fpga_vjtag();
		}
		else
			printf("Not found any supported virtual jtag hub\n");
	}

	tap_reset();
	jflush();

	return 0;
}

