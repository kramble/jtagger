/* usercode.c - custom interface to fpga

A bit repetitive, TODO write functions for common steps.

*/

#include "common.h"

#ifndef __MINGW32__
// This is a weird one as PRIx64 expands to "lx" not "llx" on 64 bit linux gcc
#undef PRIx64
#define PRIx64 "llx"
#endif

// Instruction Register opcodes, see fpga/txrxmem/defines.v
#define IRADDR	1
#define IWADDR	2
#define IRDATA	5
#define IWDATA	6
#define IFLAGS	14

// Debug flags (sets LED source), see fpga/txrxmem/system.v
#define FLAGS_DEBUG  0x100
#define FLAGS_RADDR  0x200
#define FLAGS_WADDR  0x400
#define FLAGS_WDATA  0x800
#define FLAGS_RDATA  0x1000

static int print_nibble(void)
{
	// eg g_clientmsg.mtext == "9KRX02020203Z"
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

static unsigned long long get_bitbang(unsigned int len, unsigned int shift)
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
		printf("value 0x%" PRIx64 "\n", n);
	return n;
}

static void IRSHIFT_USER0(void)	// Select VDR, entry RUNIDLE exit IRPAUSE 
{
	respond("WX2e2f2e2f2c2d2c2d2c810c2c2d2e2f2e2e2c2d2cZ");	// IRSHIFT USER0 0x0c
}

static void IRSHIFT_USER1(void)	// Select VIR, entry RUNIDLE exit IRPAUSE 
{
	respond("WX2e2f2e2f2c2d2c2d2c810e2c2d2e2f2e2e2c2d2cZ");	// IRSHIFT USER1 0x0e
}

static void DRSHIFT_RUNIDLE(void)	// from DRSHIFT to RUNIDLE
{
	respond("WX2e2e2f2c2d2cZ");
}

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
	respond("WX2e2f2e2f2e2f2c2d2c2d2c2c6d2c6d2c6d2c6d2c6d2c6d2c6d2e6f2eZ");	// IRPAUSE to DRSCAN 8 zeros

	clientflushrx();
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

	DRSHIFT_RUNIDLE();
	IRSHIFT_USER0();

	scan_dr_int(0, 4);

	clientflushrx();
	respond("RX04Z");	// read 4 bytes
	io_check();
	buildinfo = (buildinfo >> 4) | (print_nibble() << 28);

	// The remaining nibbles can loop (but not the first as the DRSCAN is different)
	// Also scan out the node info for the first node (an extra 8 nibbles). If there were
	// more nodes they would be scanned out in the same way (just increase the loop end value)

	for (int i=1; i<16; i++)
	{
		// respond("WX2e2e2f2c2d2cZ");	// from DRSHIFT to RUNIDLE) 
		DRSHIFT_RUNIDLE();
		respond("WX2e2f2c2d2c2d2c2c6d2c6d2c6d2e6f2eZ");	// RUNIDLE to DRSCAN 4 bits

		clientflushrx();
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

	scan_dr_int(0x10 | vir, 5);	// IRPAUSE to DRSCAN 00001 (addr=1 is MSB, hence 0x10)

	clientflushrx();

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

// Both of these work OK, so use the shorter one
#if 1
	DRSHIFT_RUNIDLE();
#else
	tap_reset();	// Send TAP_RESET
	runtest5();		// Move from TAP_RESET to RUN/IDLE
#endif

	IRSHIFT_USER0();

#if 0 	// KEEP this as a comment since referenced in io_check() as an example of how to cause a TIMEOUT

	// This is WRONG, do not read IR... causes io_check() timeout since was no read data available
	respond("RX04Z");	// read 4 bytes (captured IR)
	io_check();
#endif

	scan_dr_int(0x10 | vdr, 5);		// IRPAUSE to DRSCAN, top bit is enable so set 0x10, bottom 4 are leds

	clientflushrx();
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

	return 0;
}

long long unsigned scan_vir_vdr(unsigned int irlen, unsigned int vrlen, unsigned int vir, unsigned int vdr)
{
	long long unsigned vdr_ret = 0;

	// TODO speed this up

	tap_reset();
	runtest5();

	IRSHIFT_USER1();	// entry RUNIDLE exit IRPAUSE 

	// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
	scan_dr_int(0x10 | vir, irlen+1);	// IRPAUSE to RUNIDLE (addr=1 is MSB, hence 0x10)

	clientflushrx();

	respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
	io_check();

	DRSHIFT_RUNIDLE();
	IRSHIFT_USER0();

	scan_dr_int(vdr, vrlen);

	clientflushrx();
	char str[256];
	sprintf(str, "RX%02xZ", vrlen);
	respond(str);	// read bitbangs
	io_check();

	vdr_ret = get_bitbang(vrlen, 0);

	if (!g_silent)
		printf("scan_vir_vdr returned %08" PRIx64 "\n", vdr_ret);

	return vdr_ret;
}

static void bulk_upload(char *mem, unsigned int len)
{
	// Setup is same as scan_vir_vdr(4, 32, IWDATA ...)
	// TODO could buffer the setup (it won't give much speedup, and this is LIGHTNING fast anyway)

	int vir = IWDATA, irlen = 4;

	tap_reset();
	runtest5();

	IRSHIFT_USER1();

	// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
	scan_dr_int(0x10 | vir, irlen+1);	// IRPAUSE to DRSCAN (addr=1 is MSB, hence 0x10)

	clientflushrx();

	respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
	io_check();

	DRSHIFT_RUNIDLE();
	IRSHIFT_USER0();	// exits in IRPAUSE

	// Bulk upload is based on scan_dr_int(vdr, vrlen) and parse_rbf()
	// Scans data non-stop (does not pass through DRUPDATE), see jtag_vdr.v

	// Copied verbatim from parse_rbf()
	// NB unlike scan_dr_int() we do not set the readback flag

	char packbuf[BUF_LEN];	// NB BUF_LEN is message buffer
	*packbuf = 0;			// Set as empty

#if 0
	// This does not work properly (sends garbage)
	strcpy (packbuf, "WX2e2f2e2f2e2f2c2d2c2d2c2c");	// IRPAUSE to DRSHIFT
	char *dst = packbuf + strlen(packbuf);
#else
	// Do it EXACTLY like parse_rbf()
	respond("WX2e2f2e2f2e2f2c2d2c2d2c2cZ");	// IRPAUSE to DRSHIFT
	char *dst = packbuf;
	strcpy(dst, "WX");
	dst += 2;
#endif

	char *p = mem;
	int byte = 0;
	while (p < mem + len)
	{
		if (byte == 0)
		{
			// packet header
			*dst++ = 'b';
			*dst++ = 'f';
		}

		unsigned char ch = *p++;

		// Two hex chars (bytes)
		*dst++ = TOHEX(ch>>4);
		*dst++ = TOHEX(ch);
		*dst = 0;	// Not necessary but makes debugging easier since can see end of data when printing buffer

		if (++byte > 62)
		{
			byte = 0;

			// Always operate in packmode
			size_t pblen = dst - packbuf;
			if (pblen > sizeof(packbuf) - 132)	// Tweak so it just fits
			{
				*dst++ = 'Z';
				*dst = 0;

				respond(packbuf);

				dst = packbuf;
				strcpy(dst, "WX");
				dst += 2;
				*dst = 0;	// Again, not necessary
			}
		}
	}

	send_residual(packbuf, dst);
	tap_reset();
}

int fpga_txrxmem(char *uparams)
{
	// TODO speed improvements eg buffer the FTDI I/O and avoid readback (clear the read flag in the write data),
	// use bulk byte loads instead of bitbang

	printf("\nExercising fpga/txrxmem\n\n");

	tap_reset();
	runtest5();

	unsigned long long vdr_ret = 0;

	// vdr_ret = scan_vir_vdr(4, 32, IFLAGS, 0x55);			// Set the LEDs to 0x55
	// vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_WADDR);	// Set LEDs to waddr
	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_DEBUG);		// Set LEDs to debug (4 bits waddr, wdata)

	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0x0);	// Set read address 0
	vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0);	// Set write address 0

	// Use current time as prefix to debug what data is written and when
	int prefix = (time(NULL) & 0xffff) << 16;

	int count = 9;
	for (int i=0; i<count+1; i++)
	{
		printf("write addr %d val %08x", i, prefix+i);
		vdr_ret = scan_vir_vdr(4, 32, IWDATA, prefix+i);	// Write values (address auto increments)
		printf(" returned vdr %08" PRIx64 "\n", vdr_ret);
		if (uparams && strchr(uparams, 'w'))
			JTAGGER_SLEEP(1000 * 1000);	// watch addr/data on LEDS (needs FLAGS_DEBUG, see above)
	}

	printf("\n");

	int addr = 0;

	// Readback the ram values ...
	for (int i=0; i<=count+2; i++)
	{
		vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0);	// Read values (address auto increments)
		printf("addr %08x returned vdr %08" PRIx64 "\n", addr, vdr_ret);
		addr++;
	}

	printf("\nRANDOM numbers ...\n");

	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0x0);	// Set read address 0
	vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0);	// Set write address 0

	srand(1);

	for (int i=0; i<=count; i++)
	{
		unsigned int n = rand();
		printf("write addr %d val %08x", i, n);
		vdr_ret = scan_vir_vdr(4, 32, IWDATA, n);	// Write values (address auto increments)
		printf(" returned vdr %08" PRIx64 "\n", vdr_ret);
	}

	printf("\n");

	srand(1);

	for (int i=0; i<=count; i++)
	{
		unsigned int n = rand();
		vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0);	// Read values (address auto increments)
		printf("addr %d returned %08" PRIx64 " expected %08x %s\n", i, vdr_ret, n,
				vdr_ret == n ? "MATCH" : "BAD");
	}

	// vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0);
	// printf("post read VDR returned %08" PRIx64 "\n", vdr_ret);

	// COMMENT it's slow, but that's because we're bitbanging and not buffering. Fixing that should
	// give speeds similar to the RBF programming, vis 700kB in 2 seconds... and it DOES !!!!

	// Test bulk uploading
#define MEMSIZE 16384	// 32 bit words
	unsigned int mem[MEMSIZE];

	srand(2);	// NB different since we alredy have srand(1) data loaded

	for (int i=0; i<MEMSIZE; i++)
		mem[i] = rand();

	vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0);	// Set write address 0

	int size = MEMSIZE * sizeof(int);

	// Upload is FAST as we're continuously shifting (EXACTLY the same as programming the bitstream)

	if (uparams && strchr(uparams, 's'))
	{
		int iter = 100;
		printf("\nBulk upload %d bytes speedtest %d iterations...\n", size, iter);
		time_t tstart, tfinish;
		time(&tstart);
		for (int i=0; i<iter; i++)	// NOT while() since want iter below
		{
			vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0x0);	// Set write address 0
			bulk_upload((char*)mem, size);	// NB size is in bytes
			if (i%10 == 9)
				printf("done %d iterations of %d\n", i+1, iter);
		}
		time(&tfinish);
// Don't know if this is a bug or not, but it's annoying
#ifndef __MINGW32__
#undef PRId64
#define PRId64 "lld"
#endif
		printf("... done %" PRId64 " bytes in %" PRId64 " seconds = %"  PRId64 " bytes/sec\n\n",
			 iter * (long long) size, (long long)(tfinish - tstart),
			 iter * (long long) size / (long long)(tfinish - tstart));
	}
	else
	{
		printf("\nBulk upload %d bytes...\n", size);
		bulk_upload((char*)mem, size);	// NB size is in bytes
		printf("... done\n\n");
	}
	
	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0x0);	// Set read address 0

	// Readback is SLOW as we're bitbanging, so just do random locations
	printf("Testing random locations (SLOW since bitbang)\n");

	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_RADDR);	// Set LEDs to read address for blinkenlights

	if (uparams && strchr(uparams, 's'))
		count = 16;		// Reduced so as to leave the speed test result on the screen
	else
		count = 64;

	for (int i=0; i<=count; i++)
	{
		int addr = rand() & 0x3FFF;
		unsigned int n = mem[addr];
		vdr_ret = scan_vir_vdr(4, 32, IRADDR, addr);// Set read address
		vdr_ret = scan_vir_vdr(4, 32, IRDATA, 0);	// Read value
		printf("addr %04x returned %08" PRIx64 " expected %08x %s\n", addr, vdr_ret, n,
				vdr_ret == n ? "MATCH" : "BAD");
	}

	// At exit...
	tap_reset();

	return 0;
}

int usercode(char *uparams)
{
	printf("Starting usercode\n\n");

	if (get_hub_info())
	{
		printf("NOT proceeding with vjtag_test()\n");
		return 5;
	}

	tap_reset();	// Send TAP_RESET
	runtest5();		// Move from TAP_RESET to RUN/IDLE

	// Determine which bitstream has been loaded, assumes 4 bit VIR
	// Select VIR opcode 0 (IIDENT in fpga/txrxmem, does nothing in fpga/vjtag)

	unsigned long long id = 0;

#define IIDENT 0	// see fpga/txrxmem/defines.v

	tap_reset();
	runtest5();

	IRSHIFT_USER1();

	// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
	scan_dr_int(0x10 | IIDENT, 5);	// IRPAUSE to DRSCAN (addr=1 is MSB, hence 0x10)

	clientflushrx();

	respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
	io_check();

	DRSHIFT_RUNIDLE();
	IRSHIFT_USER0();

	int len = 32;
	scan_dr_int(0, len);	// IRPAUSE to DRSCAN

	clientflushrx();
	char str[256];
	sprintf(str, "RX%02xZ", len);
	respond(str);	// read bitbangs
	io_check();

	id = get_bitbang(len,0);


	printf("VDR ident %08" PRIx64 "\n", id);

	if (id == 0x97d2f9ce)
	{
		printf("Found txtxmem OK\n");
		return fpga_txrxmem(uparams);
	}
	else
	{
		// Check for vjtag

#define EXTEST 3	// see fpga/vjtag/jtag_tap_defines.v

		tap_reset();
		runtest5();

		IRSHIFT_USER1();

		// NB need to scan 5 bits for 4 bit VIR as top bit is hub address
		scan_dr_int(0x10 | EXTEST, 5);	// IRPAUSE to DRSCAN (addr=1 is MSB, hence 0x10)

		clientflushrx();

		respond("RX05Z");	// read 5 bytes (expect 0202020202020202, 5 zeros)
		io_check();

		DRSHIFT_RUNIDLE();
		IRSHIFT_USER0();

		int len = 5;
		scan_dr_int(0, len);	// IRPAUSE to DRSCAN

		clientflushrx();
		char str[256];
		sprintf(str, "RX%02xZ", len);
		respond(str);	// read bitbangs
		io_check();

		id = get_bitbang(len,0);

		printf("VDR ident %08" PRIx64 "\n", id);

		if (id == 0x9)	// Yeah, not very specific, but it is what is is
		{
			printf("Found vjtag OK\n");
			return fpga_vjtag();
		}
		else
			printf("Not found any supported virtual jtag hub\n");
	}

	tap_reset();

	return 0;
}

