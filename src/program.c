/* program.c - read Serial Vector Format (.svf) file and (attempt to) program FPGA

For the SVF specification google svf_specification.pdf, for hints see openFPGALoader and OpenOCD
Altera's Jrunner has lots of JTAG stuff, including IR opcodes, chip ids and consts for determining
the scan length for the configuration status check (SDR 732 TDI in the svf example below), eg
	EP4CE22E22 {0x20F30DD, 244, 149, 10},	... NB mutiply by 3, so SDR 732 and check bit 447
I suspect this is doing a boundary scan and testing the CONFIG_DONE pin, even though the CHECK_STATUS (0x4)
opcode is not the same as the boundary scan codes. See the cyclone5_handbook.pdf which describes opcode functions,
including scary looking private "do not use, these may destroy chip" opcodes (O_o)

To generate a SVF from Quartus SOF file (using wine since Quartus is not natively installed on my linuxmint) ...
$ wine /media/mark/Vista/altera/10.1/quartus/bin/quartus_cpf.exe -c -q 12.0MHz -g 3.3 -n p system.sof system.svf
NB unlike .sof a .svf is a plain text file (though .sof does have a text header)

Raw Binary Files (.rbf) are created in the same way (see openFPGALoader/doc/vendors/intel). JTAG does not
support compression, use -r switch on openFPGALoader for rbf input) ...
quartus_cpf.exe -c --option=bitstream_compression=off system.sof system.rbf

From the spec (quote) ...
The SVF file is defined as an ASCII file that consists of a set of SVF statements.
The maximum number of characters allowed on a line is 256, although one SVF
statement can span more than one line. Each statement consists of a command
and associated parameters. Each SVF statement is terminated by a semicolon.
SVF is not case sensitive. Comments can be inserted into a SVF file after an
exclamation point ‘!’ or a pair of slashes ‘//’. Either ‘//’ or ‘!’ will comment out the
remainder of the line.

For lazy sake, I'm going to assume the following sequence from a simple DE0-Nano SVF...
FREQUENCY 2.40E+07 HZ;
TRST ABSENT;
ENDDR IDLE;
ENDIR IRPAUSE;
STATE IDLE;
SIR 10 TDI (002);
RUNTEST IDLE 24000 TCK ENDSTATE IDLE;
SDR 5748760 TDI (... lots of data, in REVERSE byte order);
SIR 10 TDI (004);
RUNTEST 120 TCK;
SDR 732 TDI (...) TDO (...) MASK (...);
SIR 10 TDI (003);
RUNTEST 120000 TCK;		... fpag initialises a short way into this sequence
RUNTEST 512 TCK;
SIR 10 TDI (3FF);		... set bypass mode
RUNTEST 24000 TCK;
STATE IDLE;

*/

#include "common.h"

// Parse error codes, see printerror() at the bottom for descriptions
#define PROGERR_FILE 101
#define PROGERR_BOUNDS 102
#define PROGERR_STAGE 103
#define PROGERR_CLOCKS 104
#define PROGERR_TDIDATA 105
#define PROGERR_SLASH 106
#define PROGERR_DATA 107
#define PROGERR_BYTES 108

// This is the large static buffer shared by both filetype versions of the programmer
// TODO allocate dynamically and resize as neccessary (mmap looks handy, but is not Windows compatible)
// #define HEXBUF (2 * 1024 * 1024)	// (Old value) sufficient for the De0-Nano Cyclone EP4CE22 FPGA
#define HEXBUF (16 * 1024 * 1024)	// It's bss so no harm making this a lot bigger (in case someone has a HUGE fpga)
static char buf[HEXBUF];			// Retain buffer from run==0 for run==magic call

static int g_lineno;			// Used in printerror() so global
static int g_databytes;
static unsigned int g_clocks;	// Reads value from "SDR 5748760 TDI"

// LAZY during debugging (TODO pass a struct around)
static int g_device_index;		// index into device_params[]
static int g_filetype;

#if 0
static void dump (char *buf, size_t n)	// DEBUG dump to file
{
	FILE *f = fopen("zdump.out", "wb");
	if (!f)
		DOABORT("write");
	int raw = 0;	// NB raw in the sense of the incoming data, which may actually be "cooked" as hex
	if (raw)
		fwrite(buf, n, 1, f);		// In the case of SVF, this actually dumps hex data
	else
	{
		// convert hex to binary
		char *p = buf;
		while (p < buf+n)
		{
			unsigned char b, c = 0;
			b = *p++;	// BEWARE do not use UNHEX() directly on p++ (else ++ will happen 6 times)
			c = UNHEX(b);
			b = *p++;
			c = c<<4 | UNHEX(b);
			putc(c, f);
		}
	}
	fclose(f);
}
#endif

static int begin_programming()
{
	// Perform initial steps. Originally based on an SVF file, but now just following openFPGALoader sequences

	// TODO fix this mess (it's basically just cribbed from a dumped FTDI parameters log of an
	// openFPGALoader session). Do it programatically instead.

	// ALSO if I'm to continue supporting SVF, fully parse the file and queue the correct actions (retain
	// the current sequence for RBF only).

	// Enter STATE IDLE

	tap_reset();	// Not actually needed - it works fine without it, I suspect because the next string starts 
	runtest5();		// 3e3f3e3f3e3f3e3f3e3f3e3f which is tap_reset (with TDI=1 cf TDI=0 in my version)

	// openFPGALoader (log file analysis) does this ...
	// It looks like tap reset then some checks (readback IR perhaps), then IRSCAN 0x2 PROGRAM (see the "8102" substring)
	respond("WX3e3f3e3f3e3f3e3f3e3f3e3f2e3c3d3e3f3c3d3c3d2c2cc4ffffffff2cc4ffffffff2cc4ffffffff2cc4ffffffff2cc3ffffff"
			"3c7d3c7d3c7d3c7d3c7d3c7d3c7d3e7f3e3e3f3e3f3e3f3e3f3e3f3e3f2e3e3f3e3f3e3f3e3f3e3f3e3f2e3c3d2c3e3f3e3f"
			"3c3d3c3d2c2c81022c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");

	// SIR 10 TDI (002);	... IRSCAN 0x2 PROGRAM (incuded in the above string)

	// RUNTEST IDLE 24000 TCK ENDSTATE IDLE;
	if (g_filetype == FILETYPE_SVF)	// RBF does not need it (neither does SVF but it wants it!)
		runtest(24000);

	// SDR 5748760 TDI (....)
	respond("WX2c3e3f3c3d3c3d2c2cZ");	// TMS 1,0,0 (TDI=1 then 0) Move from IDLE to SHIFTDR for the bulk data load

	return 0;
}

static int finish_programming()
{

#if 0	// We do not actually need to do CHECK_STATUS. DE0-Nano initializes OK without it.

	// SIR 10 TDI (004);	... IRSCAN 0x4 CHECK_STATUS
	// RUNTEST 120 TCK;
	// SDR 732 TDI (...)	... NB this is chip-specific for EP4CE22 (see note at top, Jrunner/jb_device.h has the values)

	// openFPGALoader (log file analysis) has the following ... IRSCAN 0x4 CHECK_STATUS (see the "8104" substring)
	respond("WX2c2d2c2d2c2d2c2d2c2d2c2d2c2d2e2f2e3e3f3c3d2c3e3f3e3f3c3d3c3d2c2c81042c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");
	respond("WX8f000000000000000000000000000000Z");		// Scan 15 bytes / 120 bits - actually is runtest(60)
	
	// TODO Rework this to use the check_bit value from devices.c instead of this which seems arbitary

	respond("WX2c3e3f3c3d3c3d2c2cZ"); // TMS 1,0,0 ie RUNTEST -> SHIFTDR

	// The following reads 91 bytes / 728 bits, which is not a multiple of 3, approx 243 in jb_device.h terms.
	// However looking at openFPGALoader/src/altera.cpp ...
	// _jtag->shiftDR(tx, rx, 864, Jtag::RUN_TEST_IDLE); ... why does this not match (728 cf 864)
	// Possibly I'm reading the wrong function as that is in "void Altera::programMem(RawParser &_bit)", perhaps
	// the scan is generated by some other function? TODO chase this up (trap and generate a backtrace).
	// NB The 864 value matches 288 * 3 = 864 for several Cyclone V devices in Jrunner jb_device.c

	respond("WXff0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
			"00000000000000000000000000000dc00000000000000000000000000000000000000000000000000000000Z");

	// Am I missing a subsequent bitbang sequence? Yes 3 bits from the STARTUP string below, giving 244 which matches
	// the smaller CYCLONE III devices.

	// SIR 10 TDI (003);	... IRSCAN 0x3 STARTUP (see the "8103" substring)
	respond("WX2c6d2c6d2c6d2e6f2e3e3f3c3d2c3e3f3e3f3c3d3c3d2c2c81032c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");

#else

	// For now I'll just go straight to IRSCAN 0x3 STARTUP instead. Either of the following work for both SVF and RBF,
	// so I'll use the RBF one since I'm depreciating SVF.

	// With 0's to finish SHIFTDR (matching the final SVF byte)
	// respond("WX2c2d2c2d2c2d2c2d2c2d2c2d2c2d2e2f2e3e3f3c3d2c3e3f3e3f3c3d3c3d2c2c81032c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");

	// With 1's to finish SHIFTDR (more appropriate for RBF)
	respond("WX3c3d3c3d3c3d3c3d3c3d3c3d3c3d3e3f3e3e3f3c3d2c3e3f3e3f3c3d3c3d2c2c81032c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");

#endif

#if 1
	int startup_clocks = 120000;	// This is the value from the Quartus 10.0 SVF file, it's plenty too much
	startup_clocks = device_params[g_device_index][DEVICE_PARAMS_STARTUP];
	// printf("runtest %d\n", startup_clocks);
	if (startup_clocks > 0)			// Sanitise!
		runtest(startup_clocks);
#endif

	// NOTE Any of the following is needed to finish (unless a large number >= 3176 of startup_clocks are applied above)
	//      Also subsequently running jtagger without programming will initialise the fpga (inspired by tap_reset)

	if (1)
		respond("WX2c3e3f3e3f3c3d3c3d2c2c81ff3c3d3e3f3e3c3d2c3e3f3e3f3c3d2c2cZ");	// sets BYPASS IRSHIFT 0x3ff

	if (0)	// But this also works INSTEAD (must be >= 3176 if no runtest(startup_clocks) have been applied)
		runtest(3176);

	if (0)	// This works too
		tap_reset();

	return 0;
}

static int push_data(char *token)
{
	// Push data to FPGA
	// Expects only hex digits or ')' to finish
	// Returns 0, or offending char on error

	const char *func = __func__;	// for doabort

	// BEWARE TDI data is in reverse byte order in the svf file, so we have to buffer and reverse

	static char *pend = buf;	// BUT here in push_data() we want this.
	static char *p = buf;

	char ch = 0;
	int nibble = 0;	// HACK to swap hex chars (see reverse further down)

	while ((ch = *token++))	// assignment, and test > NULL (gcc wants double brackets here -Wparentheses)
	{
		if (ch == '(')	// First char is a bracket
			continue;

		if (ch == ')')	// End of data
			break;

		if (!isxdigit(ch))
		{
			printf("push_data bad char 0x%02x '%c'\n", ch, ch);
			return ch;		// ERROR
		}

		ch = tolower(ch);	// DEBUG so we can compare debuglog.out with -p and -x

		g_databytes++;

		pend = p;
		if (pend - buf >= sizeof(buf) - 1024)
		{
			printf("buffer overflow g_lineno %d buf %p p %p\n", g_lineno, buf, p);
			doabort(func, "buffer overflow, TODO increase HEXBUF");
		}

		// HACK to swap hex chars (see reverse further down)
		if (nibble++ & 1)
		{
			*p = *(p-1);
			*(p-1) = ch;
			*++p = 0;
		}
		else
		{
			*p++ = ch;
			*p = 0;
		}
		pend++;
	}

	return ch;
}

void send_residual(char *packbuf, char *dst)	// NOT static since called from usercode()
{
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

static int send_data(void)
{
	// Sending 63 (the max FTDI chunk) of byte-wise data at a time
	// interspersed with header bytes 0xbf packed into a single message.
	// HOWEVER the final few bits will need to be bitbanged in order to set the TMS correctly
	// (else we may overrun the shift), for now I assume these are zeros

	const char *func = __func__;	// for doabort

	char *p = buf;

	int seq = 0;

	size_t buflen = strlen(buf);
	printf("program bytes %" PRIuPTR "\n", buflen / 2);

	int TDIbits = g_clocks;		// DE0-NANO 5748760
	if (buflen != TDIbits/4)
		// doabort(func, "bad len (total hex chars)");
		printf("WARNING buflen %" PRIuPTR " != TDIbits %d /4\n", buflen, TDIbits);

	// Pack multiple FTDI writes into max size message ... see packmode above
	char packbuf[BUF_LEN];	// NB BUF_LEN is message buffer
	*packbuf = 0;			// Set as empty

	int count = 0;

	// TODO run the loop backwards, but for now just reverse the buffer (a bit inefficient)
	reverse(buf);	// NB relies on hex byte swap above as we need to pre-swap so as to undo the reverse's swap

	// dump (buf, strlen(buf));	// DEBUG dump the reversed buffer to a file for comparison with system.rbf

#if 0	// REINSTATED the last byte (See note in finish_programming(), as I now append 0xFF)

	// puts(buf+buflen-16);	// DEBUG last 16 char to see what's there (last byte's zero, so nice)
	if (strcmp(buf+buflen-2, "00"))			// Check it because finish_programming() relies on this
		doabort(func,"last byte not 00");	// BEWARE some svf's may break, forward value to bitbang

	buf[buflen-2] = 0;	// Shorten by two as last byte needs to be bitbanged in finish_programming()

#endif

	seq = (buflen * 105) / (BUF_LEN * 100);	// approximate since buffer is not filled completely (and ignores WXZ)

	// Construct 63 byte packets and fill 4096 byte buffer up to the brim

	char *dst = packbuf;
	strcpy(dst, "WX");
	dst += 2;

	int byte = 0;
	while (*p)
	{
		if (byte == 0)
		{
			// packet header
			*dst++ = 'b';
			*dst++ = 'f';
		}

		// Two hex chars (bytes)
		*dst++ = *p++;
		if (!*p)			// Check since not covered by while (*p)
			doabort(func,"buf ends");
		*dst++ = *p++;
		*dst = 0;

		if (++byte > 62)
		{
			byte = 0;

			// Always operate in packmode
			// size_t pblen = strlen(packbuf);
			size_t pblen = dst - packbuf;
			if (pblen > sizeof(packbuf) - 132)	// Tweak so it just fits
			{
				*dst++ = 'Z';
				*dst = 0;

				respond(packbuf);

				dst = packbuf;
				strcpy(dst, "WX");
				dst += 2;

				if (++count % 50 == 0)
					printf("FTDI packets (of approx %d) sent %d\n", seq, count);
			}
		}
	}

	// printf("count = %d * sizeof(packbuf) = %" PRIuPTR "\n", count, count * sizeof(packbuf));

	send_residual(packbuf, dst);

	// printf("sent %d FDTI write packets (expected approx %d last len %d)\n", count, seq, lastlen);

	return 0;
}

static int parse_rbf(FILE *f, int run)
{
	// NB the rbf is slightly smaller at 718569 bytes than the svf at 718595 bytes (TDI 5748760 bits).

	// Keep the magic incantation local to parse(), make the caller remember it separately
	unsigned int magic = 0xF00FB175;	// Foofbits (!!)

	// static char buf[HEXBUF];		// This is now a global, so it can be shared with svf loader
	static char *pend = buf + sizeof(buf) - 1024;	// -1024 for safety margin
	char *p = buf;
	static int seq;					// Retain count

	if (run == 0)
	{
		// size_t expected = 718569;	// That's specific to EP4CE22
		size_t len = fread(p, 1, sizeof(buf), f);
		if (len == sizeof(buf))
			DOABORT("buffer overflow, TODO increase HEXBUF");
		// if (len != expected)
		//	printf("WARNING read %" PRIuPTR " bytes expected %" PRIuPTR " bytes for DE0-NANO\n", len, expected);
		// else
			printf("loaded %" PRIuPTR " bytes\n", len);
		pend = p + len;

		// dump (buf, pend-buf);

		return 0;
	}
	else if (run != magic)
		DOABORT("invalid run");

	// Program ...

	begin_programming();

	seq = ((pend - buf) * 210) / (BUF_LEN * 100);	// approximate since buffer is not filled completely (and ignores WXZ)

	// BEWARE unlike SVF we're NOT reserving the last byte for bitbang (I guess I forgot), but it's FF in the RBF
	// so is subsumed into the postamble (though the bitbang is still 00). However SVF does end on 00 (actually the first
	// few chars of the "SDR 5748760 TDI (00000000FFFFF" since it's reversed). The RBF has a longer sequence of FF's
	// so it seems those 00's are superfluous. BUT I noted this above ... the rbf is slightly smaller at 718569 bytes than
	// the svf at 718595 bytes (TDI 5748760 bits) ... DONE a full comparison of SVF vs RBF (did I not do this before?)
	// CONCLUDE after reversing, the SVF has an additional 22 0xff byte preamble (54 bytes, cf 32 in the RBF).
	// Additionally the SVF has a 4 byte postamble of 0x00. That totals 26 bytes, matching the size difference.
	// Also there are a few differences in the first 42 bytes following the 0xff preamble. None of this seems to affect
	// the successful loading of the bitstream (on this particular example, TODO try a more complex bitstream).
/*
$ diff z22+rbf.hd zsvfdump.hd
3,6c3,6
< 00000030  ff ff ff ff ff ff 6a f7  f7 f7 f7 f7 f7 f3 fb f3  |......j.........|
< 00000040  f9 fb f1 f1 f9 f9 fd f9  f9 fb fb fb fd f9 ff ff  |................|
< 00000050  ff fb fd fd f9 fd fd fd  fd f9 ff fb ff fb fb 58  |...............X|
< 00000060  11 ff ff ff ff ff ff ff  ff ff ff ff ff ff ff ff  |................|
---
> 00000030  ff ff ff ff ff ff 6a f7  f7 f7 f7 f7 f7 f3 fb fb  |......j.........|
> 00000040  f9 fb f1 f1 f9 f9 f9 f9  f9 fb fb fb f9 f9 fb fb  |................|
> 00000050  fb fb f9 f9 f9 f9 f9 f9  f9 f9 fb fb fb fb fb c4  |................|
> 00000060  09 ff ff ff ff ff ff ff  ff ff ff ff ff ff ff ff  |................|
13848,13849c13848,13850
*/

	// Construct 63 byte packets and fill 4096 byte buffer up to the brim

	char packbuf[BUF_LEN];	// NB BUF_LEN is message buffer
	*packbuf = 0;			// Set as empty

	int count = 0;

	int preamble = device_params[g_device_index][DEVICE_PARAMS_PREAMBLE];
	int postamble = device_params[g_device_index][DEVICE_PARAMS_POSTAMBLE];

	if (preamble < 0) preamble = 0;		// Sanitise!
	if (postamble < 0) postamble = 0;

	// TODO bitbang any surplus bits (preamble before, postamble after), but for now just round up

	// printf("initial preamble %d postamble %d\n", preamble, postamble);
	preamble = (preamble + 7) & ~7;
	postamble = (postamble + 7) & ~7;
	// printf("rounded preamble %d postamble %d\n", preamble, postamble);
	preamble /= 8; postamble /= 8; 
	// printf("bytes   preamble %d postamble %d\n", preamble, postamble);

	int total_bytecount = 0;	// DEBUG total bytes sent (excludes header bytes, WXZ)

	char *dst = packbuf;
	strcpy(dst, "WX");
	dst += 2;

	int byte = 0;
	while (p < pend + postamble)
	{
		if (byte == 0)
		{
			// packet header
			*dst++ = 'b';
			*dst++ = 'f';
		}

		unsigned char ch;

		// BEWARE, this is bit tricksy.
		// Once in postamble, p no longer increments in the else part, so we need to increment it
		// in the first, so we test preamble to do this (and ensure it does not decrement)
		// NB p ends up at pend+postamble, well beyond the end of valid data

		if (preamble || p >= pend)
		{
			ch = 0xff;
			if (preamble)
				preamble--;
			else
				p++;
		}
		else
			ch  = *p++;

		// Two hex chars (bytes)
		*dst++ = TOHEX(ch>>4);
		*dst++ = TOHEX(ch);
		*dst = 0;	// Not necessary but makes debugging easier since can see end of data when printing buffer
		total_bytecount++;

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

				if (++count % 50 == 0)
					printf("FTDI packets (of approx %d) sent %d\n", seq, count);
			}
		}
	}

	// printf("count = %d * sizeof(packbuf) = %ld \n", count, count * sizeof(packbuf));

	send_residual(packbuf, dst);

	printf("total bytecount %d\n", total_bytecount);	// Includes preamble/postamble

	// printf("sent %d FDTI write packets (expected approx %d last len %d)\n", count, seq, lastlen);

	finish_programming();

	return 0;
}

static int parse (FILE *f, int run, int filetype)
{
	// Parse SVF file, see /home/mark/misc/FPGA/quartus/jtagger/docs/svf_specification.pdf
	// If run is set, do programming (usage is first call with run=0, reset file then with run=1)
	const char *func = "program fpga: parse";	// for doabort

	if (!f)
		return 1;

	g_filetype = filetype;	// LAZY

//	g_silent = 0;		// DEBUG turn OFF silent in client
//	respond("MX03Z");	// DEBUG turn OFF silent in server (keep spoofing - DEBUG ONLY)
//	g_debug_log = 1;	// DEBUG log writes (create our very own hex dump of FTDI write calls, useful for
						// comparing with the OpenFPGALoader dump mentioned above, after a bit of text wrangling)

	// This value likely depends on the FPGA device family/part number (see notes above on jrunner)
	unsigned int expect_clocks	= 5748760;

	// Keep the magic incantation local to parse(), make the caller remember it separately
	unsigned int magic = 0xF00FB175;	// Foofbits (!!)

	if (run == magic)
	{
		printf("programming...\n");
	}
	else if (run)
		doabort(func, "wrong magic, no programming for you");

	if (filetype == FILETYPE_RBF)
		return parse_rbf(f, run);
	else if (filetype != FILETYPE_SVF)
		doabort(func, "filetype not implemented");

	// FILETYPE_SVF

	if (run == magic)
	{
		begin_programming();
		send_data();
		finish_programming();
		return 0;
	}

	// This is going to be quite crude, just need to sucessfully read a Quartus svf file so that's
	// all I'm going to implement support for. I'm going to assume they are all very similar, so if
	// I can parse one, then I'm done. [COMMENTARY: this was the original parser, for svf. Now that I
	// support rbf files, it's not really worth bringing this up to production quality (a HUGE amount
	// of work). Perhaps I should just remove the svf option entirely?]

	int ch;
	int comment = 0;	// Set to '!' after '!' or '/' after single '/' and '!' after a pair
	int slash = 0;		// For parsing single slash
	int wanttoken = 1;
	int stage = 0;		// Parsing token sequence
	char token[1024];	// Needs to be able to hold entire line (max expected 256)
	char *ptoken = token;

	g_lineno = 1;		// NB parse() is called twice, so set global here

	while (EOF != (ch = getc(f)))
	{
		// Handle newlines and comments
		if (ch == '\n')
		{
			g_lineno++;
			comment = 0;
			slash = 0;		// May be unneccessary, but safe
			continue;
		}

		if (comment == '!')
			continue;

		if (ch == '!')
		{
			comment = '!';
			continue;
		}

		if (comment == '/')
		{
			if (ch == '/')
			{
				slash = 0;
				comment = '!';
				continue;
			}
			comment = 0;
			// Do not continue, parse it below
			slash = 1;	// TODO turn this off below after parsing (ADDENDUM, don't expect it at all!)
		}

		if (slash)
			return PROGERR_SLASH;	// Not expecting a bare slash

		// If we get here, we're parsing active content. Assemble tokens.
		if (wanttoken)	// TODO this is always true, can remove test
		{
			if (!isspace(ch))
			{
				if (ptoken - token > sizeof(token) - 4)
					return PROGERR_BOUNDS;
				*ptoken++ = ch;
				*ptoken = 0;
				continue;
			}

			if (!*token)	// Discard empty tokens (line end results in an extra one, likely due to /r/n)
			{
				ptoken = token;	// Clear token
				*token = 0;
				continue;
			}

			// Process token

			char *p = token;
			while (*p)
			{
				*p = toupper(*p);
				p++;
			}

			// printf("TOKEN \"%s\"\n", token);

			// VERY LAZY parsing, ignore the preamble and just search for the token SDR, then clocks (5748760), then TDI
			switch (stage)
			{
				case 0:
				if (!strcmp(token, "SDR"))
					stage++;
				break;

				case 1:
				if (sscanf(token, "%u", &g_clocks) != 1)
					return PROGERR_CLOCKS;
				if (g_clocks != expect_clocks)
					// return PROGERR_CLOCKS;	// Someone may want to program something else than a DE0-NANO!
					printf("WARNING clocks %u does not match DE0-NANO expect %d\n", g_clocks, expect_clocks);
				stage++;
				break;

				case 2:
				if (!strcmp(token, "TDI"))
					stage++;
				break;

				case 3:
				if (*token != '(')
					return PROGERR_TDIDATA;
				if (push_data(token))
					return PROGERR_DATA;	// Not expecting it here (error or end)
				stage++;
				break;

				case 4:
				{
					int r = push_data(token);
					if (r == ')')
					{
						printf("loaded %d bytes\n", g_databytes / 2);
						if (g_databytes != expect_clocks / 4)
							// return PROGERR_BYTES;
							printf("WARNING does not match clocks\n");
						// TODO check next char of token is ';' but for now just discard (this is automatic
						// as we just fetch the next token)
						stage++;
						break;
					}
					if (r)
						return PROGERR_DATA;	// Anything but 0 or ')'
				}
				break;

				case 5:
				if (!strcmp(token, "SIR"))
				{
					// Can just complete the programming (though ideally check the rest of the svf)
					// TODO parse the rest of the file, just to check it is as expected)
					return 0;	// Finished parse
					// stage++;	// Not reached (required if remove return)
				}
				break;

				case 6:	// WORK IN PROGRESS (not reached)
				printf("TOKEN \"%s\"\n", token);
				return 0;

				default:
				return PROGERR_STAGE;
			}

			ptoken = token;	// Clear token
			*token = 0;
		}
	}

	printf("EOF\n");	// Unexpected
	return PROGERR_FILE;
}

void printerror(int err)
{
	if (!err)
	{
		// printf("SUCCESS\n");
		return;
	}

	// To save on typing, do all error printing here (called from end of main)
	// UMMM, OK they're a bit silly. But I'm quite fond of them now. So there.
	printf("ERROR line %d ", g_lineno);
	switch (err)
	{
		case PROGERR_FILE:
		printf("PROGERR_FILE File I/O failure");
		break;
		case PROGERR_BOUNDS:
		printf("PROGERR_BOUNDS Array bounds overflow");
		break;
		case PROGERR_STAGE:
		printf("PROGERR_STAGE Stage fright");
		break;
		case PROGERR_CLOCKS:
		printf("PROGERR_CLOCKS Stop the clocks");
		break;
		case PROGERR_TDIDATA:
		printf("PROGERR_TDIDATA Missing a '(' bracket");
		break;
		case PROGERR_SLASH:
		printf("PROGERR_SLASH Slasher alert");
		break;
		case PROGERR_DATA:
		printf("PROGERR_DATA Bad data, bad");
		break;
		case PROGERR_BYTES:
		printf("PROGERR_BYTE Bad byte count, Dracul?");
		break;
		default:
		printf("Unknown error %d", err);
	}
	printf("\n");
}

int program_fpga(char *fname, int filetype, int device_index, int yes)
{
	printf("PROGRAM FPGA ... reading %s\n", fname);
	FILE *f = fopen(fname, "rb");
	if (!f)
	{
		printf("program fpga: ERROR opening file %s\n", fname);
		return 1;
	}

	g_device_index = device_index;	// Lazy
	
	int ret = parse(f, 0, filetype);
	if (!ret)
	{
		char line[256];

		if (g_spoofprog || yes)
			*line = 'Y';	// DEBUG skip prompt
		else
		{
			printf("parse OK, enter y to program\n");
			if (!fgets(line, sizeof(line), stdin))
				*line = 0;	// Just to be sure
		}

		if (*line)
		{
			if (toupper(line[0]) == 'Y')
			{
				time_t tstart, tfinish;
				time(&tstart);
				ret = parse(f, 0xF00FB175, filetype);	// Recite the Magic Spell "FoofBits" (just so I don't call it by mistake)
				time(&tfinish);

#ifndef __MINGW32__
				// This is a weird one as PRId64 expands to %ld not %lld on 64 bit linux gcc
				printf("programming complete, duration %lld seconds\n", (long long)(tfinish - tstart));
#else
				printf("programming complete, duration %" PRId64 " seconds\n", (long long)(tfinish - tstart));
#endif
			}
			else
				printf("programming declined, exit\n");
		}
		else	// Ctrl-D will get here
		{
			printf("no input, programming declined, exit\n");
		}
	}

	fclose(f);

	printerror(ret);

	return ret;
}


