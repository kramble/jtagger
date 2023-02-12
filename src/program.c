/* program.c - read Serial Vector Format (.svf) file and (attempt to) program FPGA

For spec see /home/mark/misc/FPGA/quartus/jtagger/docs/svf_specification.pdf
For hints see /home/mark/misc/FPGA/quartus/openFPGALoader-master

To generate a SVF from Quartus SOF file...
$ wine /media/mark/Vista/altera/10.1/quartus/bin/quartus_cpf.exe -c -q 12.0MHz -g 3.3 -n p system.sof system.svf
NB unlike .sof a .svf is a plain text file (though .sof does have a text header)

Raw Binary Files (.rbf) are created in the same way (see openFPGALoader/doc/vendors/intel). JTAG does not
support compression, use -r switch on openFPGALoader for rbf input) ...
quartus_cpf.exe -c --option=bitstream_compression=off system.sof system.rbf

From the spec...
The SVF file is defined as an ASCII file that consists of a set of SVF statements.
The maximum number of characters allowed on a line is 256, although one SVF
statement can span more than one line. Each statement consists of a command
and associated parameters. Each SVF statement is terminated by a semicolon.
SVF is not case sensitive. Comments can be inserted into a SVF file after an
exclamation point ‘!’ or a pair of slashes ‘//’. Either ‘//’ or ‘!’ will comment out the
remainder of the line.

For lazy sake, I'm going to assume the following sequence ...
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

#define PROGERR_FILE 101	// File I/O
#define PROGERR_BOUNDS 102	// Array bounds overflow
#define PROGERR_STAGE 103
#define PROGERR_CLOCKS 104
#define PROGERR_TDIDATA 105
#define PROGERR_SLASH 106
#define PROGERR_DATA 107
#define PROGERR_BYTES 108

// This is the large static buffer shared by both filetype versions of the programmer
//#define HEXBUF (2 * 1024 * 1024)	// Sufficient for De0-Nano Cyclone EP4CE22 FPGA
#define HEXBUF (16 * 1024 * 1024)	// It's bss so no harm making this a lot bigger (in case someone has a HUGE fpga)
static char buf[HEXBUF];			// Retain buffer from run==0 for run==magic call

int g_lineno;	// Used in printerror() so global
int g_databytes;
unsigned int g_clocks;	// TDI from svf

#define TOHEX(n) (((n) & 15) > 9 ? (((n)&15) -10 + 'a') : (((n)&15) + '0'))		// Convert nibble to hex char

// reverse string in place 
static void reverse(char *s)
{
	char *j;
	int c;
	j = s + strlen(s) - 1;
	while (s < j)
	{
		c = *s;
		*s++ = *j;
		*j-- = c;
	}
}

static int begin_programming()
{
	// Perform initial steps

	// TODO fix this mess (it's basically just cribbed from a dumped FTDI parameters log of an
	// openFPGALoader session). Do it programatically instead.

	// Enter STATE IDLE

	// tap_reset();	// io_check() hangs when g_spoofprog so just use respond
	// respond("WX2e2f2e2f2e2f2e2f2e2f2e2eZ");

	// runtest5();
	// respond("WX2c2d2c2d2c2d2c2d2c2d2cZ");

	// openFPGALoader (log file analysis) does ...
	respond("WX3e3f3e3f3e3f3e3f3e3f3e3f2e3c3d3e3f3c3d3c3d2c2cc4ffffffff2cc4ffffffff2cc4ffffffff2cc4ffffffff2cc3ffffff"
			"3c7d3c7d3c7d3c7d3c7d3c7d3c7d3e7f3e3e3f3e3f3e3f3e3f3e3f3e3f2e3e3f3e3f3e3f3e3f3e3f3e3f2e3c3d2c3e3f3e3f"
			"3c3d3c3d2c2c81022c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");

	// SIR 10 TDI (002);	... IRSCAN 0x2

	// respond("WX3e3f3e3f3c3d3c3d2c2c81022c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");	// IRSHIFT 0x02

	// RUNTEST IDLE 24000 TCK ENDSTATE IDLE;
	runtest(24000);

	// openFPGALoader (log file analysis) does ...
	respond("WX2c3e3f3c3d3c3d2c2cZ");

	return 0;
}

static int finish_programming()
{
	// SIR 10 TDI (004);
	// RUNTEST 120 TCK;
	// SDR 732 TDI (0000 ....

	// openFPGALoader...
	respond("WX2c2d2c2d2c2d2c2d2c2d2c2d2c2d2e2f2e3e3f3c3d2c3e3f3e3f3c3d3c3d2c2c81042c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");
	respond("WX8f000000000000000000000000000000Z");
	respond("WX2c3e3f3c3d3c3d2c2cZ");

	respond("WXff0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
			"00000000000000000000000000000dc00000000000000000000000000000000000000000000000000000000Z");

	respond("WX2c6d2c6d2c6d2e6f2e3e3f3c3d2c3e3f3e3f3c3d3c3d2c2c81032c2d2e2f2e3c3d2c3e3f3e3f3c3d2c2cZ");

#if 0
	// SIR 10 TDI (003);
	respond("WX2e2f2e2f2c2d2c2d2c81032c2d2e2f2e2e2c2d2cZ");	// IRSHIFT 0x03
	// need to get to IDLE, so whatever is needed, maybe...
	respond("WX2e2e2f2c2d2cZ");	// from DRSHIFT to RUNIDLE (should work for IRSHIFT)
#endif

	// TEST early return (experiment...)

	// runtest(1200);		// NO, NOT programmed
	// runtest(6000);		// YES, programmed
	// return 0;	

	// So we could reduce this, but it's so ridiculously fast anyway that I won't risk it

	runtest(120000);	// This is the svf value

	return 0;

	respond("WX2c2cZ");

	runtest(512);

	// openFPGALoader
	respond("WX2c3e3f3e3f3c3d3c3d2c2c81ff3c3d3e3f3e3c3d2c3e3f3e3f3c3d2c2cZ");

//	respond("WX2e2f2e2f2c2d2c2d2c81ff3c3d3e3f3e3e3c3d2cZ");	// IRSHIFT 0x3ff (unsure about top bits 3c3d etc)
//	respond("WX2e2e2f2c2d2cZ");	// from DRSHIFT to RUNIDLE (should work for IRSHIFT)

	runtest(24000);
	respond("WX2cZ");	// openFPGALoader has this, but I guess it's superfluous

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

	char ch;
	if (1)
	{
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

		// if (ch != ')')	// End of data
		return ch;
	}

	return 0;
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
	printf("program bytes %ld\n", buflen / 2);

	int TDIbits = g_clocks;		// DE0-NANO 5748760
	if (buflen != TDIbits/4)
		// doabort(func, "bad len (total hex chars)");
		printf("WARNING buflen %ld != TDIbits %d /4\n", buflen, TDIbits);

	// Pack multiple FTDI writes into max size message ... see packmode above
	char packbuf[BUF_LEN];	// NB BUF_LEN is message buffer
	*packbuf = 0;			// Set as empty

	int count = 0;
	// int lastlen = 0;	// DEBUG

	// TODO run the loop backwards, but for now just reverse the buffer (a bit inefficient)
	reverse(buf);	// NB relies on hex byte swap above as we need to pre-swap so as to undo the reverse's swap
	// printf("buflen %ld expect %d g_databytes %d\n", buflen, TDIbits/4, g_databytes);

	// puts(buf+buflen-16);	// DEBUG last 16 char to see what's there (last byte's zero, so nice)
	if (strcmp(buf+buflen-2, "00"))			// Check it because finish_programming() relies on this
		doabort(func,"last byte not 00");	// BEWARE some svf's may break, forward value to bitbang

	buf[buflen-2] = 0;	// Shorten by two as last byte needs to be bitbanged in finish_programming()

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

	// printf("count = %d * sizeof(packbuf) = %ld \n", count, count * sizeof(packbuf));

	if (dst > packbuf+2)
	{
		int numpackets = (dst - packbuf - 2) / 64;
		char* last = packbuf + numpackets * 64 + 2;
		// lastlen = dst - last;	// DEBUG
		// printf("numpackets = %d packbuf = %p last = %p lastlen = %d\n", 
		//	numpackets, packbuf, last, lastlen);
		// printf("packlen = %ld lastoffset = %ld\n", dst - packbuf, last - packbuf);

		if (last != dst)	// TODO CHECK and TEST (simulate case) - works for DE0-NANO but general case needs checking
		{
			// update header for last packet
			unsigned char header = 0x80 | ((dst - last) / 2 - 1);	// WHY -1 ?
			if (header > 0xff)
				doabort(func,"header");
			char tmp[3];	// TODO use TOHEX()
			sprintf(tmp, "%02x", header);
			// printf("tmp [%s]\n", tmp);
			*last = tmp[0];
			*(last+1) = tmp[1];
		}

		// Write residual
		*dst++ = 'Z';
		*dst = 0;

		respond(packbuf);
	}

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
		size_t expected = 718569;
		size_t len = fread(p, 1, sizeof(buf), f);
		if (len == sizeof(buf))
			doabort(__func__, "buffer overflow, TODO increase HEXBUF");
		if (len != expected)
			printf("WARNING read %ld bytes expected %ld bytes for DE0-NANO\n", len, expected);
		else
			printf("loaded %ld bytes\n", len);
		pend = p + len;
		return 0;
	}
	else if (run != magic)
		doabort(__func__, "invalid run");

	// Program ... TODO merge with send_data() which is very similar

	begin_programming();

	seq = ((pend - buf) * 210) / (BUF_LEN * 100);	// approximate since buffer is not filled completely (and ignores WXZ)

	// Construct 63 byte packets and fill 4096 byte buffer up to the brim

	char packbuf[BUF_LEN];	// NB BUF_LEN is message buffer
	*packbuf = 0;			// Set as empty

	int count = 0;
	// int lastlen = 0;	// DEBUG

	char *dst = packbuf;
	strcpy(dst, "WX");
	dst += 2;

	int byte = 0;
	while (p < pend)
	{
		if (byte == 0)
		{
			// packet header
			*dst++ = 'b';
			*dst++ = 'f';
		}

		unsigned char ch = *p++;

		// Two hex chars (bytes)
#if 0		
		char tmp[3];
		sprintf(tmp, "%02x", ch);	// Inefficient, TODO properly, DONE see #else
		*dst++ = tmp[0];
		*dst++ = tmp[1];
		*dst = 0;
#else
		*dst++ = TOHEX(ch>>4);
		*dst++ = TOHEX(ch);
		// printf("%02x, %02x, '%c', %02x, '%c'\n", ch, dst[-2], dst[-2], dst[-1], dst[-1]);
		// doabort(__func__,"test TOHEX");
#endif

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

				if (++count % 50 == 0)
					printf("FTDI packets (of approx %d) sent %d\n", seq, count);
			}
		}
	}

	// printf("count = %d * sizeof(packbuf) = %ld \n", count, count * sizeof(packbuf));

	if (dst > packbuf+2)
	{
		int numpackets = (dst - packbuf - 2) / 64;
		char* last = packbuf + numpackets * 64 + 2;
		// lastlen = dst - last;	// DEBUG
		// printf("numpackets = %d packbuf = %p last = %p lastlen = %d\n", 
		//	numpackets, packbuf, last, lastlen);
		// printf("packlen = %ld lastoffset = %ld\n", dst - packbuf, last - packbuf);

		if (last != dst)	// TODO CHECK and TEST (simulate case) - works for DE0-NANO but general case needs checking
		{
			// update header for last packet
			unsigned char header = 0x80 | ((dst - last) / 2 - 1);	// WHY -1 ?
			if (header > 0xff)
				doabort(__func__,"header");
			char tmp[3];	// TODO use TOHEX()
			sprintf(tmp, "%02x", header);
			// printf("tmp [%s]\n", tmp);
			*last = tmp[0];
			*(last+1) = tmp[1];
		}

		// Write residual
		*dst++ = 'Z';
		*dst = 0;

		// puts(packbuf);
		respond(packbuf);
	}

	// printf("sent %d FDTI write packets (expected approx %d last len %d)\n", count, seq, lastlen);

	finish_programming();

	return 0;
}
		
static int parse_openFPGALoader_analysis(FILE *f, int run)
{
	const char *func = __func__;	// for doabort

	// Keep the magic incantation local to parse(), make the caller remember it separately
	unsigned int magic = 0xF00FB175;	// Foofbits (!!)

	// static char buf[HEXBUF];		// This is now a global, so it can be shared with svf loader
	static char *pend = buf + sizeof(buf) - 1024;	// -1024 for safety margin
	char *dst = buf;
	char *p = buf;
	static int seq;					// Retain count

	if (run == 0)
	{
		// Read text file format: sequenceno "len" length hexstring
		for (int i=0 ; /*none*/ ; i++ )
		{
			int remain = pend - p;
			if (remain < 1024)
			{
				printf("buffer overflow i %d buf %p p %p\n", i, buf, p);
				doabort(func, "buffer overflow, TODO increase HEXBUF");
			}
			if (!fgets(p, remain, f))
				break;	// Done

			// Skip the first 5 lines (these are info) - BEWARE if change analyse.c
			if (i < 5)
				continue;

			int len;
			if (sscanf(p, "%d len %d", &seq, &len) != 2)
			{
				printf("scan error line %d\n", i);
				doabort(func, "scan");
			}

			if (seq != i-5 || len < 1 || len > remain)	// len > remain is just an approximate check
			{
				printf("bad scan input line %d seq %d len %d\n", i, seq, len);
				doabort(func, "scan");
			}

			// Move the hex string
			char *dst0 = dst;	// DEBUG
			p += 15;			// Skip to first hex (assumes fixed length fields, see analyze.c)
			int count = 0;
			unsigned char c;
			while ((c = *p++))
			{
				if (isspace(c))
					continue;
				if (!isxdigit(c))
				{
					printf("line %d char %d expected hex got %02x (%c)\n", i, count, c, c);
					doabort(func, "scan");			
				}
				*dst++ = c;
				count++;		// Only counts hex chars moved
			}

			*dst++ = 0;			// Add string terminator for each hex segment
			*dst = 0;			// Add second terminator for entire buffer (to terminate magic pass below)
			
			p = dst;			// Append next fget() after dst instead of after last fget() input (saves space)

			if (count != len*2)
			{
				printf("expected count %d got %d [%s]\n", len, count, dst0);
				doabort(func, "scan");			
			}
		}

		pend = p;
		printf("loaded sequences %d bytes %ld\n", seq, (pend-buf)/2);
		// printf("loaded seq %d buf %p pend %p total %ld\n", seq, buf, pend, pend-buf);
		if (seq != 11793)
			doabort(func, "bad seq count");

		return 0;
	}
	else if (run != magic)
		doabort(func, "invalid run");

	// Program ...

	int packmode = 1;	// CONFIGURE .. this WORKS so use it always

	size_t len = pend - buf;
	if (len != 1503556 + seq + 1)	// Allow for all the nulls added as string terminators
		doabort(func, "bad len (total hex chars)");

	// Pack multiple FTDI writes into max size message ... see packmode above
	char packbuf[BUF_LEN];	// NB BUF_LEN is message buffer
	*packbuf = 0;			// Set as empty

	char s[BUF_LEN];
	int count = 0;
	// int lastlen = 0;	// DEBUG

	while (*p)
	{
		// printf("buf [%s] *p %02x *(p+1) %02x\n", s, *p, *(p+1));
		int slen = strlen(p);
		// lastlen = slen;	// DEBUG
		if(slen >= sizeof(s) - 8)	// Allow for WX(*)Z plus a bit of margin
			doabort(func, "respond buffer overflow");

		char *dst = s;
		strcpy(dst, "WX");
		dst += 2;
		strcpy(dst, p);
		dst += slen;
		*dst++ = 'Z';
		*dst = 0;
		p += strlen(p) + 1;
		count++;

		if (packmode)
		{
			size_t pblen = strlen(packbuf);
			if (pblen + strlen(s) > sizeof(packbuf) - 4)
			{
				respond(packbuf);
				strcpy(packbuf,s);	// Clear packbuf and copy s
			}
			else
			{
				if (!pblen)
				{
					static int n;	// check it only runs once
					if (++n > 1)
						doabort(func,"pblen");
					strcpy(packbuf, s);		// First time only
				}
				else
					strcpy(packbuf+pblen-1, s+2);	// Overwrite the Z, omitting WX in s
			}
		}
		else
			respond(s);

		if (count % 1000 == 0)
			printf("FTDI packets (of %d) sent %d\n", seq, count);
	}

	if (*packbuf)			// Flush buffer (won't happen if !packbuf so no need to test)
		respond(packbuf);

	// printf("sent %d FDTI write packets (expected %d last len %d)\n", count, seq, lastlen);
	if (count != seq + 1)
		doabort(func, "count did not match seq");

	return 0;
}

static int parse (FILE *f, int run, int filetype)
{
	// Parse SVF file, see /home/mark/misc/FPGA/quartus/jtagger/docs/svf_specification.pdf
	// If run is set, do programming (usage is first call with run=0, reset file then with run=1)
	const char *func = "program fpga: parse";	// for doabort

	if (filetype < 1 || filetype > 3)
		doabort(func, "program fpga: unsupported filetype");

	if (!f)
		return 1;

//	g_silent = 0;		// DEBUG turn OFF silent in client
//	respond("MX03Z");	// DEBUG turn OFF silent in server (keep spoofing - DEBUG ONLY)
//	g_debug_log = 1;	// DEBUG log writes

	// This value likely depends on the FPGA device family/part number
	unsigned int expect_clocks	= 5748760;

	// Keep the magic incantation local to parse(), make the caller remember it separately
	unsigned int magic = 0xF00FB175;	// Foofbits (!!)

	if (run == magic)
	{
		printf("programming...\n");
	}
	else if (run)
		doabort(func, "wrong magic, no programming for you");

	if (filetype == 2)
		return parse_openFPGALoader_analysis(f, run);
	else if (filetype == 3)
		return parse_rbf(f, run);
	else if (filetype != 1)
		doabort(func, "filetype not implemented");

	// filetype 1 ... svf

	if (run == magic)
	{
		begin_programming();
		send_data();
		finish_programming();
		return 0;
	}

	// This is going to be quite crude, just need to sucessfully read a Quartus svf file so that's
	// all I'm going to implement support for. I'm going to assume they are all very similar, so if
	// I can parse one, then I'm done.

	int ch;
	int comment = 0;	// Set to '!' after '!' or '/' after single '/' and '!' after a pair
	int slash = 0;		// For parsing single slash
	int wanttoken = 1;
	int stage = 0;		// Parsing token sequence
	// unsigned int clocks = 0;	// Expect 5748760 now a global g_clocks
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

int program_fpga(char *fname, int filetype)
{
	printf("PROGRAM FPGA ... reading %s\n", fname);
	FILE *f = fopen(fname, "rb");
	if (!f)
	{
		printf("program fpga: ERROR opening file %s\n", fname);
		return 1;
	}

	int ret = parse(f, 0, filetype);
	if (!ret)
	{
		char line[256];

		if (g_spoofprog)
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

				printf("programming complete, duration %ld seconds\n", tfinish - tstart);
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


