/* usercode.c - custom interface to fpga. This version for fpga/audio.

*/

#include "../../src/common.h"

#ifndef __MINGW32__
#include <termios.h>
#include <sys/select.h>
#endif

#define UNUSED __attribute__((__unused__))	// Just a handy shortcut

#define NEOEXEFILE "neorv32_exe.bin"	// Used as default

#define CHIPID_v4 0x97d2f9d1		// fpga/riscv

// Instruction Register opcodes, see fpga/riscv-basic/rtl/defines.v
#define IIDENT	0
#define IRADDR	1
#define IWADDR	2
#define IRDATA	5
#define IWDATA	6
#define IUART	12
#define IAUDIO	13
#define IFLAGS	14
#define IBYPASS	15	(actually anything except the above acts as a bypass opcode)

// These are currently unused (legacy from fpga/txrxmem)
#define FLAGS_DEBUG  0x100
#define FLAGS_RADDR  0x200
#define FLAGS_WADDR  0x300
#define FLAGS_RDATA  0x400
#define FLAGS_WDATA  0x500

// #define FLAGS_TMODE 0x00010000	// Unused (legacy from fpga/txrxmem)
// #define FLAGS_TWRAP 0x00020000	// Unused (legacy from fpga/txrxmem)
// #define FLAGS_APLAY 0x00040000	// Unused (legacy from fpga/audio)

#define FLAGS_RUN	0x00080000	// RESET is active low, so take high to start cpu
#define FLAGS_URX	0x00100000	// Enable uart rx logging
#define FLAGS_UTX   0x00200000	// Uart tx strobe
#define FLAGS_UTXM  0x00400000	// Enable tx multi-byte mode (4 bytes sent per strobe)
#define FLAGS_UTXB  0x00800000	// Enable tx block mode

#define RESET_OFFSET 1			// see print_rx_buffer()

#define MEMSIZE 2048			// FPGA block RAM 32 bit words (jtagger riscv jtag i/o buffer)

#if MEMSIZE & (MEMSIZE-1)
#error "MEMSIZE must be a power of 2"	// This is relied on for address validation
#endif

#define UBUF_LEN (MEMSIZE * 4)			// Convert to byte size for bulk_read()

#define UPLOAD_UALREADY 0x10000			// Arbitary flag value (must be single bit, not in low 8 bits since or'd with mode char)

unsigned int ubuf[MEMSIZE];				// 8kB buffer

static volatile int g_quit;	// Set by signal handler to force exit

static int get_hub_info(void)
{
	// Exercise the Virtual Jtag, note the USER0 = 0xE and USER1 = 0xC values (use ONLY these)
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

	printf("hubinfo = 0x%08x\n", hubinfo);

	unsigned int expected_hubinfo = 0x08086e04;
	if (hubinfo != expected_hubinfo)
	{
		// Value depends on the FPGA virtual jtag configuration vis VIR size, address size (number of instances)
		printf("\n==================================================\n");
		printf("WARNING hubinfo does not match expected %08x\n", expected_hubinfo);
		printf("Check and update expected value if this is correct\n");
		printf("==================================================\n");
	}

	printf("nodeinfo = 0x%08x\n", nodeinfo);

	// Decode the hub and node info

	printf("hub info m = %d mfg = 0x%x n = %d version = %d\n",
				hubinfo & 0xff, (hubinfo>>8) & 0x1ff, (hubinfo>>19) & 0xff, (hubinfo>>27) & 0x1f);

	printf("node info inst = %d mfg = 0x%x id = %d version = %d\n",
				nodeinfo & 0xff, (nodeinfo>>8) & 0x1ff, (nodeinfo>>19) & 0xff, (nodeinfo>>27) & 0x1f);

	if (hubinfo != expected_hubinfo)
		return ERROR_FAIL;

	return 0;

}	// end get_hub_info()

static void print_rx_buffer(int reset_offset)
{
	unsigned long long vdr_ret;

	static int offset;

	if (reset_offset)
		offset = 0;

	vdr_ret = scan_vir_vdr(4, 32, IUART, 0, READMODE);

	int rxpos = (vdr_ret >> 16) & (UBUF_LEN-1);	// mask with size

	int bytesneeded = rxpos - offset;

	if (bytesneeded == 0)
		return;

	// Round the starting offset to next lowest int
	int rxaddr = offset / 4;
	int skip = offset - rxaddr * 4;		// up to 3 extra chars are read, so setup to skip them later

	if (bytesneeded < 0)
	{
		printf("negative bytesneeded %d setting %d offset %d\n", bytesneeded, bytesneeded + MEMSIZE, offset);
		bytesneeded += MEMSIZE;
	}

	if (skip)
		bytesneeded += skip;

	offset = rxpos;

	int count = 72 * 4;			// BEWARE do not exceed 72 * 4 (breaks bulk_read)

	while (bytesneeded > 0)
	{
		if (bytesneeded < count)
			count = bytesneeded;

		unsigned int ch;

		// Set read address each time as automatic increment will have left it one too high
		scan_vir_vdr(4, 32, IRADDR, rxaddr, NOREADMODE);

		int readbytes = (count + 3) & ~3;	// Round up to 4 else bulk_read barfs		
		bulk_read((char*)ubuf, readbytes, IRDATA);

		int *p = (int*)ubuf;

		int loops = count / 4;

		if (loops == 0)
			loops = 1;

		for (int i=0; i<loops; i++)
		{
			// NB we always start on a int boundary since IRADDR is int-based
			ch = *p++;

			// Adjust the final chars since they are shifted to MSB of int word
			int charsneeded = 4;
			if (bytesneeded < 4)
			{
				// Unfortunately the rx may have updated the buffer between reading IUART and
				// the bulk read (which results in random corruption), so check it again
				vdr_ret = scan_vir_vdr(4, 32, IUART, 0, READMODE);
				int newrxpos = (vdr_ret >> 16) & (UBUF_LEN-1);
				if (rxpos != newrxpos)
				{
					// push back the residual, we will process them next time
					offset -= bytesneeded;
					fflush(stdout);
					return;
				}

				charsneeded = bytesneeded;
				ch >>= 8 * (4 - bytesneeded);	// Adjust final chars
			}

			for (int j=0; j<charsneeded; j++)
			{
				char cout = ch & 0xff;
				ch >>= 8;

				if (!cout)
					cout = '*';

				if (skip)
					skip--;
				else
					fputc(cout, stdout);

				bytesneeded--;
			}
		}

		rxaddr += count / 4;

	}	// end while

	fflush(stdout);

}	// end print_rx_buffer()

static void wait_uart(int timeout, int delay)
{
	int ret_busy = 1;
	while (ret_busy)
	{
		if (!timeout--)
		{
			printf("ERROR uart tx stuck busy\n");
			exit(1);
		}
		unsigned long long vdr_ret = scan_vir_vdr(4, 32, IUART, 0, READMODE);
		int ret_busy = vdr_ret & 0x20000000 ? 1 : 0;	// Wait for tx_busy to clear
		if (!ret_busy)
			break;
		JTAGGER_SLEEP(delay * MILLISECONDS);
	}
	// printf("wait_uart ended at timout = %d\n", timeout);
}

static void send_char(char txchar, int delay)
{
	wait_uart(50, delay);

	scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN, NOREADMODE);
	scan_vir_vdr(4, 32, IUART, txchar, READMODE);
	scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN | FLAGS_UTX, NOREADMODE);
	scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN, NOREADMODE);	// Could omit this for speed, but safer not to
	// NB not flushed for speed, so take care when followed by JTAGGER_SLEEP
}

static int upload(char *uparams, int mode, int delay)
{
	int blockmode = 1;	// CONFIGURATION (else uses FLAGS_UTXM mode)

	// printf ("upload mode = %08x\n", mode);

	if ((mode & 0xff) == 'U')	// CAPTIAL (masking off UPLOAD_UALREADY flag)
	{
		printf("WARNING slow mode selected 'U', for fast mode use lowercase 'u'\n");
		blockmode = 0;
	}

	// Check for file parameter
	char *fname = uparams;
	if (fname && *fname)	// BEWARE uparams could be NULL since we can be called from terminal
	{
		fname++; // skip 'u'
		// Trim leading spaces (TODO trailing too), though neither are actually needed as main()
		// trims both before calling usercode (unless filename is quoted and contains spaces)
		while (*fname && isspace(*fname))	// isspace also checks for tabs
			fname++;
	}
	if (!fname || !*fname)
		fname = NEOEXEFILE;

	printf("Uploading \"%s\"\n", fname);

	FILE *ifile = fopen(fname, "rb");
	if (!ifile)
	{
		printf("ERROR could not open %s\n", fname);
		return ERROR_FAIL;
	}

	time_t tstart, tfinish;
	time(&tstart);

	if (!(mode & UPLOAD_UALREADY))
		send_char('u', delay);

	jflush();	// ALWAYS flush before JTAGGER_SLEEP
	JTAGGER_SLEEP(50 * MILLISECONDS);	// Allow longer for bootloader to print prompt via uart

	print_rx_buffer(0);
	printf("\n");

	unsigned int chin;
	int count = 0;
	int bytes_read;
	unsigned long long vdr_ret;

	int busy_count = 0;	// DEBUG stuff
	int max_nloop = 0;

	// Ensure sane flags (clear all TX variants so we get an initial positive edge)
	if (blockmode)
		scan_vir_vdr(4, 32, IFLAGS, FLAGS_RUN, NOREADMODE);	// Disable rx to avoid buffer corruption
	else
		scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN, NOREADMODE);	// May as well enable rx for feedback

	wait_uart(50, delay);

	// NB UBUF_LEN - 4 because fpga tx does not send final address (TODO could fix verlog, but this is simpler)
	while ((bytes_read = fread(ubuf, 1, UBUF_LEN - 4, ifile)) > 0)
	{
		printf("bytes read = %d\n", bytes_read);
		int bufaddr = 0;	// For residual (index into int array, not char array)

		if (bytes_read < 8)
			blockmode = 0;	// Send final data the slow way

		if (blockmode)
		{
			int send_bytes = bytes_read & ~3;	// Round DOWN
			scan_vir_vdr(4, 32, IWADDR, 0 , NOREADMODE);	// Set write address
			bulk_write((char*)ubuf, send_bytes, IWDATA);

			// NB fpga tx does not send final address, so use bytes_read/4 as end, hence cannot use that
			// location for data, see fread() above
			// printf("set end_addr %d\n", send_bytes/4 );
			vdr_ret = scan_vir_vdr(4, 32, IUART, send_bytes/4, NOREADMODE);	// Set end_addr to uart

			scan_vir_vdr(4, 32, IFLAGS, FLAGS_RUN | FLAGS_UTXB, NOREADMODE);	// Initiate

			vdr_ret = scan_vir_vdr(4, 32, IUART, send_bytes/4, READMODE);	// Get uart status

			// printf("uart status after bulk upload %08x\n", (unsigned int) vdr_ret);
			if ((vdr_ret & 0x60000000) == 0x60000000)	// Want tx_block_run && tx_busy
			{
				printf("Bulk uploaded %d bytes, uart now processing (please wait a few seconds)...\n", send_bytes);
				count += send_bytes;
				wait_uart(2000, delay);	// Approx 10 seconds
				vdr_ret = scan_vir_vdr(4, 32, IUART, 0, READMODE);	// Get uart status
				// printf("uart status after wait %08x\n", (unsigned int) vdr_ret);
				bytes_read -= send_bytes;
				bufaddr = send_bytes / 4;		// For residual
			}
			else
			{
				printf("Bulk upload not supported with this bitstream (use a newer one)\n");
				printf("Fallback to slow mode\n");
				blockmode = 0;
				// Fall through to !blockmode
			}			
			scan_vir_vdr(4, 32, IFLAGS, FLAGS_RUN, NOREADMODE);	// Remove FLAGS_UTXB (so we get positive edge for each loop)
		}
		
		if (bytes_read)		// TODO can omit since while (bytes_read > 0) takes care of this
		{
			// printf("Residual: bytes_read %d bufaddr %d\n", bytes_read, bufaddr);	// DEBUG - don't omit "if (bytes_read)" yet!

			// NB uses send_char() so FLAGS_URX is re-enabled (ie OK, we can use it in slow mode for feedback)
			while (bytes_read > 0)
			{
				int bytes_send = bytes_read > 3 ? 4 : bytes_read;
				bytes_read -= bytes_send; 
				if (bytes_send == 4)
				{
					int ret_busy = 1;
					int nloop = 0;
					chin = ubuf[bufaddr++];		// NB int array, not char
					while (ret_busy)
					{
						nloop++;
						if (nloop > max_nloop)
							max_nloop = nloop;
						vdr_ret = scan_vir_vdr(4, 32, IUART, chin, READMODE);	// Set chin to uart and get uart status
						int ret_busy = vdr_ret & 0x20000000 ? 1 : 0;
						if (!ret_busy)
							break;
						busy_count++;
						JTAGGER_SLEEP(delay * MILLISECONDS);
					}

					vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN | FLAGS_UTXM, NOREADMODE);
					vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN, NOREADMODE);
				}
				else
				{
					// Not usually done since exe file is a multiple of 4 bytes, but can be forced by sending
					// eg a plain text file (monitor the result using zdebug.bin neorv32 client executable)
					printf("Sending residual %d\n", bytes_send);
					chin = ubuf[bufaddr++];		// NB int array, not char
					for (int i=0; i<bytes_send; i++)
					{
						send_char(chin & 0xff, delay);	// NB no jflush() for speed
						chin >>= 8;
					}
				}
				count += bytes_send;
				if (count % 100 == 0)	// BEWARE ensure mod is divisible by 4 as count increments by 4
					printf("%d bytes sent\n", count);
			}
		}

	}	// end while

	// Clear the buffer else we print binary garbage in print_rx_buffer()

	memset(ubuf, 0, UBUF_LEN);
		bulk_write((char*)ubuf, UBUF_LEN, IWDATA);

	scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN, NOREADMODE);	// Enable rx

	send_char('e', delay);	// Execute
	jflush();	// ALWAYS flush before JTAGGER_SLEEP

	JTAGGER_SLEEP(50 * MILLISECONDS);	// Allow for bootloader to print prompt via uart
	print_rx_buffer(RESET_OFFSET);		// Reset the buffer offset since we cleared the rx_addr via !FLAGS_URX

	time(&tfinish);
	if (blockmode)
	{
		printf("\nTotal bytes %d ", count);
		printf("Elapsed time %"  PRId64 " seconds\n", (int64_t)(tfinish - tstart));
	}
	else
	{
		printf("\nTotal bytes %d busy count %d max %d ", count, busy_count, max_nloop);
		printf("Elapsed time %"  PRId64 " seconds\n", (int64_t)(tfinish - tstart));
	}

	return 0;
}

#ifndef __MINGW32__
static int stdin_avail()
{
	// https://stackoverflow.com/questions/26948723/checking-the-stdin-buffer-if-its-empty
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(STDIN_FILENO, &readfds);

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	return select(1, &readfds, NULL, NULL, &timeout);
}

static int terminal_noncanonical(int delay)
{
	int mode = -1;	// Returned
	char esc = '~';

#ifndef JTAGGER_NOHELP	// Set in Makefile to omit this, CFLAGS append -DJTAGGER_NOHELP
	printf("\n\nInteractive mode, escape char is '%c' press '%c' ENTER to QUIT\n", esc, esc);
	printf("For bootloader upload press 'u' then at \"Awaiting neorv32_exe.bin\" press '%cu'\n", esc);
#endif

	// Originally meant for interaction with the bootloader so only sent first char, see
	// terminal_canonical(), but now extended for use as a general terminal interface.
	// This (linux, noncanonical) version reads characters from input without waiting for
	// ENTER, so is more intuitive than the old terminal_canonical() version.
	// However this requires an escape character to handle special cases, as follows
	// ~ ~		sends a single ~
	// ~ ENTER	exits
	// ~ u		uploads the default file (NEOEXEFILE "neorv32_exe.bin")
	//			... NB press 'u' first to tell the bootloader to expect a file

	int noncanonical = 0;

	struct termios old_termios, new_termios;

	tcgetattr(STDIN_FILENO, &old_termios);

	new_termios = old_termios;
	new_termios.c_lflag &= ~ICANON & ~ECHO;			// No ECHO since bootloader does echo itelf
	tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
	noncanonical = 1;

	char txchar;
	int gotesc = 0;
	int sleeps = 0;	// DEBUG

	for (;;)
	{
		if (stdin_avail() < 1)
		{
			print_rx_buffer(0);
			if (g_quit)
				break;
			sleeps++;
			JTAGGER_SLEEP((50) * MILLISECONDS);
			if (g_quit)
				break;
			continue;
		}

		txchar = fgetc(stdin);
		if (g_quit)
			break;

		if (txchar == esc && !gotesc)
		{
			gotesc = 1;
			continue;
		}

		if (txchar == '\n' && gotesc)
			break;

		if ((txchar == 'u' || txchar == 'U') && gotesc)
		{
			mode = txchar | UPLOAD_UALREADY;	// Flag for upload without need for 'u' prefix
			break;
		}

		gotesc = 0;

		// Else send char
		send_char(txchar, delay);

		if (g_quit)
			break;

		jflush();	// ALWAYS flush before JTAGGER_SLEEP
		JTAGGER_SLEEP((20) * MILLISECONDS);			// wait for response

		if (g_quit)
			break;

		print_rx_buffer(0);

		if (g_quit)
			break;
	}

	if (noncanonical)
		tcsetattr(STDIN_FILENO,TCSANOW, &old_termios);

	// printf("\nterminal_noncanonical sleeps = %d\n", sleeps);	// DEBUG
	return mode;
}
#else
static int terminal_canonical(int delay)
{
	int mode = -1;	// Returned

	printf("\n\nInteractive mode, press ENTER on an empty line to QUIT\n");
	for (;;)
	{
		// Originally meant for interaction with bootloader so only sent first
		// char, but now extended for use as a general terminal interface. Note
		// the special case for a bare 'u' which invokes upload and 'r' for reset
		// Ideally these should be escaped, but that makes usage non-intuitive
		// eg the bootloader prompts for 'u', so would expect that to just work
		// and not need escaping.

		// Using fgets() so need to press ENTER to process input (the alternative
		// invoves setting non-canonical input which is difficult to get right
		// especially cleanup on receipt of signal, eg control-c). This means
		// that behavoir is DIFFERENT to that expected by bootloader, which
		// complains about the unexpected \n character. This is worked-around
		// in the case of u,r,h,e and ? but NOT otherwise else it breaks usage
		// as a terminal for non-bootloader executables

		char line[256];
		fgets(line, sizeof(line), stdin);	// NB only returns on ENTER
		char txchar = line[0];	// Just use first char with bootloader

		if (txchar == '\n')
			break;

		if (txchar == 'u' && line[1] == '\n')	// Handle as special case
		{
			mode = txchar;	// Invokes 'u' procedure below
			break;
		}

		// Handle as special case, we do not send '\n' else bootloader complains about invalid command
		if (strchr("ehr?", txchar) && line[1] == '\n')
		{
			send_char(txchar, delay);
			jflush();	// ALWAYS flush before JTAGGER_SLEEP
			JTAGGER_SLEEP((txchar == 'r' ? 1000 : 50) * MILLISECONDS);	// reset needs longer delay
			print_rx_buffer(0);
			continue;
		}

		// Else send line
		char *p = line;
		while ((txchar = *p++))
			send_char(txchar, delay);
	
		jflush();	// ALWAYS flush before JTAGGER_SLEEP
		JTAGGER_SLEEP((50) * MILLISECONDS);			// wait for response
		print_rx_buffer(0);
	}

	return mode;
}
#endif	// __MINGW32__

static int fpga_riscv(char *uparams, unsigned int chipid)
{
	tap_reset();
	runtest5();

	unsigned long long vdr_ret = 0;

	int mode = -1;			// Flag for no-mode (enables interactive terminal)
	if (uparams)
		mode = uparams[0];	// First char determines mode

	// Get uart status
	vdr_ret = scan_vir_vdr(4, 32, IUART, 0, READMODE);
	printf("uart_status %08" PRIx64 "\n", (uint64_t)vdr_ret);

	int ret_run = vdr_ret & 0x80000000 ? 1 : 0;
	int ret_utxm = vdr_ret & 0x40000000 ? 1 : 0;
	int ret_busy = vdr_ret & 0x20000000 ? 1 : 0;
	int rxpos = (vdr_ret >> 16) & (UBUF_LEN-1);	// mask with size
	printf("run = %d flags_utxm = %d txd_busy = %d rxpos %04x (%d)\n", ret_run, ret_utxm, ret_busy, rxpos, rxpos);

	int default_flags = ret_run ? FLAGS_RUN : 0;	// Retain reset state (where not overriden below)

	if (mode == '?')	// Just wanted status as above
		return 0;

	scan_vir_vdr(4, 32, IRADDR, 0, NOREADMODE);	// Set read address
	scan_vir_vdr(4, 32, IWADDR, 0, NOREADMODE);	// Set write address

	int delay = 1;	// 1 millisecond, currently used in wait_uart() busy loop

	if (mode == 'r')
	{
		scan_vir_vdr(4, 32, IFLAGS, 0, NOREADMODE);	// Enter reset mode
		return 0;
	}

	if (mode == 'c')
	{
		// Clear rx buffer
		scan_vir_vdr(4, 32, IFLAGS, default_flags, NOREADMODE);	// Unset FLAGS_URX (resets rx_addr in fpga)

		memset ((char*)ubuf, 0, sizeof(ubuf));
		bulk_write((char*)ubuf, sizeof(ubuf), IWDATA);	// Address was set above

		scan_vir_vdr(4, 32, IFLAGS, default_flags | FLAGS_URX, NOREADMODE);	// Enable uart_rx
		return 0;
	}

	// All other modes: Ensure reset is not asserted and enable uart rx
	scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN, NOREADMODE);
	jflush();	// ALWAYS flush before JTAGGER_SLEEP

	// Wait for bootloader to initialize (not always necessary, but we don't know
	// the prior state, so need to do it every time)

	JTAGGER_SLEEP(200 * MILLISECONDS);	// 100mS is insufficient to capture initial boot screen

	print_rx_buffer(0);

	if (mode == 'p')	// Just wanted print_rx_buffer()
	{
		printf("\n");
		return 0;
	}

	if (mode == -1)		// No params, so prompt
	{
#ifdef __MINGW32__
		// I'm not going to do noncanonical on windows, roll your own if you want.
		mode = terminal_canonical(delay);
#else
		mode = terminal_noncanonical(delay);
		if (g_quit)
			return 0;
#endif
	}

	// NB not "else if" since falls through from terminal
	if ((mode&0xff) == 'u' || (mode&0xff) == 'U')	// mask off UPLOAD_UALREADY flag
	{
		upload(uparams, mode, delay);
	}
	else if (mode == 'h')
	{
		send_char('h', delay);
		jflush();	// ALWAYS flush before JTAGGER_SLEEP
		JTAGGER_SLEEP(50 * MILLISECONDS);	// Allow longer for bootloader to print prompt via uart
		print_rx_buffer(0);
	}
	else if (mode == 'e')
	{
		send_char('e', delay);
		jflush();	// ALWAYS flush before JTAGGER_SLEEP
		JTAGGER_SLEEP(50 * MILLISECONDS);
		print_rx_buffer(0);
	}
	else if (mode == 'q')	// quiet
	{
		// Disable uart rx, but leave running (RESET not asserted)
		vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_RUN, NOREADMODE);
		return 0;	// skip the default below
	}
	else if (mode == 'b' || mode == -1)
	{
		// Just take default action, so fall through
	}
	else
	{
		printf("Command \"%c\" not recognised\n", mode);
	}

	// NB all modes except 'r' leave processor running (INCLUDING unrecognised modes)

	// Enable uart rx and remove reset
	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_URX | FLAGS_RUN, NOREADMODE);

	return 0;
}

#ifndef __MINGW32__	// MinGW does not have signals, but Cygwin does
void sigterm_handler(int sig)
{
	// Don't try to access JTAG from here, it does not work
	g_quit = 1;

		// struct sigaction sa = { 0 };	// Older gcc warns about this (though it seems to work)
		struct sigaction sa;
		memset (&sa, 0, sizeof(sa));	// So do this instead

		sa.sa_handler = SIG_DFL;
		sigaction(SIGTERM, &sa, 0);
		sigaction(SIGINT, &sa, 0);
		sigaction(SIGHUP, &sa, 0);

	printf("\nCaught signal %d, exiting\n", sig);
}
#endif

int usercode(char *uparams)
{
	if (get_hub_info())
	{
		printf("Unrecognised FPGA configuration (no virtual jtag hub)\n");
		printf("You should load a supported bitstream.\n");
		return ERROR_BADBITSTREAM;
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

	if (id == CHIPID_v4)
	{
		printf("Found riscv fpga bitstream\n");

#ifndef __MINGW32__	// MinGW does not have signals, but Cygwin does
		// struct sigaction sa = { 0 };	// Older gcc warns about this (though it seems to work)
		struct sigaction sa;
		memset (&sa, 0, sizeof(sa));	// So do this instead

		sa.sa_handler = sigterm_handler;	// See sigterm_handler() defined above
		sigaction(SIGTERM, &sa, 0);
		sigaction(SIGINT, &sa, 0);
		sigaction(SIGHUP, &sa, 0);
#endif
		fpga_riscv(uparams, id);
	}
	else
	{
		printf("Unrecognised FPGA configuration (incorrect chip id)\n");
		printf("You should load a supported bitstream.\n");
	}

	tap_reset();
	jflush();

	return 0;
}

