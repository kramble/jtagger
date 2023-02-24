/* audiomain.c

TODO customise for fpga/audio

*/

#include "../../src/common.h"

static void usage(void)
{
	printf("Usage: jtagaudio --help -v -y -p filename.svf -r filename.rbf -u params\n");
}

static void help(void)
{
	printf(
"Usage: jtaggaudio --help -v -p filename.svf -r filename.rbf -u params\n\n"
"A standalone jtag driver for the DE0-Nano (Quartus is not required).\n"
"AUDIO version, enter WAV filename for -u params eg jtaggaudio -u file.wav\n"
"Should now also accept jtaggaudio file.wav\n"

"OPTIONS\n"
"-v sets verbose mode.\n"
"-p will program a .svf file (default %s), likely BUGGY (use -r instead)\n"
"-r will program a .rbf file (default %s), must not be compressed.\n"
"-y autoconfirm programming\n"
"-u file.wav (play WAV file)\n"
"NB only Altera/Intel Quartus .svf programming files are supported as the svf\n"
"parsing is very crude, tested on Quartus 10.1 (other versions may not work).\n\n"

"Employs code from OpenOCD and OpenFPGALoader under the GPL license.\n"
"You may find those projects more useful than jtagger which was written as a\n"
"personal project to drive a vitual jtag hub without needing Quartus installed.\n"
"Nevertheless, you may be pleasantly surprised at just how FAST it programs!\n"
, PROGRAMFILE_S, PROGRAMFILE_R);
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
	int yes = 0;
	int filetype = FILETYPE_NONE;
	char *fname = NULL;
	char *uparams = NULL;

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
			// Client/Server does NOT work for audio streaming (it's too slow and thus stutters)
			printf("-s mode is DEPRECIATED, do not use (switch IGNORED)\n");
			argc--;
			argv++;
			continue;
		}

		if (!strcmp(argv[1], "-y"))
		{
			yes = 1;
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

		if (!strncmp(argv[1], "-u", 2))
		{
			if (argv[1][2])			// a character following u indicates concatenated parameters
				uparams = argv[1]+2;
			else if (argc > 2)		// params is next option
			{
				uparams = argv[2];
				argc--;
				argv++;
				// decrement again below
			}
			argc--;
			argv++;
			continue;
		}

#if 0
		printf("option(s) not recognised or invalid\n");
		usage();
		return 1;
#else
		// Instead just pass it as uparam (TODO better argument processing, eg multiple files)
		uparams = argv[1];
		argc--;
		argv++;
#endif
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
			return (program_fpga(fname, filetype, device_index, yes));

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
			ret = usercode(uparams);
			// printf("jtagger exit with status %d\n", ret);
			break;	// Omit this break in order to to loop regardless of successful result
		}
		JTAGGER_SLEEP (2000 * MILLISECONDS);	// 2 seconds between retries
	}

	// Finish with TAP_RESET (safe since server will reject if FTDI not open, but check ftdi_ok anyway
	// as server does complain which could be distracting)
	if (g_ftdi_ok)
	{
		tap_reset();
		jflush();
	}

	// Print timeout stats
	// NB if g_flushrx_timeout > 0, then g_flushrx_maxdelay will always be 99
	// printf("io_check timeouts %d maxdelay %d\n", g_flushrx_timeout, g_flushrx_maxdelay);

#if 0	// Debug stuff
	printf("\nJtagger exit with status %d\n", ret);

#ifdef WANT_CLOCK_GETTIME
	printf("FTDI read cumulative %d.%.9" PRId64 " seconds\n", (int)g_cumulative_read_time.tv_sec,
				 g_cumulative_read_time.tv_nsec);
#endif

	printf("respondlen %d write ops %d bytes %d read ops %d bytes %d\n",
					g_respond_len, g_wop, g_wbyte, g_rop, g_rbyte);
#endif

	return ret;
}
