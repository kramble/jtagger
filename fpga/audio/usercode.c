/* usercode.c - custom interface to fpga. This version for fpga/audio.

*/

#include "../../src/common.h"

// Bitstream ientifiers
#define CHIPID_v1 0x97d2f9ce
#define CHIPID_v2 0x97d2f9cf
#define CHIPID_v3 0x97d2f9d0	// fpga/audio

// Instruction Register opcodes, see fpga/txrxmem/defines.v
#define IIDENT	0
#define IRADDR	1
#define IWADDR	2
#define IRDATA	5
#define IWDATA	6
#define IAUDIO	13
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
#define FLAGS_TWRAP 0x00020000
#define FLAGS_APLAY 0x00040000

#define MEMSIZE 16384	// 32 bit words

#if MEMSIZE & (MEMSIZE-1)
#error "MEMSIZE must be a power of 2"	// This is relied on for address validation
#endif

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

//	printf("====================\n");
	printf("hubinfo = 0x%08x\n", hubinfo);
//	printf("====================\n");

	unsigned int expected_hubinfo = 0x08086e04;
	if (hubinfo != expected_hubinfo)
	{
		// Value depends on the FPGA virtual jtag configuration vis VIR size, address size (number of instances)
		printf("\n==================================================\n");
		printf("WARNING hubinfo does not match expected %08x\n", expected_hubinfo);
		printf("Check and update expected value if this is correct\n");
		printf("==================================================\n");
	}

//	printf("\n=====================\n");
	printf("nodeinfo = 0x%08x\n", nodeinfo);
//	printf("=====================\n");

	// Decode the hub and node info

	printf("hub info m = %d mfg = 0x%x n = %d version = %d\n",
				hubinfo & 0xff, (hubinfo>>8) & 0x1ff, (hubinfo>>19) & 0xff, (hubinfo>>27) & 0x1f);

	printf("node info inst = %d mfg = 0x%x id = %d version = %d\n",
				nodeinfo & 0xff, (nodeinfo>>8) & 0x1ff, (nodeinfo>>19) & 0xff, (nodeinfo>>27) & 0x1f);

	if (hubinfo != expected_hubinfo)
		return ERROR_FAIL;

	return 0;

}	// end get_hub_info()

// I don't want to mess about finding the right header on linux, so using this which I found on StackOverflow
// https://stackoverflow.com/questions/23030980/creating-a-stereo-wav-file-using-c

#pragma pack(push, 1)
typedef struct wavfile_header_s
{
    char    ChunkID[4];     /*  4   */
    int32_t ChunkSize;      /*  4   */
    char    Format[4];      /*  4   */
    
    char    Subchunk1ID[4]; /*  4   */
    int32_t Subchunk1Size;  /*  4   */
    int16_t AudioFormat;    /*  2   */
    int16_t NumChannels;    /*  2   */
    int32_t SampleRate;     /*  4   */
    int32_t ByteRate;       /*  4   */
    int16_t BlockAlign;     /*  2   */
    int16_t BitsPerSample;  /*  2   */
    
    char    Subchunk2ID[4];
    int32_t Subchunk2Size;
} wavfile_header_t;
#pragma pack(pop)

static wavfile_header_t header;

static int parse_wave(char *buf, size_t len)
{
	// Returns offset to data, or negative for error

	if (len < sizeof(header))
		return -1;

	memcpy(&header, buf, sizeof(header));

	if (strncmp(header.ChunkID, "RIFF", 4))
		return -1;
	if (strncmp(header.Format, "WAVE", 4))
		return -1;
	if (strncmp(header.Subchunk1ID, "fmt ", 4))
		return -1;

	printf("AudioFormat %d NumChannels %d SampleRate %d\nByteRate %d BlockAlign %d BitsPerSample %d\n",
#ifdef __CYGWIN__
			(int)header.AudioFormat, (int)header.NumChannels, (int)header.SampleRate,
			(int)header.ByteRate, (int)header.BlockAlign, (int)header.BitsPerSample);
#else
			header.AudioFormat, header.NumChannels, header.SampleRate,
			header.ByteRate, header.BlockAlign, header.BitsPerSample);
#endif

	if (header.AudioFormat != 1 || header.NumChannels < 1 || header.NumChannels > 2 || header.BitsPerSample != 16)
	{
		printf("Audio format not supported (stereo S16LE only)\n");
		return -1;
	}

	size_t pos = sizeof(header);
	while (strncmp(header.Subchunk2ID, "data", 4))
	{
		int32_t skip = header.Subchunk2Size;
		pos += skip;
		if (pos > len - 4)
			return -1;
		memcpy(&header.Subchunk2ID, buf+pos, 8);
	}

	if (pos > sizeof(header))	// A simple WAV with just a data chunk does not need adjusting
		pos += 8;

	printf("Data offset %08x (%d)\n", (int)pos, (int)pos);

	return pos;
}

static void	stereoize(short *abuf, int len)
{
	len /= 2;					// Convert length from bytes to shorts
	short *dst = abuf + len;
	short *src = abuf + len/2;	// NB Buffer is half-full on entry
	while (src > abuf)
	{
		*--dst = *src;
		*--dst = *--src;
	}
}

static int fpga_audio(char *uparams, unsigned int chipid)
{
	tap_reset();
	runtest5();

	if (!uparams)
	{
		printf("Need audio filename as -u file\n");
		return ERROR_FAIL;
	}

	FILE *afile = fopen(uparams, "rb");
	if (!afile)
	{
		printf("ERROR opening audio file %s\n", uparams);
		return ERROR_FAIL;
	}

#define ABUF_LEN (16 * 1024)	// 16kB buffer is 1/4 of fpga block RAM
	char abuf[ABUF_LEN];

	size_t bytes_read = fread(abuf, 1, ABUF_LEN, afile);

	int offset = parse_wave(abuf, bytes_read);

	if (offset < 0)
	{
		printf("ERROR can't play %s (format not supported)\n", uparams);
		fclose(afile);
		return ERROR_FAIL;
	}

	fseek(afile, offset, SEEK_SET);

	// int sample_duration = 1134;	// 44100 samples/sec @50MHz DE0-Nano clock
	int sample_duration = 50000000 / header.SampleRate;	// Ought to work for non-CD rates too!
	int playing = 0;
	unsigned long long vdr_ret = 0;

	int waddr = 0;

	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, 0, NOREADMODE);	// Turn off audio play (resets audio pointer)

	vdr_ret = scan_vir_vdr(4, 32, IRADDR, 0, NOREADMODE);	// Set read address
	vdr_ret = scan_vir_vdr(4, 32, IWADDR, 0, NOREADMODE);	// Set write address

	int final = 0;	// This ensures we play all the way to the end (TODO less clunky)

	for(;;)	// main loop
	{
		if (final == 0)
		{
			if (header.NumChannels == 1)
				bytes_read = fread(abuf, 1, ABUF_LEN/2, afile);
			else
				bytes_read = fread(abuf, 1, ABUF_LEN, afile);

			// printf("Read %ld %d\n", bytes_read, ABUF_LEN);

			if (bytes_read < 1)
				break;

			if (header.NumChannels == 1)
			{
				stereoize((short*)abuf, ABUF_LEN);
				bytes_read *= 2;
			}

			if (bytes_read < ABUF_LEN)
				final = 6;	// Needs to be this big due to delay filling buffer
		}
		else if (--final > 0)
		{
			bytes_read = 0;		// Fill with silence
			// printf("final %d\n", final);
		}
		else
			break;

		while (bytes_read < ABUF_LEN)
			 abuf[bytes_read++] = 0;		// pad final read with zeros

#if 1	// It's a bit loud on my earphones so reduce volume
		// BEWARE this also reduces the fidelity since the LSB are discarded
		short *p = (short*)abuf;			// S16LE audio
		for(int i=0; i<ABUF_LEN/2; i++)
		{
			p[i] /= 4;	// Or whatever eg...
			// p[i] = (short) ((int)(p[i]) * 2 / 3);
		}
#endif

		for(;;)
		{
			// Write sample duration and read back current audio pointer (NB different registers)
			vdr_ret = scan_vir_vdr(4, 32, IAUDIO, sample_duration, READMODE);
			int aaddr = vdr_ret & 0x3fff;
	//		printf ("audio pointer %08" PRIx64 " %08x\n", (int64_t)vdr_ret, aaddr);
			if ( !playing ||
				 (aaddr >= waddr + ABUF_LEN/4) || 
				 (waddr == 3 * ABUF_LEN/4 && aaddr < waddr) )
			{
	//			printf("write %08x\n", waddr);
				vdr_ret = scan_vir_vdr(4, 32, IWADDR, waddr, NOREADMODE);	// Set write address
				bulk_write(abuf, ABUF_LEN, IWDATA);		// NB length is bytes cf the ints we need when adjusting waddr
				jflush();
				waddr = (waddr + ABUF_LEN/4) & (MEMSIZE - 1);	// wrap address

				if (playing == 0 && waddr == 3 * ABUF_LEN/4)
				{
					vdr_ret = scan_vir_vdr(4, 32, IFLAGS, FLAGS_APLAY, NOREADMODE);
					jflush();
					playing = 1;
				}
				break;
			}

			JTAGGER_SLEEP (20 * MILLISECONDS);	// At 44100Hz the buffer should fill every 100mS approx
			if (g_quit) break;
		}
		if (g_quit) break;
	}

	vdr_ret = scan_vir_vdr(4, 32, IFLAGS, 0, NOREADMODE);	// Turn off audio play

	fclose(afile);
	return 0;
}

void sigterm_handler(int sig)
{
	// Don't try to access JTAG from here, it does not work
	g_quit = 1;

#ifndef __MINGW32__	// MinGW does not have signals, but Cygwin does
		// struct sigaction sa = { 0 };	// Older gcc warns about this (though it seems to work)
		struct sigaction sa;
		memset (&sa, 0, sizeof(sa));	// So do this instead

		sa.sa_handler = SIG_DFL;
		sigaction(SIGTERM, &sa, 0);
		sigaction(SIGINT, &sa, 0);
		sigaction(SIGHUP, &sa, 0);
#endif

	printf("\nCaught signal %d, exiting\n", sig);
}

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

	if (id == CHIPID_v3)
	{
		printf("Found audio fpga bitstream\n");

		FLAGS_DEBUG = FLAGS_v2_DEBUG;
		FLAGS_RADDR = FLAGS_v2_RADDR;
		FLAGS_WADDR = FLAGS_v2_WADDR;
		FLAGS_WDATA = FLAGS_v2_WDATA;
		FLAGS_RDATA = FLAGS_v2_RDATA;

#ifndef __MINGW32__	// MinGW does not have signals, but Cygwin does
		// struct sigaction sa = { 0 };	// Older gcc warns about this (though it seems to work)
		struct sigaction sa;
		memset (&sa, 0, sizeof(sa));	// So do this instead

		sa.sa_handler = sigterm_handler;	// See sigterm_handler() defined above
		sigaction(SIGTERM, &sa, 0);
		sigaction(SIGINT, &sa, 0);
		sigaction(SIGHUP, &sa, 0);
#endif
		fpga_audio(uparams, id);
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

