/*
 * mad_neorv32.c - based on minimad.c https://www.underbit.com/products/mad/
 *
 * Original COPYRIGHT declaration follows...
 *
 * libmad - MPEG audio decoder library
 * Copyright (C) 2000-2004 Underbit Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: minimad.c,v 1.4 2004/01/23 09:41:32 rob Exp $
 */

// ---------------------------------------------------------------------

# include <neorv32.h>
# include <string.h>

# include <stdio.h>
# include <unistd.h>
# include <sys/stat.h>

# include "mad.h"

static volatile unsigned int *dma = (unsigned int*) 0xFFFE0000;	// dma memory
static volatile unsigned int *dmc = (unsigned int*) 0xFFFE2000;	// dma control

static volatile unsigned int *adma = (unsigned int*) 0xFFFE4000;	// audio dma memory
static volatile unsigned int *admc = (unsigned int*) 0xFFFE6000;	// audio dma control

// The MP3 frame length is 1152 which won't quite fit into a 4kB buffer, so using 8kB (see verilog)
#define ALEN 2048	// Audio buffer length (int count, 2048 ints is 8kB 0x2000 bytes, 13 bit address)

static int arate = 2267;	// Default to 44100 samples/second at 100MHz clock (NB overwritten below with
							// the correct rate for the current MP3 audio and NEORV32_SYSINFO.CLK)

static int g_nframes, g_nidle, g_lastidle;	// DEBUG stats
static int g_minspace, g_maxspace;
static uint64_t g_total_time, g_idle_time;

static char *g_rxdata_mem;				// Dynamic pointer to current rx position
unsigned int g_rxdata_mem_size;			// Max available space for MP3 data
static unsigned int g_total_received;	// Data received from jtagger

static int g_led = 0x80;	// A bit of user feedback that something is actually happening

static int rxdata_start(void)	// Wait for jtagger to send some data
{
	volatile unsigned int ret = 0;
	unsigned int timeout_max = 2000000;	// NB loop iterations, not clocks, so several 10s of milliseconds
	unsigned int timeout = timeout_max;

	neorv32_gpio_port_set(++g_led);

	while (--timeout)
	{
		if ((ret = *dmc) == 0x11000000)	// Wait for start command
			break;
	}
	
	if (ret == 0x11000000)
	{
		// DEBUG so now omitted
		// neorv32_uart0_printf("rxdata_start received start %x after %d cycles\n", ret, timeout_max - timeout);
	}
	else
	{
		// This is normal until jtagger starts sending, so omit the print
		// neorv32_uart0_printf("ERROR received %x expect %x after %d cycles\n", ret, 0x11000000, timeout_max - timeout);
		return 1;
	}

	*dmc = 0x22000000;	// Respond "ready"
	return 0;
}

#define RXDATA_NEXT_WAIT 1	// pass as dowait

static int rxdata_next(int dowait)	// Called from idle() in output() to poll for jtagger data
{
	// Returns bytes_received for success, -1 for finshed (also success), -2/-3 for error
	// If called without wait, returns 0 if not data available

	volatile unsigned int *pdst = (volatile unsigned int *)g_rxdata_mem;
	volatile unsigned int ret = 0;
	unsigned int timeout_max = 2000000;	// NB loop iterations, not clocks, so several 10s of milliseconds
	unsigned int timeout = timeout_max;

	static unsigned int seq = 3;	// seq must be preserved between calls

	neorv32_gpio_port_set((++g_led) >> 8);	// Slower else it's manic

	if (g_total_received > g_rxdata_mem_size - 0x4000)	// Allow at least one buffer worth of slack
		return -3;	// ERROR no space left

	unsigned int cmdexp = 0;

	timeout = timeout_max;
	cmdexp = 3 << 28 | (seq++ & 0xf) << 24;
	unsigned int finexp = (cmdexp & 0x0fffffff) | 5 << 28;

	while (--timeout)
	{
		ret = *dmc;
		if ((ret & 0xff000000) == cmdexp)	// Wait for data
			break;
		if ((ret & 0xff000000) == finexp)	// Or for finish
			break;
		if (!dowait)
			break;
	}
				
	if ((ret & 0xff000000) == cmdexp)
	{
		// This is SLOW for huge files (and clutters rx buffer) so omit print
		// neorv32_uart0_printf("Received data %x after %d cycles\n", ret, timeout_max - timeout);
	}
	else if ((ret & 0xff000000) == finexp)
	{
		neorv32_uart0_printf("Received finish %x after %d cycles\n", ret, timeout_max - timeout);
		cmdexp = finexp;	// It's checked after the loop
		return -1;
	}
	else
	{
		if (!dowait)
		{
			seq--;
			return 0;
		}
		neorv32_uart0_printf("ERROR received %x expect %x after %d cycles\n", ret, cmdexp, timeout_max - timeout);
		return -2;
	}

	unsigned int bytes_received = ret & 0x00ffffff;
	bytes_received = (bytes_received + 3) & ~3;			// Round it up to maintain buffer alignment

	g_total_received += bytes_received;

	unsigned int buf_offset = 0x0400;	// Must match usercode.c (NB int offset)
	volatile unsigned int *pdma = dma + buf_offset;
	unsigned int words_received = bytes_received / 4;

	while (words_received--)
	{
		if (pdma - dma >= 0x2000)	// Must match verilog
		{
			neorv32_uart0_printf("ERROR pdma bounds exceeded\n");
			return -2;
		}
		*pdst++ = *pdma++;
	}

#if 0	// We rounded-up bytes_received, so this no longer applies
	unsigned int residual =  bytes_received - (bytes_received / 4) * 4;

	if (residual)
	{
		neorv32_uart0_printf("TODO residual %d bytes\n", residual);
		*pdst++ = *pdma++;	// Writes one to three extra bytes (and possibly misaligns)
	}
#endif

	unsigned int resp = 4 << 28 | (seq++ & 0xf) << 24 | bytes_received;
	*dmc = resp;	// Respond

	return bytes_received;
}

// Relocate the synth() .rodata to IMEM at address 00000000 - requires custom linker script to setup segments.
// The O0 optimisation attribute is ESSENTIAL else gcc will NOT do what is requested here and is likely to
// deliberately emit an EBREAK instruction as it thinks we have no business writing to address 00000000.
// Stackoverflow has some juicy comments about this (writing to NULL is a requirement for some embedded environments
// and you don't get much more embedded than we are here). NB O0 was required with similar code located in synth.c and
// global optimisation level O3. I moved the code here so that I don't have to supply a modified synth.c.
// It seems less strict at the current global O1 but I'm keeping it in place as we're out on a wing here. Testing is a
// little awkward as we need to reload the bitstream to clear the IMEM at 00000000 (else it will run quite happily using
// data written by a previous version of the program, even if it itself is broken).

extern char mad_textend;	// End of .text segment so we can locate .rodata (NB char, not *char)

static void __attribute__((optimize("O0"))) relocate_synth_rodata(void)
{
	void *dest = 0;						// Yeah, writing to NULL, deliberately
	size_t size = 0x880;				// See objdump -h main.elf for section size

// gcc gets quite stroppy about what we're doing here (it's less upset if we do the copy ourselves
// rather than using memcpy, but where's the fun in that?)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overread"
#pragma GCC diagnostic ignored "-Warray-bounds"
	memcpy (dest, &mad_textend, size);	// Needs O0 or -fno-delete-null-pointer-checks to avoid EBREAK
#pragma GCC diagnostic pop
}

static int decode(unsigned char *, unsigned long);

int main(int argc, char *argv[])
{

#define BAUD_RATE 19200

	// capture all exceptions and give debug info via UART
	// this is not required, but keeps us safe
	neorv32_rte_setup();

	// Clear audio buffer so we don't play crud while loading initial MP3 data
	arate = NEORV32_SYSINFO.CLK / 44100;
	*admc = arate;
	volatile unsigned int *padma = adma;
	for (int i=0; i<ALEN; i++)
		*padma++ = 0;

	// init UART at default baud rate, no parity bits, no HW flow control
	neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

	// check available hardware extensions and compare with compiler flags
	neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

	neorv32_gpio_port_set(g_led);
 	neorv32_uart0_puts("Starting mad\n");

	relocate_synth_rodata();	// BEWARE requires custom linker script

#if 0	// OLD embedded MP3 data for debugging
	void *fdm;

	static	// "static char mp3data[]" creates MUCH smaller executable than stack (auto) array
	#include "clip.h" // defines mp3data array

	fdm = mp3data;
	unsigned int size = sizeof(mp3data);
#endif

	// Heap is used by malloc() in decoder.c and layer3.c. While we could use the heap for the MP3 buffer, it seems
	// safer to keep it separate so we define a small heap in the linker script and place the MP3 data AFTER the end
	// of the heap, ranging up to the stack at the end of memory. The rom/ram split is at 256kB, so the first 512kB of
	// ram is used for .data, .bss and heap, then our MP3 buffer starts at 512kB into the ram area (address 0x910c0000)

	g_rxdata_mem_size = 31 * 1024 * 1024; 	// 31MB allows 1MB for program text, data, heap and stack

	// Splits the remaining 1M as 750KB for program text, data, heap and 250kB for stack (plenty)
	// Since the stack is at the end of memory, we locate the buffer at offset 750kB (0xc0000)
	// NB as currently configured in mad.ld, the stack is actually relocated to the end of the internal IMEM
	// for speed, so the mp3 buffer could be exended into that area if needed.
	void *fdm = (void*)0x900c0000;			// TODO use the exported linker heap symbols instead

	neorv32_uart0_printf("mp3data size %d\n", g_rxdata_mem_size);

	while (rxdata_start())	// wait for jtagger to send data
	{
		// neorv32_uart0_printf("rxdata_start error retrying ...\n");	// DEBUG (omit since fills uart rx buffer area)
	}

	// neorv32_uart0_printf("rxdata_start good\n");

	// Pre-load some data so the decoder has something to work with
	// The remaining data is loaded from idle() in output()

	g_rxdata_mem = fdm;

	int bytes_received = 0;

	int loops = 8;
	for (int i=0; i<loops; i++)
	{
		bytes_received = rxdata_next(RXDATA_NEXT_WAIT);
		if (bytes_received != 0x1000)	// Expect a full buffer first time
		{
			neorv32_uart0_printf("rxdata_next error %d ... EXIT\n", bytes_received);
			return 1;
		}

		// neorv32_uart0_printf("rxdata_next good, received %d\n", bytes_received);

		g_rxdata_mem += bytes_received;
	}

	neorv32_uart0_printf("buffer preload completed, received %d\n", g_total_received);

    for(;;)	// Repeat forever
    {

		uint64_t tbegin = neorv32_mtime_get_time();

		neorv32_uart0_printf("fdn %x size %d\n", fdm, g_total_received);

		decode(fdm, g_total_received);

		// **** NB decode() no longer returns since input() now streams continuously ****

		// (*** CURRENTLY NOT REACHED ... stats moved to input() ***)

		neorv32_uart0_printf("returned from decode\n");

		uint64_t tend = neorv32_mtime_get_time();

		uint32_t elapsed_ms = (tend - tbegin) / (NEORV32_SYSINFO.CLK/1000);	// truncated from uint64_t to uint32_t

		neorv32_uart0_printf("Elapsed %d mS\n", elapsed_ms);

		neorv32_uart0_printf("nframes %d idle %d lastidle %d g_minspace %d g_maxspace %d\n",
				g_nframes, g_nidle, g_lastidle, g_minspace, g_maxspace);

		neorv32_uart0_printf("Total %d Idle %d Utilization %d percent\n",
				(uint32_t)(g_total_time/1000LL), (uint32_t)(g_idle_time/1000LL),
				(uint32_t)((g_total_time - g_idle_time) / (g_total_time / 100LL)));

		// 125MHz clip_full.h 209.7 seconds elapsed (VLC reports 222 sec!) O3=71% O1=53% Os=58%
		// 100MHz O1=67% (however it sounds slow and scrappy, 243 sec elapsed). Os=59% but 255 sec elapsed! Seems
		// wrong, the idle() spincount may not be working as I excpected, or something else may be skewing it.
		// In any case, the utilization percentage is clearly not at all accurate (and elapsed may not be either)
		// Actual clock time on the 125MHz version (Os)... 210sec so consistent with reported elapsed time.
  }

	// Clear on exit (*** CURRENTLY NOT REACHED ***)
	padma = adma;
	for (int i=0; i<ALEN; i++)
		*padma++ = 0;

	return 0;
}

/*
 * This is a private message structure. A generic pointer to this structure
 * is passed to each of the callback functions. Put here any data you need
 * to access from within the callbacks.
 */

struct buffer {
  unsigned char *start;	// Removed const as it needs far too much castyness
  unsigned long length;
};

/*
 * This is the input callback. The purpose of this callback is to (re)fill
 * the stream buffer which is to be decoded. In this example, an entire file
 * has been mapped into memory, so we just call mad_stream_buffer() with the
 * address and length of the mapping. When this callback is called a second
 * time, we are finished decoding.
 */

static
enum mad_flow input(void *data,
		    struct mad_stream *stream)
{
  struct buffer *buffer = data;

  // Some rather ugly code to manage the input buffer which extends as jtagger loads data
  // Really needs to be refactored, possibly using lower level libmad functions instead of the
  // simple minimad interface (TODO investigate how the "madplay" example code does this)

  // It's not quite right as there is an audible "glitch" after the initial preloaded buffer is
  // processed, though the problem may be in the output idle() processing instead.

  static int init;
  static unsigned char *base;
  static unsigned char *pos;
  static uint64_t tbegin;

  if (!init)
  {
	init++;

	if (!buffer->length)		// From original minimad.c, no harm keeping it (buffer->length is actually
		return MAD_FLOW_STOP;	// discarded in favour of using "g_total_received - (pos - base)" below)

	// The first call occurs from decode() prior to processing the initial preload buffer.
	// Save the start position so we can wrap once play has reached the end
	base = buffer->start;

	pos = buffer->start;	// Setup for processing

	tbegin = neorv32_mtime_get_time();
  }

  // Buffer length dynamically increases until fully loaded, so handle this
  if (!buffer->length)	// zero, ie subsequent calls, not first
  {
    if (pos - base >= g_total_received)
    {
		pos = base; 
		buffer->length = g_total_received;

		// Move stats here since we no longer return to main() from decode()
		uint64_t tend = neorv32_mtime_get_time();
		uint32_t elapsed_ms = (tend - tbegin) / (NEORV32_SYSINFO.CLK/1000);	// truncated from uint64_t to uint32_t
		neorv32_uart0_printf("Elapsed %d mS\n", elapsed_ms);

		neorv32_uart0_printf("nframes %d idle %d lastidle %d g_minspace %d g_maxspace %d\n",
				g_nframes, g_nidle, g_lastidle, g_minspace, g_maxspace);

		neorv32_uart0_printf("Total %d Idle %d Utilization %d percent\n",
				(uint32_t)(g_total_time/1000LL), (uint32_t)(g_idle_time/1000LL),
				(uint32_t)((g_total_time - g_idle_time) / (g_total_time / 100LL)));

		tbegin = neorv32_mtime_get_time();
	}
    else
       buffer->length = g_total_received - (pos - base);
  }

  buffer->start = pos;
  pos = buffer->start + buffer->length;	// for next call

  // Not quite sure how this works, I think it just sets up for the decoder to process the buffer
  mad_stream_buffer(stream, buffer->start, buffer->length);

  buffer->length = 0;	// for next call (handled in test above)

  return MAD_FLOW_CONTINUE;
}

/*
 * Wait for space to become available for audio output
 * Use this time to load mp3 input from jtagger.
 */

static void idle(void)
{
	static int finished;
	static int bytes;
	if (!finished)
	{
		int bytes_received = rxdata_next(0);
		if (bytes_received < 0)
		{
			neorv32_gpio_port_set(0);
			if (bytes_received != -1)
				neorv32_uart0_printf("idle rxdata_next returned error %d, total %d bytes received\n", bytes_received, bytes);
			else
				neorv32_uart0_printf("idle rxdata_next finished, total %d bytes received\n", bytes);
			finished = 1;
		}
		else	// NB returns 0 if no data available, so this is OK
		{
			g_rxdata_mem += bytes_received;
			bytes += bytes_received;
		}
	}
	return;
}

/*
 * The following utility routine performs simple rounding, clipping, and
 * scaling of MAD's high-resolution samples down to 16 bits. It does not
 * perform any dithering or noise shaping, which would be recommended to
 * obtain any exceptional audio quality. It is therefore not recommended to
 * use this routine if high-quality output is desired.
 */

static inline
signed int scale(mad_fixed_t sample)
{
  /* round */
  sample += (1L << (MAD_F_FRACBITS - 16));

  /* clip */
  if (sample >= MAD_F_ONE)
    sample = MAD_F_ONE - 1;
  else if (sample < -MAD_F_ONE)
    sample = -MAD_F_ONE;

  /* quantize */
  return sample >> (MAD_F_FRACBITS + 1 - 16);
}

/*
 * This is the output callback function. It is called after each frame of
 * MPEG audio data has been completely decoded. The purpose of this callback
 * is to output (or play) the decoded PCM audio.
 */

static
enum mad_flow output(void *data,
		     struct mad_header const *header,
		     struct mad_pcm *pcm)
{
  unsigned int nchannels, nsamples;
  mad_fixed_t const *left_ch, *right_ch;

  uint64_t tbegin = neorv32_mtime_get_time();
  uint64_t tidle = 0;

  /* pcm->samplerate contains the sampling frequency */
  int diffrate = arate * pcm->samplerate - NEORV32_SYSINFO.CLK;	// mult is faster than div, hence inverted division
  if (diffrate < 0)
    diffrate = -diffrate;
  if (diffrate > NEORV32_SYSINFO.CLK / 100)	// Check within one percent
  {
	neorv32_uart0_printf("check arate * pcm->samplerate %d == CLK %d diff %d\n",
			arate * pcm->samplerate,  NEORV32_SYSINFO.CLK, diffrate);
	arate = NEORV32_SYSINFO.CLK / pcm->samplerate;
    *admc = arate;
	neorv32_uart0_printf("set samplerate %d arate %d\n", pcm->samplerate, arate);
  }

  nchannels = pcm->channels;
  nsamples  = pcm->length;
  left_ch   = pcm->samples[0];
  right_ch  = pcm->samples[1];

  static int pidx;		// Previous

  static int init;
  if (!init)
  {
	init = 1;
	int aidx = *admc;	// Current audio buffer hardware address
	pidx = aidx + 16;	// Start writing slightly ahead of current position (arbitary 16)
	if (pidx >= ALEN)
		pidx -= ALEN;
	neorv32_uart0_printf("init samplerate %d channels %d, length %d, pidx %d\n",
					pcm->samplerate, nchannels, nsamples, pidx);
  }

  g_nframes++;

  while (nsamples--)
  {
	unsigned int sample;
	unsigned int audioout;

	sample = scale(*left_ch++) & 0xffff;
	audioout = sample << 16;			// stereo left channel / mono

	if (nchannels == 2)
		sample = scale(*right_ch++) & 0xffff;	// stereo right channel

	audioout |= sample;					// stereo, also applies mono to both channels if appropriate

	for(;;)
	{
		uint64_t tloop = neorv32_mtime_get_time();
		int aidx = *admc;	// Current audio buffer hardware address
		int space = aidx - pidx;
		if (space < 0)
			space += ALEN;

		if (space > g_maxspace)
			g_maxspace = space;

		if (space < g_minspace)
			g_minspace = space;

		if (space < 16)
		{
			if (g_lastidle != g_nframes)
				g_nidle++;

			g_lastidle = g_nframes;

			idle();		// Use this time to load mp3 input from jtagger

			tidle += neorv32_mtime_get_time() - tloop;	// HMMM, seems rather inaccurate
			continue;
		}

		adma[pidx++] = audioout;
		if (pidx >= ALEN)
			pidx -= ALEN;	// Could just set to 0
		break;
	}
  }	// End while (nsamples--) 

  uint64_t tend = neorv32_mtime_get_time();

  g_total_time += tend - tbegin;
  g_idle_time += tidle;

  return MAD_FLOW_CONTINUE;
}

/*
 * This is the error callback function. It is called whenever a decoding
 * error occurs. The error is indicated by stream->error; the list of
 * possible MAD_ERROR_* errors can be found in the mad.h (or stream.h)
 * header file.
 */

static
enum mad_flow error(void *data,
		    struct mad_stream *stream,
		    struct mad_frame *frame)
{
  struct buffer *buffer = data;

  fprintf(stderr, "decoding error 0x%04x (%s) at byte offset %u\n",
	  stream->error, mad_stream_errorstr(stream),
	  stream->this_frame - buffer->start);

  /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */

  return MAD_FLOW_CONTINUE;
}

/*
 * This is the function called by main() above to perform all the decoding.
 * It instantiates a decoder object and configures it with the input,
 * output, and error callback functions above. A single call to
 * mad_decoder_run() continues until a callback function returns
 * MAD_FLOW_STOP (to stop decoding) or MAD_FLOW_BREAK (to stop decoding and
 * signal an error).
 */

static
int decode(unsigned char *start, unsigned long length)
{
  struct buffer buffer;
  struct mad_decoder decoder;
  int result;

  /* initialize our private message structure */

  buffer.start  = start;
  buffer.length = length;

  /* configure input, output, and error functions */

  mad_decoder_init(&decoder, &buffer,
		   input, 0 /* header */, 0 /* filter */, output,
		   error, 0 /* message */);

  /* start decoding */

  result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);

  /* release the decoder */

  mad_decoder_finish(&decoder);

  return result;
}
