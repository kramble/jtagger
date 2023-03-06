// #################################################################################################
// # << NEORV32 - loadsdram - upload to sdram at 0x9000000 (based on bootloader.c)                 #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file hello_world/main.c
 * @author Stephan Nolting
 * @brief Classic 'hello world' demo program.
 **************************************************************************/

#include <neorv32.h>
#include "neorv32_mtime.h"

// #include <stdio.h>	// So how would this integrate with neorv32 ... printf() is allowed
						// but adds 10k to exe size! HOW does this work? ...
						// AHA see sw/lib/source/syscalls.c _read(), _write() etc

/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/



/**********************************************************************//**
 * Main function; prints some fancy stuff via UART.
 *
 * @note This program requires the UART interface to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/

// From bootloader.c
#define PRINT_TEXT(...) neorv32_uart0_puts(__VA_ARGS__)
#define PRINT_XNUM(a) print_hex_word(a)
#define PRINT_GETC(a) neorv32_uart0_getc()
#define PRINT_PUTC(a) neorv32_uart0_putc(a)

uint32_t get_exe_word (void)
{
  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } data;

  uint32_t i;
  for (i=0; i<4; i++) {
      data.uint8[i] = (uint8_t)PRINT_GETC();
  }

  return data.uint32;
}

void start_app(uint32_t exe_addr)
{
  // deactivate global IRQs
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // register uint32_t app_base = NEORV32_SYSINFO.ISPACE_BASE; // default = start at beginning of IMEM

  PRINT_TEXT("Booting from ");
  //  PRINT_XNUM(exe_addr);				// barfs!
  neorv32_uart0_printf("%x", exe_addr);	// use this instead
  PRINT_TEXT("...\n\n");

  // wait for UART0 to finish transmitting
  while (neorv32_uart0_tx_busy());

  // start application
  asm volatile ("jalr ra, %0" : : "r" (exe_addr));

  __builtin_unreachable();
  while (1); // should never be reached
}

static void adler_checksum (unsigned char *buf, unsigned size, unsigned int *checksum)
{
	// Bytewise Adler checksum - NB buf MUST be unsigned char (signed gives wrong result)
	unsigned int a = *checksum & 0xffff;
	unsigned int b = *checksum >> 16;
	while (size--)
	{
		// a = (a + *buf++) % 65521;	// simple version (slow due to %)
		// b = (b + a) % 65521;

		a += *buf++;					// faster version (see zlib for a more complex, even faster algorithm)
		if (a >= 65521)
			a -= 65521;
		b += a;
		if (b >= 65521)
			b -= 65521;
	}
	*checksum = b << 16 | a;
}


int main() {

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // init UART at default baud rate, no parity bits, no HW flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch


	int led = 8;

 	neorv32_uart0_puts("Starting loadsdram\n");

	volatile unsigned int *mem = (unsigned int*) 0x90000000;	// sdram
	volatile unsigned int *dma = (unsigned int*) 0xFFFE0000;	// dma memory
	volatile unsigned int *dmc = (unsigned int*) 0xFFFE2000;	// dma control

	int badsig = 0;
	int exe_ok = 0;
	unsigned int size = 0;
	unsigned int csum_expect = 0;
	unsigned int csum_actual = 0;

	while (1)
	{
 		neorv32_gpio_port_set(++led);

		PRINT_TEXT("> ");
		char c = PRINT_GETC();

		if (c == 's')
		{
			PRINT_TEXT("Got s\n");	// Just print some status
			neorv32_uart0_printf("HEADER size %x checksum %x\n", size, csum_expect);
			neorv32_uart0_printf("badsig=%d exe_ok=%d\n", badsig, exe_ok);
			continue;
		}

		if (c == 'c')	// DMA control
		{
			PRINT_TEXT("Got c\n");
			volatile unsigned int* pdmc = dmc;	// yep, caught me out, MUST be volatile
			unsigned int val;
			val = *pdmc;
			neorv32_uart0_printf("DMC addr %x read init %x\n", pdmc, val);
			val = 0x1235600 + led;
			*pdmc = val;
			neorv32_uart0_printf("DMC addr %x wrote val %x\n", pdmc, val);
			val = *pdmc;
			neorv32_uart0_printf("DMC addr %x read back %x\n", pdmc, val);
			continue;
		}

		if (c == 'd')	// DMA data (took WAY too long to debug, FLAGS_URX being the problem here)
		{
			int store[64];
			unsigned int i=0, val, val1, val2, val3, val0;
			unsigned int errcount = 0, erraddr = 9999;
			volatile unsigned int* pdma = dma;	// yep, caught me out, MUST be volatile

			for (i=0; i<sizeof(store)/sizeof(int); i++)
			{
				val1 = *pdma;
				store[i] = val1;
				val = 0x3a3b3c40 + i;
				if (i == 0)
					val0 = val;
				*pdma = val;
				val2 = *pdma;		// without volatile, gcc just "remembers" what it wrote instead
				if (val2 != val)	// This never fails
				{
					errcount++;
					erraddr = i;
				}
				volatile unsigned int* p0dma = dma;
				val2 = *p0dma;
				if (val2 != val0)	// This fails 63 times, first at i==1
				{
					errcount+=1000;
					if (erraddr == 9999)
						erraddr = i;
				}
				// CONCLUDE FFFE0000 is written, read back OK once, then subsequently reads as 00000000
				// So how does it get overwritten? AHA on the next iteration, else it stays unchanged
				// it we only execute the loop once. Possibly due to the delay pipeline in verilog. Phew!

				pdma++;	// NB index by int
			}

			// Check addr 0 again, see usercode.c (the expected value is not being read there)
			// NB defer the print since the first few characters are lost in uart transfer until FLAGS_URX is set
			pdma = dma;
			val3 = *pdma;	// Reads 00000000, so it did NOT get written! However setting dma = 0xFFFE0004
							// does work! Something strange is happening here.

			// Print AFTER so as not to interfere with buffer, except I forgot that FLAGS_URX gets set during terminal
			// operation, so the write NEVER OCCURS! Have to send 'd' as an explicit jtagger "function" without FLAGS_URX.

			PRINT_TEXT("Got d\n");
			neorv32_uart0_printf("iterated %d\n", i);
			neorv32_uart0_printf("DMA read init %x\n", val1);
			neorv32_uart0_printf("DMA wrote val %x\n", val);
			neorv32_uart0_printf("DMA read back %x\n", val2);
			for (int i=0; i<16; i++)
				neorv32_uart0_printf("addr %d read %x\n", i, store[i]);

			neorv32_uart0_printf("recheck (pre output)  addr %d read %x\n", 0, val3);
			neorv32_uart0_printf("recheck (post output) addr %d read %x\n", 0, *dma);	// yep, uart rx has overwritten it
			neorv32_uart0_printf("errcount %d erraddr %d\n", errcount, erraddr);

			continue;
		}

// #define NOEXECUTE		// While debugging
#ifdef  NOEXECUTE
		if (c == 'e')
		{
			PRINT_TEXT("Got e NOT executing (use CAPITAL E)\n");	// Don't want auto execute after upload
			neorv32_uart0_printf("HEADER size %x checksum %x\n", size, csum_expect);
			neorv32_uart0_printf("badsig=%d exe_ok=%d\n", badsig, exe_ok);
			continue;
		}
#else
		if (c == 'e')
		{
			PRINT_TEXT("Got e\n");
			if (exe_ok)
			{
				PRINT_TEXT("EXECUTE\n");
				start_app((uint32_t)mem);
			}
	
			PRINT_TEXT("NOT exe_ok\n");
			continue;
		}
#endif

		if (c == 'E')	// capital E since jtagger upload sends 'e'
		{
			PRINT_TEXT("Got E\n");
			if (exe_ok)
			{
				PRINT_TEXT("EXECUTE\n");
				start_app((uint32_t)mem);
			}
	
			PRINT_TEXT("NOT exe_ok\n");
			continue;
		}

		if (c == 'f')	// fast sdram upload
		{
			PRINT_TEXT("Got f\n");

			PRINT_TEXT("Fast uploading, waiting for .bin\n");

			uint32_t tbegin = NEORV32_MTIME.TIME_LO;

			badsig = 0;
			exe_ok = 0;

			volatile unsigned int *pdst = mem;
			volatile unsigned int ret = 0;
			unsigned int timeout_max = 2000000;	// NB loop iterations, not clocks, so several 10s of milliseconds
			unsigned int timeout = timeout_max;

			while (--timeout)
			{
				if ((ret = *dmc) == 0x11000000)	// Wait for start command
					break;
			}
			
			if (ret == 0x11000000)
				neorv32_uart0_printf("Received start %x after %d cycles\n", ret, timeout_max - timeout);
			else
			{
				neorv32_uart0_printf("ERROR received %x expect %x after %d cycles\n", ret, 0x11000000, timeout_max - timeout);
				continue;	// Aborts loading
			}

			*dmc = 0x22000000;	// Respond "ready"

			unsigned int cmdexp = 0;
			unsigned int seq = 3;
			unsigned int total_received = 0;
			int errfail = 0;	// flag to force quit from inner block

			for(;;)
			{
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
				}
				
				if ((ret & 0xff000000) == cmdexp)
				{
					// This is SLOW for huge files (and clutters rx buffer) so omit it
					// neorv32_uart0_printf("Received data %x after %d cycles\n", ret, timeout_max - timeout);
				}
				else if ((ret & 0xff000000) == finexp)
				{
					neorv32_uart0_printf("Received finish %x after %d cycles\n", ret, timeout_max - timeout);
					cmdexp = finexp;	// It's checked after the loop
					break;
				}
				else
				{
					neorv32_uart0_printf("ERROR received %x expect %x after %d cycles\n", ret, cmdexp, timeout_max - timeout);
					errfail = 1;
					break;
				}

				unsigned int bytes_received = ret & 0x00ffffff;
				total_received += bytes_received;

				unsigned int buf_offset = 0x0400;	// Must match usercode.c (NB int offset)
				volatile unsigned int *pdma = dma + buf_offset;
				unsigned int words_received = bytes_received / 4;
				unsigned int residual =  bytes_received - (bytes_received / 4) * 4;

				while (words_received--)
				{
					if (pdma - dma >= 0x2000)	// Must match verilog
					{
						neorv32_uart0_printf("ERROR pdma bounds exceeded\n");
						errfail = 1;
						break;
					}
#if 0	// DEBUG print first three ints received (header)
					static int count;
					if (count++ < 3)
						neorv32_uart0_printf("dma addr %x val %x\n", pdma, *pdma);
#endif				
					*pdst++ = *pdma++;
				}

				if (errfail)
					break;

				if (residual)
					neorv32_uart0_printf("TODO residual %d bytes\n", residual);

				unsigned int resp = 4 << 28 | (seq++ & 0xf) << 24 | bytes_received;
				*dmc = resp;	// Respond

			}	// end for

			if (errfail)
				continue;	// Aborts loading

			if ((ret & 0xff000000) != cmdexp)
			{
				neorv32_uart0_printf("FAILED ret = %x\n", ret);
				continue;
			}

			volatile unsigned int *pend = pdst;
			if (pend - mem != total_received / 4)
			{
				neorv32_uart0_printf("Bad pointers end is %x expected %x\n", pend - mem, total_received / 4);
				// continue;
			}

			// Check signature, length, checksum

			pdst = mem;	// Reset to start
			unsigned int sig = *pdst++;

			if (sig == 0x4788cafe)
				neorv32_uart0_printf("GOOD signature %x\n",sig);
			else
			{
				badsig = 1;
				neorv32_uart0_printf("BAD signature %x\n",sig);
				// continue;
			}

			size = *pdst++;
			csum_expect = *pdst++;

			if ((pend - mem) * 4 - 12 != size)	// Adjust ints to bytes and skip header
			{
				neorv32_uart0_printf("SIZE BAD received %x header %x\n", (pend - mem) * 4 - 12, size);
				// continue;
			}

			// neorv32_uart0_printf("DEBUG EXIT\n");
			// continue;	// DEBUG

			unsigned int val = 0;
			
			for (int i=0; i<size/4; i++)
			{
				val = *pdst++;
				csum_actual += val;	// That's NOT a proper checksum! Adler would be better.
			}
				
			if (csum_actual + csum_expect == 0)	// OOPS, it's not "expect" after all, they sum to zero
			{
				PRINT_TEXT("CHECKSUM OK\n");
				exe_ok = 1;
			}
			else
				neorv32_uart0_printf("CHECKSUM BAD got %x header %x sum %x\n",
						csum_actual, csum_expect, csum_actual + csum_expect);

			unsigned int adler = 1;	// Init to 1 is a requirement for Adler checksum
			adler_checksum((unsigned char*)mem, total_received, &adler);	// NB checksum of entire file including header

			neorv32_uart0_printf("Adler checksum %x expected (ignore 8 msb) %x %s\n", adler, ret & 0x00ffffff,
					(adler & 0x00ffffff) == (ret & 0x00ffffff) ? "OK" : "BAD");

			uint32_t tend = NEORV32_MTIME.TIME_LO;

			// TODO use NEORV32_MTIME.TIME_HI for wrapped values, though the following handles rollover just fine
			// for short durations (< 40 seconds at 100MHz clock)
			neorv32_uart0_printf("Time: begin %x end %x elapsed %x (%d) = %d mS\n",
					tbegin, tend, tend-tbegin, tend-tbegin, (tend-tbegin) / (NEORV32_SYSINFO.CLK/1000));

			// Remove the header
			volatile unsigned int *psrc = mem + 3;
			pdst = mem;
			for (int i=0; i<size/4; i++)
				*pdst++ = *psrc++;
		}

		if (c == 'u')	// sdram upload
		{
			PRINT_TEXT("Got u\n");

			PRINT_TEXT("Uploading, waiting for .bin\n");

			uint32_t tbegin = NEORV32_MTIME.TIME_LO;

			badsig = 0;
			exe_ok = 0;

			int i;
			for (i=0; i<4; i++)
			{
				unsigned char byte = PRINT_GETC();

				// Check signature (one byte at a time so we give early feedback)
				if (i==0 && byte != 0xfe)
					break;
				if (i==1 && byte != 0xca)
					break;
				if (i==2 && byte != 0x88)
					break;
				if (i==3 && byte != 0x47)
					break;
			}

			if (i != 4)
				badsig = 1;
			else
			{
				size = get_exe_word();
				csum_expect = get_exe_word();

				// perhaps this corrupts uart rx? Yep, don't do it then ...
				// neorv32_uart0_printf("HEADER size %x checksum %x\n", size, csum_expect);

				unsigned int val = 0;

				for (i=0; i<size/4; i++)
				{
					val = get_exe_word();
					mem[i] = val;
					csum_actual += val;	// That's NOT a proper checksum! Adler would be better.
				}
				
				neorv32_uart0_printf("final addr %x data %x\n", i, val);

				if (csum_actual + csum_expect == 0)	// OOPS, it's not "expect" after all
				{
					PRINT_TEXT("CHECKSUM OK\n");
					exe_ok = 1;
				}
				else
					neorv32_uart0_printf("CHECKSUM BAD got %x header %x sum %x\n",
							csum_actual, csum_expect, csum_actual + csum_expect);
			}

			uint32_t tend = NEORV32_MTIME.TIME_LO;

			// Not really very helpful as it wraps every 40 seconds, and the NEORV32_MTIME.TIME_HI
			// seems to be stuck at zero. ADDENDUM no I'm wrong it does increment. TODO use it.
			neorv32_uart0_printf("begin %x end %x duration %x (%d)\n", tbegin, tend, tend-tbegin, tend-tbegin);

		}

		if (c == 't')
		{

			uint64_t now = neorv32_mtime_get_time();
			uint32_t nowhi = (now >> 32);
			uint32_t nowlo = now;
			neorv32_uart0_printf("time.high %x time.low %x\n", nowhi, nowlo);
			unsigned int *t = (unsigned int*) &now;
			nowhi = t[1];
			nowlo = t[0];
			neorv32_uart0_printf("time.high %x time.low %x\n", nowhi, nowlo);
		}

		if (c == 'T')
		{

			uint32_t nowhi = NEORV32_MTIME.TIME_HI;
			uint32_t nowlo = NEORV32_MTIME.TIME_LO;
			neorv32_uart0_printf("time.high %x time.low %x\n", nowhi, nowlo);
		}
	}

	return 0;
}
