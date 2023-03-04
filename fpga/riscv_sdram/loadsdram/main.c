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
						// but adds 10k to exe size! HOW does this work?

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

	unsigned int *mem = (unsigned int*) 0x90000000;	// sdram

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

				//	if (i > size/4 - 4)
				//	if (i < 8)
				//		neorv32_uart0_printf("addr %x val %x\n", i, val);

					mem[i] = val;
					csum_actual += val;
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
			// seems to be stuck at zero, so that's not useful either.
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
