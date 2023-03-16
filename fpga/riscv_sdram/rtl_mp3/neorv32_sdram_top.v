/* neorv32_sdram_top.v

Based on  https://github.com/emb4fun/neorv32-de0n-ref

LICENSED copyright notice for code samples ...

-- ****************************************************************************
-- *  Copyright (c) 2021 by Michael Fischer (www.emb4fun.de) 
-- *  All rights reserved.
-- *
-- *  Redistribution and use in source and binary forms, with or without 
-- *  modification, are permitted provided that the following conditions 
-- *  are met:
-- *  
-- *  1. Redistributions of source code must retain the above copyright 
-- *     notice, this list of conditions and the following disclaimer.
-- *  2. Redistributions in binary form must reproduce the above copyright
-- *     notice, this list of conditions and the following disclaimer in the 
-- *     documentation and/or other materials provided with the distribution.
-- *  3. Neither the name of the author nor the names of its contributors may 
-- *     be used to endorse or promote products derived from this software 
-- *     without specific prior written permission.
-- *
-- *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
-- *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
-- *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
-- *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
-- *  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
-- *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
-- *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
-- *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
-- *  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
-- *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
-- *  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
-- *  SUCH DAMAGE.
-- *
-- ****************************************************************************

Output is on pins 1 and 2 of the JP1 GPIO connector, ASCII ART follows...

             L R      GND
             | |       |
        jp1  x x o o o x o o o...
[*] [*]      o o o o o o o o o...
 ^   ^
blue+green          [ row of green leds ]
power/usb led

*/

`include "defines.v"

module de0_nano (
	// -- Global control --
	input	clk_i,		// 50MHz clock
	input	rstn_i,		// Button, active low (KEY0 the right-hand one)

	// -- GPIO --
	output [7:0] gpio_o,

	output		audio_left_,
	output		audio_right_,

	output      SDRAM_CLK,   // Master Clock
	output      SDRAM_CKE,   // Clock Enable    
	output      SDRAM_CS_N,  // Chip Select
	output      SDRAM_RAS_N, // Row Address Strobe
	output      SDRAM_CAS_N, // Column Address Strobe
	output      SDRAM_WE_N,  // Write Enable
	output      SDRAM_DQML,  // Output Disable / Write Mask (low)
	output      SDRAM_DQMU,  // Output Disable / Write Mask (high)
	output [12:0] SDRAM_ADDR,// Address output (12 bits)
	output      SDRAM_BA_0,  // Bank Address 0
	output      SDRAM_BA_1,  // Bank Address 1
	inout [15:0] SDRAM_DQ    // Data I/O (16 bits)
);

	localparam BAUDRATE = 19200;

	wire clk, dclk, pll_locked, reset_n;
	wire cpu_uart0_rxd_i, cpu_uart0_txd_o;
	wire [7:0] cpu_gpio_o;
	reg reset_n_d = 1'b0;

	// Wishbone neorv32
	wire [2:0]	wb_tag_o;	// request tag
	wire [31:0]	wb_adr_o;	// address
	wire [31:0]	wb_dat_i;	// read data
	wire [31:0]	wb_dat_o;	// write data
	wire		wb_we_o;	// read/write
	wire [3:0]	wb_sel_o;	// byte enable
	wire		wb_stb_o;	// strobe
	wire		wb_cyc_o;	// valid cycle
	wire		wb_ack_i;	// transfer acknowledge
	wire		wb_err_i;	// transfer error

	// Wishbone control
	wire		wbs_we_i;
	wire [31:0]	wbs_dat_i;
	wire [3:0]	wbs_sel_i;

	// Wishbone sdram
	wire		wbs1_stb_i;
	wire		wbs1_ack_o;
	wire [27:0]	wbs1_adr_i;
	wire [31:0]	wbs1_dat_o;

	// Wishbone jtagger
	wire		wbs2_stb_i;
	wire		wbs2_ack_o;
	wire [15:0]	wbs2_adr_i;
	wire [31:0]	wbs2_dat_o;

	wire [1:0]	sdram_ba;
	wire [1:0]	sdram_dqm;

`ifndef SIM
//	localparam CLK_FREQUENCY = 100_000_000;	// NB also set in wb_sdram.vhd AND neorv32.vhd
//	localparam CLOCK_MULTIPLIER = 2;
//	localparam CLOCK_DIVIDER = 1;
//	localparam CLOCK_DIVIDER_DAC = 8;

//	125MHz requires modified clk1_phase_shift in main_pll.v (also fails setup timing on slow models)
	localparam CLK_FREQUENCY = 125_000_000;	// NB also set in wb_sdram.vhd AND neorv32.vhd
	localparam CLOCK_MULTIPLIER = 5;
	localparam CLOCK_DIVIDER = 2;
	localparam CLOCK_DIVIDER_DAC = 16;

//	133MHz does not work (perhaps modified clk1_phase_shift in main_pll.v would fix it)
//	localparam CLK_FREQUENCY = 133_333_333;	// NB also set in wb_sdram.vhd AND neorv32.vhd
//	localparam CLOCK_MULTIPLIER = 8;
//	localparam CLOCK_DIVIDER = 3;

	assign reset_n = rstn_i & flags_run & pll_locked;
	assign wb_err_i = 1'b0;

	main_pll #(	.CLOCK_MULTIPLIER(CLOCK_MULTIPLIER),
				.CLOCK_DIVIDER(CLOCK_DIVIDER),
				.CLOCK_DIVIDER_DAC(CLOCK_DIVIDER_DAC)) u_pll
	(
		.inclk0 (clk_i),
		.c0     (clk),			// 100 MHz
		.c1     (SDRAM_CLK),	// 100 MHz @phase -1500 (may need to tweak for 133MHz)
		.c2     (dclk),			// dac	
		.pll_locked (pll_locked)
	);

	neorv32_test_setup_bootloader u_riscv
	(
		.clk_i			(clk),
		.rstn_i			(reset_n_d),
		.gpio_o			(cpu_gpio_o),
		.uart0_rxd_i	(cpu_uart0_rxd_i),
		.uart0_txd_o	(cpu_uart0_txd_o),

		.wb_tag_o		(wb_tag_o),
		.wb_adr_o		(wb_adr_o),
		.wb_dat_i		(wb_dat_i),
		.wb_dat_o		(wb_dat_o),
		.wb_we_o		(wb_we_o),
		.wb_sel_o		(wb_sel_o),
		.wb_stb_o		(wb_stb_o),
		.wb_cyc_o		(wb_cyc_o),
		.wb_ack_i		(wb_ack_i),
		.wb_err_i		(wb_err_i)
	);

	wb_intercon u_wbi
	(
		.clk_i			(clk),
		.rst_i			(!reset_n),

		// Wishbone neorv32
		.wbm_stb_i		(wb_stb_o),
		.wbm_cyc_i		(wb_cyc_o),
		.wbm_we_i		(wb_we_o),
		.wbm_ack_o		(wb_ack_i),
		.wbm_adr_i		(wb_adr_o),
		.wbm_dat_i		(wb_dat_o),
		.wbm_dat_o		(wb_dat_i),
		.wbm_sel_i		(wb_sel_o),

		// Wishbone control
		.wbs_we_o		(wbs_we_i),
		.wbs_dat_o		(wbs_dat_i),
		.wbs_sel_o		(wbs_sel_i),

		// Wishbone sdram
		.wbs1_stb_o		(wbs1_stb_i),
		.wbs1_ack_i		(wbs1_ack_o),
		.wbs1_adr_o		(wbs1_adr_i),
		.wbs1_dat_i		(wbs1_dat_o),

		// Wishbone jtagger
		.wbs2_stb_o		(wbs2_stb_i),
		.wbs2_ack_i		(wbs2_ack_o),
		.wbs2_adr_o		(wbs2_adr_i),
		.wbs2_dat_i		(wbs2_dat_o)
	);

		// Wishbone SDRAM Controller

	wb_sdram u_wbc
	(
		.clk_i			(clk),
		.rst_i			(!reset_n),

		// Wishbone
		.wbs_stb_i		(wbs1_stb_i),
		.wbs_we_i		(wbs_we_i),
		.wbs_sel_i		(wbs_sel_i),
		.wbs_adr_i		(wbs1_adr_i),
		.wbs_dat_i		(wbs_dat_i),
		.wbs_dat_o		(wbs1_dat_o),
		.wbs_ack_o		(wbs1_ack_o),

		// SDRAM
		.sdram_addr		(SDRAM_ADDR),
		.sdram_ba		(sdram_ba),
		.sdram_cas_n	(SDRAM_CAS_N),
		.sdram_cke		(SDRAM_CKE),
		.sdram_cs_n		(SDRAM_CS_N),
		.sdram_dq		(SDRAM_DQ),
		.sdram_dqm		(sdram_dqm),
		.sdram_ras_n	(SDRAM_RAS_N),
		.sdram_we_n		(SDRAM_WE_N)
	);

		assign SDRAM_BA_1 = sdram_ba[1];
		assign SDRAM_BA_0 = sdram_ba[0];

		assign SDRAM_DQMU = sdram_dqm[1];
		assign SDRAM_DQML = sdram_dqm[0];  

`else	// SIMULATING ...

	localparam CLK_FREQUENCY = BAUDRATE * 32;	// Speed up simulation

	assign clk = clk_i;
	assign SDRAM_CLK = clk_i;
	assign pll_locked = 1'b1;
	assign reset_n = rstn_i & flags_run & pll_locked;

	// Wishbone stimulus 

	reg [31:0]	wbs_dat_i_reg	= 0;
	reg [15:0]	wbs2_adr_i_reg	= 0;
	reg [3:0]	wbs_sel_i_reg	= 4'b1111;
	reg			wbs_we_i_reg	= 0;
	reg			wbs2_stb_i_reg	= 0;

	assign wbs_dat_i	= wbs_dat_i_reg;
	assign wbs2_adr_i	= wbs2_adr_i_reg;
	assign wbs_sel_i	= wbs_sel_i_reg;
	assign wbs_we_i		= wbs_we_i_reg;
	assign wbs2_stb_i	= wbs2_stb_i_reg;

	initial begin
	#6		// sync to clk

	// write to DMA control reg
//	#100	wbs2_adr_i_reg = 'h2000;	// DMA reg
	#100	wbs2_adr_i_reg = 'h6000;	// Audio DMA reg
			wbs_dat_i_reg = 'hdd44aacc;
			wbs2_stb_i_reg = 1;
			wbs_we_i_reg = 1;

	#20		wbs2_stb_i_reg = 0;
			wbs_dat_i_reg = 'h66666666;
			wbs_we_i_reg = 0;

	// write to RAM - NB simulation shows data is written to RAM one cycle late, so INVALID ('h99999999)
	// however on the actual fpga the wbs_dat_i is held for longer, so not actually a problem
//	#100	wbs2_adr_i_reg = 'h0002;	// Will see this in uart tx later
	#100	wbs2_adr_i_reg = 'h4002;	// Audio RAM
			wbs_dat_i_reg = 'haa559966;
			wbs2_stb_i_reg = 1;
			wbs_we_i_reg = 1;

	#20		wbs2_stb_i_reg = 0;
			wbs_dat_i_reg = 'h99999999;
			wbs_we_i_reg = 0;
	end

	// uart rx stimulus 
	reg r  = 1'b1;
	assign cpu_uart0_txd_o = r;

	initial begin
	// Randomish uart rx signal data (320ns per bit period)
	#5		r = 0;	// start bit
	#320	r = 1;	// toggle data
	#640	r = 0;
	#640	r = 1;
	#640	r = 0;
	#640	r = 1;	// stop bit (with some preceeding data)

	#640	r = 0;	// start bit
	#640	r = 1;	// toggle data
	#960	r = 0;
	#960	r = 1;	// data end and eventually stop bit
	end

`endif

    jtag_top u_jtag_top
    (
		.uart_state		(uart_state),
		.dma_in			(dma_in_reg),
		.rdata_in		(rdata_in),
		.wdata_out		(wdata_out),
		.raddr_out		(raddr_out),
		.waddr_out		(waddr_out),
		.flags_out		(flags_out),
		.jtag_tx_out	(jtag_tx_data),
		.dma_out		(dma_out),
		.tapsigs_out	(tapsigs_unused),
		.wram_enable	(wram_enable)
    );

	assign gpio_o = nano_led;

	// Dual port ram
	reg [31:0] ram[0:2047];			// 8kB currently, initializes to 0 on fpga (not simulation though)
	reg [10:0] ram_rd_addr_reg = 0;
	reg [10:0] ram_wr_addr_reg = 0;
	reg [31:0] ram_wr_data_reg = 0;

	// Shift register adjusts timing of ram write strobe (delayed slightly wrt addr/data ensures properly syncd)
`define WE_LEN 3
	reg [`WE_LEN-1:0] wram_enable_d = `WE_LEN'd0;
	reg jtag_wr_stalled = 0;
	reg wbs2_stb_i_d = 0;

	wire [`DR_LENGTH-1:0] uart_state;
	wire [`DR_LENGTH-1:0] jtag_tx_data;
	wire [`DR_LENGTH-1:0] rx_mem_data;
	wire [`DR_LENGTH-1:0] rdata_in;
	wire [`DR_LENGTH-1:0] wdata_out;
	wire [`DR_LENGTH-1:0] raddr_out;
	wire [`DR_LENGTH-1:0] waddr_out;
	wire [`DR_LENGTH-1:0] flags_out;
	wire [`DR_LENGTH-1:0] dma_out;
	wire [7:0]			  tapsigs_unused;
	wire				  wram_enable;

	reg  [`DR_LENGTH-1:0] dma_in_reg = 0;	// DMA control register. NB read/write are independent registers, so
											// cpu reads ONLY what jtagger wrote and vice versa.
	reg  [`DR_LENGTH-1:0] flags_d = 0;

	wire cpu_ram_sel = !wbs2_adr_i[14] & !wbs2_adr_i[13];
	wire cpu_dma_sel = !wbs2_adr_i[14] & wbs2_adr_i[13];
	wire cpu_rd_enable = cpu_ram_sel & wbs2_stb_i & !wbs_we_i;
	wire cpu_wr_enable = cpu_ram_sel & wbs2_stb_i & wbs_we_i;

	wire cpu_wr_strobe = cpu_wr_enable_d[1] & !cpu_wr_enable_d[0];
	wire jtag_wr_strobe = wram_enable_d[`WE_LEN-2] & !wram_enable_d[`WE_LEN-1];

	reg [1:0] cpu_wr_enable_d = 0;

	// RAM read: cpu_rd_enable (byte selection is ignored) has priority, then jtagger
	// OPTIONAL delay jtagger read in case of collision, but for now handle this via
	// transfer protocol instead
	assign rdata_in = ram[ram_rd_addr_reg];	// See ram_rd_addr_reg below for select logic. NB the address MUST be a
											// register else Quartus will duplicate the ram for each mux input source.

	// assign wbs2_dat_o = cpu_dma_sel ? dma_out : rdata_in;	// OLD, prior to audio
	assign wbs2_dat_o = cpu_dma_sel ? dma_out : cpu_ram_sel ? rdata_in : /* audio_dma_sel ? */ { 21'd0, audio_addr};

	// RAM write: cpu_wr_enable (byte selection is ignored) has priority,  then jtagger, then rx
	// OPTIONAL delay jtagger write in case of collision, but for now handle this via transfer protocol instead

	wire ram_write_strobe = cpu_wr_strobe | jtag_wr_strobe | jtag_wr_stalled | wram_enable_rxd_d;
	wire [10:0] ram_wr_addr = ram_wr_addr_reg;
	wire [31:0] ram_wr_data = ram_wr_data_reg;

	assign wbs2_ack_o = wbs2_stb_i_d;	// Automatic since has priority. NB failure due to flags_urx is silently
										// ignored, OPTIONAL raise wb_err_i in this case.

    always @(posedge clk)
	begin
		// Synchronize
		reset_n_d <= reset_n;	// TODO like neorv32-de0n-ref perhaps (only really needed for rstn_i button
								// KEY0 as flags_run is already sync'd to clk, unsure about pll_locked though)

		// RAM read select logic (see priority note above).
		ram_rd_addr_reg <= cpu_rd_enable ? wbs2_adr_i[12:2]
										: flags_utxb_d	? tx_addr
														: raddr_out[10:0];

		// CHANGED priority setting flags_urx LAST (with no retry).
		// ram_wr_addr_reg <= cpu_wr_enable_d[0] ? wbs2_adr_i[12:2] : waddr_out[10:0];
		// ram_wr_data_reg <= cpu_wr_enable_d[0] ? wbs_dat_i : wdata_out;
		ram_wr_addr_reg <= cpu_wr_enable_d[0] ? wbs2_adr_i[12:2]
											  : wram_enable_d[`WE_LEN-3] ? waddr_out[10:0] : { 1'b0, rx_addr[11:2] };
		ram_wr_data_reg <= cpu_wr_enable_d[0] ? wbs_dat_i : wram_enable_d[`WE_LEN-3] ? wdata_out : rx_mem_data;

		// ram_wr_addr_reg_d <= ram_wr_addr_reg;
		// ram_wr_data_reg_d <= ram_wr_data_reg;

		flags_d <= flags_out;

		// Shift register synchronizes and retimes write strobe
		wram_enable_d <= { wram_enable_d[`WE_LEN-2:0], wram_enable };
		
		// Defer jtag write on cpu priority write
		jtag_wr_stalled <=	cpu_wr_strobe &	(jtag_wr_strobe | jtag_wr_stalled);

		// Write to ram
		if (ram_write_strobe)
		begin
			ram[ram_wr_addr] <= ram_wr_data;
		end

		// Delay the wishbone write (since there is plenty of slack between wishbone operations)
		cpu_wr_enable_d <= { cpu_wr_enable_d[0], cpu_wr_enable };

		wbs2_stb_i_d <= wbs2_stb_i & !wbs2_stb_i_d;	// Acknowledge wishbone strobe delayed by one clock
													// DMA control writes immediately, but memory write is delayed
													// Everything *should* be timed OK (crosses fingers)

		// DMA control register
		if (cpu_dma_sel & wbs2_stb_i & wbs_we_i)
			dma_in_reg <= wbs_dat_i;

    end

	// wire flags_debug = flags_d[10:8] == 1;	// Not currently used
	// wire flags_raddr = flags_d[10:8] == 2;
	// wire flags_waddr = flags_d[10:8] == 3;
	// wire flags_rdata = flags_d[10:8] == 4;
	// wire flags_wdata = flags_d[10:8] == 5;

	// wire flags_tmode = flags_d[16];	// legacy from fpga/audio
	// wire flags_wrap  = flags_d[17];
	// wire flags_audio = flags_d[18];

//	wire flags_run	= flags_d[19];	// Applied to nrst, active low, so fpga starts in reset mode
//	wire flags_urx  = flags_d[20];	// Enable rx buffer (moved, see below)
	wire flags_utx  = flags_d[21];	// Strobe tx
	wire flags_utxm = flags_d[22];	// Multi-byte tx mode (4 bytes per strobe)
	wire flags_utxb = flags_d[23];	// Block tx mode

`ifndef SIM
	wire flags_run	= flags_d[19];	// Applied to nrst, active low, so fpga starts in reset mode
	wire flags_urx	= flags_d[20];	// Enable rx buffer
`else	// (LAZY) Set appropriately for simulation
	wire flags_run	= 1'b1;
	wire flags_urx	= 1'b1;
`endif

	wire [7:0] nano_led;
	assign nano_led = cpu_gpio_o;

	// CHANGED rx_addr from 13 to 12 bits so that it now only occupies the lower half of the memory to avoid
	// the risk of corrupting the top half which is used for "fast" data transfer
	reg [11:0] rx_addr = 0;		// NB LSB 2 bits count chars in ram word
	wire [2:0] txstate;			// Flags returned over jtag
	reg wram_enable_rxd = 0;
	reg wram_enable_rxd_d = 0;
	reg rxd_valid_d = 0;

	wire txd, txd_busy;
	reg [`DR_LENGTH-1:0] txd_data = 0;
	reg txd_start = 1'b0;

	wire rxd, rxd_valid, rxd_idle, rxd_endofpacket;
	wire [7:0] rxd_data;
	reg  [`DR_LENGTH-1:0] rxd_data_reg = 0;

	assign rxd = cpu_uart0_txd_o;
	assign cpu_uart0_rxd_i = txd;

	uart_tx #(.FREQUENCY(CLK_FREQUENCY), .BAUD(BAUDRATE)) u_tx
	(
		.clk	(clk),
		.start	(txd_start),
		.data	(txd_data[7:0]),
		.tx		(txd),
		.busy	(txd_busy)
	);

	uart_rx #(.FREQUENCY(CLK_FREQUENCY), .BAUD(BAUDRATE)) u_rx
	(
		.clk	(clk),
		.rx		(rxd),
		.data	(rxd_data),
		.valid	(rxd_valid)
	);

	reg flags_utx_d = 1'b0;
	reg flags_utxm_d = 1'b0;
	reg flags_utxb_d = 1'b0;
	reg tx_block_run = 1'b0;
	reg [2:0] tx_block_next = 3'd0;	// shift reg for timing
	reg [2:0] tx_count = 3'd0;
	reg [10:0] tx_addr = 10'd0;
	reg [10:0] tx_end = 10'd0;
	wire  [`DR_LENGTH-1:0] tx_data_mux = flags_utxb ? rdata_in : jtag_tx_data;

    always @(posedge clk)
	begin
		rxd_valid_d <= rxd_valid;
		wram_enable_rxd_d <= wram_enable_rxd;

		if (flags_urx && rxd_valid && !rxd_valid_d)
			wram_enable_rxd <= 1;
		else
			wram_enable_rxd <= 0;

		if (!flags_urx)
			rx_addr <= 0;
		else if (wram_enable_rxd_d)
			rx_addr <= rx_addr + 12'd1;

		if (rxd_valid && !rxd_valid_d)
			rxd_data_reg <= { rxd_data, rxd_data_reg[31:8]  };	// FIFO positions data correctly in buffer

		// BEWARE there is no interlocking, so the jtagrisc driver must ensure that only ONE of the flags_utx or
		// flags_utxm/b strobe is applied and wait for txd_busy to clear before applying another (there is a short
		// delay from strobe to txd_busy, but that is swamped by the jtag delay, so there is no race condition)

		flags_utx_d <= flags_utx;
		flags_utxm_d <= flags_utxm;
		flags_utxb_d <= flags_utxb;

		txd_start <= 1'b0;			// Default action is to clear start strobe

		tx_block_next <= { tx_block_next[1:0], 1'b0 };

		if (tx_addr == tx_end)
			tx_block_run <= 1'b0;	// Overriden below when (flags_utxb && !flags_utxb_d)

		if (flags_utxb && !flags_utxb_d)
		begin
			tx_addr <= 11'd0;
			tx_end <= jtag_tx_data[10:0];
			tx_block_run <= 1'b1;
		end
		else if (tx_count == 1 && txd_start)	// Arbitary timing (ie sometime after txd_data was latched)
			tx_addr <= tx_addr + 11'd1;

		if ((flags_utxm && !flags_utxm_d) | tx_block_next[2])
		begin
			// Transmit four chars on flags_utxm
			tx_count <= 3'd4;
		end
		else
		begin
			// Transmit single char on flags_utx
			if (flags_utx && !flags_utx_d)
			begin
				txd_start <= 1'b1;			// Generate 1 cycle strobe on positive edge (delayed wrt jtag_tx_data)
				txd_data <= tx_data_mux;
			end
		end

		if (tx_count != 0)
		begin
			if (!txd_busy && !txd_start)	// txd_start is needed as txd_busy takes an extra cycle to assert
			begin
				txd_start <= 1'b1;
				tx_count <= tx_count - 3'd1;
				if (tx_count == 4)
					txd_data <= tx_data_mux;
				else
					txd_data <= { 8'd0, txd_data[31:8] };
			end
		end
		else if (tx_block_run && tx_addr != tx_end && tx_block_next == 3'd0)
			tx_block_next <= 3'd1;

	end

	wire txd_busym = txd_busy |
					 txd_start |		// txd_start avoids a one-cycle glitch at end of tx_count
					 (flags_utxb & (tx_addr != tx_end)) |
					 (tx_count != 0);

	assign rx_mem_data = rxd_data_reg;
	assign txstate = { flags_run, tx_block_run, txd_busym };	// txd state flags (only txd_busym is needed, but may as well
															// include the others instead of just zeros, can reuse if needed)

	// State is returned over jtag on IUART vir (newest rx char in LSB byte)
	assign uart_state = { txstate[2:0], 1'b0, rx_addr[11:0], rxd_data_reg[23:16] , rxd_data_reg[31:24] };

	// ********* Audio ***********

	// The mp3 frame length is 1152 samples which won't quite fit into a 4kB buffer, so using 8kB 

	reg [31:0] audio_ram[0:2048];			// 8kB 
	reg [10:0] audio_ram_rd_addr_reg = 0;
	reg [10:0] audio_ram_wr_addr_reg = 0;
	reg [10:0] audio_addr = 0;
	reg [31:0] audio_ram_wr_data_reg = 0;
	reg [31:0] audio_dma_reg = 32'd2267;	// 44,100 samples/sec at 100MHz clock

	reg [`DR_LENGTH-1:0] audio_timer = 0;
	wire [`DR_LENGTH-1:0] audio_duration = audio_dma_reg;
	wire [`DR_LENGTH-1:0] adata_in;

	wire audio_ram_sel = wbs2_adr_i[14] & !wbs2_adr_i[13];
	wire audio_dma_sel = wbs2_adr_i[14] & wbs2_adr_i[13];
	wire audio_wr_enable = audio_ram_sel & wbs2_stb_i & wbs_we_i;

	wire audio_wr_strobe = (audio_wr_enable_d[1] & !audio_wr_enable_d[0]) | ~reset_n_d;	// Clear ram on reset
	reg [1:0] audio_wr_enable_d = 0;

	assign adata_in = audio_ram[audio_ram_rd_addr_reg];

    always @(posedge clk)
	begin
		audio_timer <= audio_timer + 1;
		if (audio_timer >= audio_duration)
		begin
			audio_timer <= 0;
			audio_addr <= audio_addr + 11'd1;	// Let it wrap automatically
		end
		else
			audio_timer <= audio_timer + 1;

		// RAM read select logic (see priority note above).
		audio_ram_rd_addr_reg <= audio_addr;

		audio_ram_wr_addr_reg <= reset_n_d ? wbs2_adr_i[12:2] : audio_addr;	// Clear ram on reset
		audio_ram_wr_data_reg <= reset_n_d ? wbs_dat_i : 31'd0;

		// Write to ram
		if (audio_wr_strobe)
		begin
			audio_ram[audio_ram_wr_addr_reg] <= audio_ram_wr_data_reg;
		end

		// Delay the wishbone write (since there is plenty of slack between wishbone operations)
		audio_wr_enable_d <= { audio_wr_enable_d[0], audio_wr_enable };

		// See jtag ram above for Wishbone Acknowledge Strobe

		// DMA control register
		if (audio_dma_sel & wbs2_stb_i & wbs_we_i)
			audio_dma_reg <= wbs_dat_i;
	end

	reg [`DR_LENGTH-1:0] adata_in_d;
    always @(posedge dclk)
		adata_in_d <= adata_in;	// Latch audio data to fix setup time violation in dac

    dac u_dacL
    (
		.clk			(dclk),
		.data_in		(adata_in_d[15:0]),
		.audio_out		(audio_left_)
	);

    dac u_dacR
    (
		.clk			(dclk),
		.data_in		(adata_in_d[31:16]),
		.audio_out		(audio_right_)
	);

endmodule

