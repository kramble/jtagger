/*
	system.v top level for jtagger/fpga/txrxmem

	Virtual jtag read/write of block RAM (64kB), see jtagger/src/usercode.c for driver
*/

`include "defines.v"

module system (
	// Pinout as in Amber system.v based on Zet koktu.v
    // Mostly unused here (just driving the LEDs)
	
    // Clock input
    input        clk_50_,

`ifndef SIM		// See defines.v
    // General purpose IO
    input         key0_,			// active low
    input         key1_,			// active low
	input  [ 3:0] nano_sw_,			// dip switches, ON=logic_0, OFF=logic_1 (the DE0-Nano manual is WRONG!)
	
    // sdram signals
    output [11:0] sdram_addr_,		// NB [12] is driven to GND in the qsf file
    inout  [15:0] sdram_data_,
    output [ 1:0] sdram_ba_,
    output        sdram_ras_n_,
    output        sdram_cas_n_,
    output        sdram_ce_,
    output        sdram_clk_,
    output        sdram_we_n_,
    output        sdram_cs_n_,

    // nano VGA signals
    output [3:0] nano_vga_r_,			// open drain
    output [3:0] nano_vga_g_,			// open drain
    output [3:0] nano_vga_b_,			// open drain
    output nano_vga_hsync_,
    output nano_vga_vsync_,

    // UART signals
    // output        uart_txd_,			// NB Unused (Zet UART), see below for my TxD, RxD, bias

    // PS2 signals
    input         ps2_kclk_,	// PS2 keyboard Clock (spec says bidirectional, but implemented as input only)
    input         ps2_kdat_,	// PS2 Keyboard Data (changed to input from inout while debugging, could change back but no need)
    inout         ps2_mclk_,	// PS2 Mouse Clock
    inout         ps2_mdat_,	// PS2 Mouse Data

    // SD card signals
    output        sd_sclk_,
    input         sd_miso_,
    output        sd_mosi_,
    output        sd_ss_,

    // I2C for audio codec
    // inout         i2c_sdat_,
    // output        i2c_sclk_,

    // Audio codec signals
    // input         aud_daclrck_,
    // output        aud_dacdat_,
    // input         aud_bclk_,
    // output        aud_xck_

	// Added for AMBER (not yet implemented)
	input		RxD,
	output		TxD,
	output		bias,
	output		bias2,

	// NB audio pin is on D9 cf C3 for RomDAChamster
	output		audio,		// DAC and speech256 audio out
`endif 	// SIM

	// nano LEDS
	output [7:0] nano_led_

  );
 
parameter CLOCK_MULTIPLIER = 1;
// parameter CLOCK_DIVIDER = 2;		// 25MHz
parameter CLOCK_DIVIDER = 1;		// 50MHz

wire clk;

`ifndef SIM
	main_pll #(CLOCK_MULTIPLIER, CLOCK_DIVIDER) pll_blk (clk_50_, clk);
`else
	assign clk = clk_50_;
`endif

    jtag_top u_jtag_top
    (
		.rdata_in		(rdata_in),
		.wdata_out		(wdata_out),
		.raddr_out		(raddr_out),
		.waddr_out		(waddr_out),
		.flags_out		(flags),
		.tapsigs_out	(tapsigs),
		.wram_enable	(wram_enable)
    );

	// Dual port ram
	// TODO ??? ... quartus reports dual clock. Retimed the addr inputs ... still dual clock, but no warning this time,
	//		it just has "Warning: Inferred RAM node "ram~0" from synchronous design logic. Pass-through logic has been added
	//		to match the read-during-write behavior of the original design."
	// TODO ??? ... quartus reports total memory bits : 524,301 / 608,256 ( 86 % ), however 64kB is 524,288 (possibly OK
	//		may be using ram for shift registers, yep all 66 M9Ks in use, see system.fitter.rpt, two of them are tiny)

	reg [31:0] ram[0:16383];	// 64kB
	reg [13:0] ram_rd_addr_reg = 0;
	reg [13:0] ram_wr_addr_reg = 0;
	reg [31:0] ram_wr_data_reg = 0;
	reg  [7:0] debug = 0;

	// Shift register adjusts timing of ram write strobe (delayed slightly wrt addr/data ensures properly syncd)
`define WE_LEN 3
	reg [`WE_LEN-1:0] wram_enable_d = 0;

// Not wanted for timing mode simulation
//`ifndef SIM
	assign rdata_in = ram[ram_rd_addr_reg];
//`else
//	assign rdata_in = ~ram[ram_rd_addr_reg];	// DEBUG use inverse to observe capture into vdr
//`endif

	wire [`DR_LENGTH-1:0] rdata_in;
	wire [`DR_LENGTH-1:0] wdata_out;
	wire [`DR_LENGTH-1:0] raddr_out;
	wire [`DR_LENGTH-1:0] waddr_out;
	wire [`DR_LENGTH-1:0] flags;
	reg  [`DR_LENGTH-1:0] flags_d = 0;

	// This could get a bit messy when flags mode changes, but we'll cope with that in usercode.c
	wire ram_write_strobe = flags_tmode ?	wram_enable_t
										:	wram_enable_d[`WE_LEN-2] & !wram_enable_d[`WE_LEN-1];

	wire [13:0] ram_wr_addr = flags_tmode	?	ram_wr_addr_t
											:	ram_wr_addr_reg;

	wire [31:0] ram_wr_data = flags_tmode	?	{ timer[23:0] , prev_tapsigs /* tapsigs_d */ }
											: ram_wr_data_reg;

	wire wram_enable;

    always @(posedge clk)
	begin
		// Synchronize
		ram_rd_addr_reg <= raddr_out[13:0];
		ram_wr_addr_reg <= waddr_out[13:0];
		ram_wr_data_reg <= wdata_out;
		flags_d <= flags;

		// Shift register synchronizes and retimes write strobe
		wram_enable_d <= { wram_enable_d[`WE_LEN-2:0], wram_enable };

		// Write to ram
		if (ram_write_strobe)
		begin
			ram[ram_wr_addr] <= ram_wr_data;
			debug <= { ram_wr_addr[3:0], ram_wr_data[3:0] };
		end
    end

	wire flags_debug = flags_d[10:8] == 1;
	wire flags_raddr = flags_d[10:8] == 2;
	wire flags_waddr = flags_d[10:8] == 3;
	wire flags_rdata = flags_d[10:8] == 4;
	wire flags_wdata = flags_d[10:8] == 5;
	wire flags_tmode = flags_d[16];

	assign nano_led_ =	flags_debug ? debug :	
						flags_raddr ? raddr_out[7:0] :
						flags_waddr ? waddr_out[7:0] :
						flags_rdata ? wdata_out[7:0] :
						flags_wdata ? rdata_in[7:0] :
						flags[7:0];

	// Timing test. Log JTAG signals to RAM
	reg [`DR_LENGTH-1:0] timer = 0;

	// Allocate a whole byte for tapsigs so it aligns nicely
	wire [7:0] tapsigs;
	reg [7:0] tapsigs_d = 0;
	reg [7:0] prev_tapsigs = 0;
	reg [13:0] ram_wr_addr_t = 0;
	reg addr_increment = 0;
	reg wram_enable_t = 0;

    always @(posedge clk)
	begin
		timer <= timer + 1;
		tapsigs_d <= tapsigs;

		if (flags_tmode && prev_tapsigs != tapsigs_d)
		begin
			wram_enable_t <= 1;
			prev_tapsigs <= tapsigs_d;
			addr_increment <= 1;
		end
		else
		begin
			wram_enable_t <= 0;
			addr_increment <= 0;
		end

		if (!flags_tmode)
			ram_wr_addr_t <= 0;
		else if (addr_increment)
			ram_wr_addr_t <= ram_wr_addr_t + 1;
	end

`ifndef SIM

// Tie unused outputs low EXCEPT ram control inputs (active low)

assign sdram_addr_ = 12'd0;
// assign sdram_data_ = 16'd0;		// Tristate so do not drive
assign sdram_ba_ = 2'd0;

assign sdram_ras_n_ = 1'd1;			// These are active low so tie high
assign sdram_cas_n_ = 1'd1;
assign sdram_ce_ = 1'd1;
assign sdram_clk_ = 1'd1;
assign sdram_we_n_ = 1'd1;
assign sdram_cs_n_ = 1'd1;

assign nano_vga_r_ = 4'd0;			// open drain
assign nano_vga_g_ = 4'd0;			// open drain
assign nano_vga_b_ = 4'd0;			// open drain
assign nano_vga_hsync_ = 1'd0;
assign nano_vga_vsync_ = 1'd0;

assign sd_sclk_ = 1'd0;
assign sd_mosi_ = 1'd0;
assign sd_ss_ = 1'd0;

assign bias = 1'd1;					// Tie serial I/O bias high
assign bias2 = 1'd1;

assign TxD = 1'd0;
assign audio = 1'd0;

`endif	// SIM

endmodule
