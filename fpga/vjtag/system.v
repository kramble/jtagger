/*
	system.v top level for /home/mark/misc/FPGA/quartus/mj_vjtag (based on AMBER CPU)
    see README_MJ	
*/

// Add support for verilator (likely won't work due to sld_virtual_jtag in jtag_tap_altera.v)
// Simulation defines VERILATOR so have it also set SIM (which removes PLL block below)
`ifdef VERILATOR
`define SIM
`endif

module system (
	// Pinout as in Amber system.v based on Zet koktu.v
    // Mostly unused here (just driving the LEDs)
	
    // Clock input
    input        clk_50_,

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

	// nano LEDS
	output [7:0] nano_led_,
	
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

	// NB audio pin is on D9 cf C3 for RomDAChamster so the firmware config (EPCS rom) no longer outputs audio.
	output		audio		// DAC and speech256 audio out
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

	// From jtag_gpios-master/top.v

	// signals (these were input/output for top,v)
    wire        countled;
    wire        button_;

    reg reset_;

    localparam NR_GPIOS = 4;

    wire [NR_GPIOS-1:0] ir_outputs;
    wire [NR_GPIOS-1:0] gpio_outputs;

    jtag_top #( .NR_GPIOS(NR_GPIOS) ) u_jtag_top
    (
        .reset_(reset_),
        .ir_outputs(ir_outputs),
        .gpio_outputs(gpio_outputs)
    );

    always @(posedge clk) begin
        reset_ <= 1'b1;
    end

    reg [31:0] counter;

    always @(posedge clk)
    begin
        if (!button_) begin
            counter <= 0;
        end
        else begin
            counter <= counter + 1'b1;
        end
    end
    
    assign countled = ~counter[24];
	assign button_ = key0_;

	assign nano_led_ = { ir_outputs[3:0],  gpio_outputs[3:1],  gpio_outputs[0] | countled};


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

endmodule
