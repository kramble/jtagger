/*
	system.v top level for jtagger/fpga/audio

	Virtual jtag audio streamer see jtagger/fpga/audio/usercode.c for driver

    From README...

fpga/audio - enough of the boring test stuff, let's have a bit of fun.

This demo will stream an audio file over jtag to the DE0-Nano.

Currently just supports .wav files, mp3 would be possible by integrating
code from eg libmad https://www.underbit.com/products/mad/ however that's
a lot of code that's not really relevant to jtag, so just .wav for now.
Anyway it'll be more fun to get the DE0-Nano itself to decode mp3 (laters!)

You can convert mp3 to wav files using ffmpeg, eg
ffmpeg -i file.mp3 file.wav

Or if you just want a quick test, you'll probably find some lying around in
your OS, eg /usr/share/sounds or C:\Windows\Media

Beware only mono or stereo S16LE wav files are supported (the most common
format), but some of the Windows files are not, so just try another one.

You'll need to connect some old-style headphones or earphones to the DE0-Nano.
I do NOT recommend connecting to any electronic system (eg hifi, powered
speakers etc.) due to the high level of RF noise produced by the delta-sigma
(one-bit) DAC, even with filtering, as it may damage your system.

Output is on pins 1 and 2 of the JP1 GPIO connector, ASCII ART follows...

             L R      GND
             | |       |
        jp1  x x o o o x o o o...
[*] [*]      o o o o o o o o o...
 ^   ^
blue+green          [ row of green leds ]
power/usb led

I recommend adding some filtering components, since I'm driving low impedance
headphones I used the following circuit, but you should customise as appropriate.

DE0-Nano L   ---/\/\/\/---+---- Headphone Left Channel
                100 ohm   |
                        ----- 470 nF (or anything vaguely similar)
                        -----
                          |
DE0-Nano GND -------------+---- Headphone Ground

And the same for the right channel. This gives an RC const of 100 * 470n = 47uS
or 20kHz, so perhaps a slightly higher value resistor/capacitor would be better?
Ideally use a sharper rolloff active filter which I'm not going to delve into
here. Anyway the load impedance (8 ohm?) is going to wildly skew the filter
curve, and this is getting way beyond my limited electronics knowledge so I'm
going to stop right there. All liability for damage etc is DISCLAIMED! Do this
at your own risk.

Building/running the demo...

cd fpga/audio
make
make cleanobj (to remove the .o file clutter)
./jtagaudio -y -r system.rbf
./jtagaudio -u file.wav			(old style demo, new version will have it's own command line parser)

Have fun!
*/

`include "defines.v"

module system (
	// Pinout as in Amber system.v based on Zet koktu.v
    // Mostly unused here (just driving the LEDs + audio)
	
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
`endif 	// SIM

	// nano LEDS
	output [7:0] nano_led_,
	output		audio_left_,
	output		audio_right_

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
		.aaddr_in		(audio_addr),
		.rdata_in		(rdata_in),
		.wdata_out		(wdata_out),
		.raddr_out		(raddr_out),
		.waddr_out		(waddr_out),
		.flags_out		(flags),
		.adur_out		(audio_duration),
		.tapsigs_out	(tapsigs),
		.wram_enable	(wram_enable)
    );

    dac u_dacL
    (
		.clk			(clk),
		.data_in		(adata_in[15:0]),
		.audio_out		(audio_left_)
	);

    dac u_dacR
    (
		.clk			(clk),
		.data_in		(adata_in[31:16]),
		.audio_out		(audio_right_)
	);

	// Dual port ram
	reg [31:0] ram[0:16383];	// 64kB
	reg [13:0] ram_rd_addr_reg = 0;
	reg [13:0] ram_wr_addr_reg = 0;
	reg [31:0] ram_wr_data_reg = 0;
	reg  [7:0] debug = 0;

	// Shift register adjusts timing of ram write strobe (delayed slightly wrt addr/data ensures properly syncd)
`define WE_LEN 3
	reg [`WE_LEN-1:0] wram_enable_d = 0;

	assign rdata_in = ram[ram_rd_addr_reg];

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
		// NB flags_audio disables jrag ram read, TODO multiplex them (just delay audio access on jtag read)
		ram_rd_addr_reg <= flags_audio ? audio_addr[13:0] : raddr_out[13:0];
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
	wire flags_wrap  = flags_d[17];
	wire flags_audio = flags_d[18];

	assign nano_led_ =	flags_audio ? vu_leds :
						flags_debug ? debug :	
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
			if (!flags_wrap || ram_wr_addr_t != 'h3fff)	// Do not wrap
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

	reg [`DR_LENGTH-1:0] audio_timer = 0;
	reg [`DR_LENGTH-1:0] audio_addr = 0;
	wire [`DR_LENGTH-1:0] audio_duration;
	wire [`DR_LENGTH-1:0] adata_in;
	assign adata_in = flags_audio ? rdata_in : 31'h0;

	wire [15:0] mono = adata_in[31:16] + adata_in[15:0];	// S16LE audio (signed)
	wire [15:0] vu = mono[15] ? -mono : mono;				// Magnitude

	// TODO there must be a better way than this...
	wire [7:0] vu_leds = {
    vu[15] | vu[14] | vu[13],
    vu[15] | vu[14] | vu[13] | vu[12],
    vu[15] | vu[14] | vu[13] | vu[12] | vu[11],
    vu[15] | vu[14] | vu[13] | vu[12] | vu[11] | vu[10],
    vu[15] | vu[14] | vu[13] | vu[12] | vu[11] | vu[10] | vu[9],
    vu[15] | vu[14] | vu[13] | vu[12] | vu[11] | vu[10] | vu[9] | vu[8],
    vu[15] | vu[14] | vu[13] | vu[12] | vu[11] | vu[10] | vu[9] | vu[8] | vu[7],
    vu[15] | vu[14] | vu[13] | vu[12] | vu[11] | vu[10] | vu[9] | vu[8] | vu[7] | vu[6]
//	flags_audio	// (instead of last line above) DEBUG to check it's running
	};

    always @(posedge clk)
	begin
		audio_timer <= audio_timer + 1;
		if (audio_timer >= audio_duration)
		begin
			audio_timer <= 0;
			if (flags_audio)
				audio_addr <= audio_addr + 1;	// Let it wrap automatically
			else
				audio_addr <= 0;
		end
		else
			audio_timer <= audio_timer + 1;
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
`endif	// SIM

endmodule
