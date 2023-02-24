`include "defines.v"

module jtag_vdr
    (
        input   wire        tck,
        input   wire        tdi,

        output              vdr_tdo,

        // TAP states
        input               capture_dr,
        input               shift_dr,
        input               update_dr,

		input				ident_enable,
		input				raddr_enable,
		input				waddr_enable,
		input				rdata_enable,
		input				wdata_enable,
		input				flags_enable,
		input				audio_enable,
		output reg			wram_enable,

        input [`DR_LENGTH-1:0]  	rdata_in,
        input [`DR_LENGTH-1:0]  	aaddr_in,
        output reg [`DR_LENGTH-1:0]	wdata_out = 0,	// BEWARE quartus 10.1 ignores port initialization values
        output     [`DR_LENGTH-1:0]	raddr_out,
        output reg [`DR_LENGTH-1:0]	waddr_out = 0,
        output     [`DR_LENGTH-1:0]	flags_out,
        output     [`DR_LENGTH-1:0]	adur_out
    );

	reg [`DR_LENGTH-1:0] raddr_reg = 0;		// It seems Quartus 10.1 ignores the initial value for port registers
	assign raddr_out = raddr_reg;

	reg [`DR_LENGTH-1:0] flags_reg = 'h99;	// Initial LED value
	assign flags_out = flags_reg;

`ifndef SIM
	reg [`DR_LENGTH-1:0] adur_reg = 1134;	// 44,100 samples/sec at 50MHz clock
`else
	reg [`DR_LENGTH-1:0] adur_reg = 64;		// Shorten for simlation
`endif
	assign adur_out = adur_reg;

    assign vdr_tdo = vdr[0];

    reg [`DR_LENGTH-1:0] vdr = 0;

	// Shift register adjusts timing of auto increment and ram write strobe
	reg [7:0] rdata_enable_d = 0;
	reg [7:0] wdata_enable_d = 0;

	// Flags control auto increment (disabled after address update, enabled after data update/capture)
	// May not actually be needed since increment is timed from capture/update (TODO try removing)
	reg read_increment = 0;
	reg write_increment = 0;

	// Alternative write strategy based on count of dr_strobe (allows continuous shifting for speed)
	reg [4:0] wr_count = 0;					// NB wraps at 31 by design
	reg latch_wdata_reg = 0;
	// wire latch_wdata = update_dr;		// normal
	wire latch_wdata = latch_wdata_reg;		// fast count

    always @(posedge tck) 
    begin
        if (shift_dr) 
            vdr       <= { tdi, vdr[`DR_LENGTH-1:1] };

		// Set an identifier so we can recognise this specific device configuration from jtagger
		// Anything will do, but I used a random value (obtained from md5sum of a source file)
        if (capture_dr & ident_enable)
		//	vdr       <= 32'h97d2f9ce;	// NB must use the same value in jtagger/src/usercode.c
		//	vdr       <= 32'h97d2f9cf;	// Changed flags behavoir so change id
			vdr       <= 32'h97d2f9d0;	// fpga/audio

        if (capture_dr & rdata_enable)
		begin
            vdr       <= rdata_in;
			read_increment <= 1;
		end

        if (update_dr && flags_enable)
            flags_reg <= vdr[`DR_LENGTH-1:0];

		// NB opcode IAUDIO writes sample duration BUT it captures audio address
        if (capture_dr & audio_enable)
            vdr       <= aaddr_in;

        if (update_dr && audio_enable)
            adur_reg <= vdr[`DR_LENGTH-1:0];

        if (update_dr && waddr_enable)
		begin
            waddr_out <= vdr[`DR_LENGTH-1:0];
			write_increment <= 0;
		end			
        else if (wdata_enable_d[6] & !wdata_enable_d[7] & write_increment)
			waddr_out <= waddr_out + 1;

        if (update_dr && raddr_enable)
		begin
            raddr_reg <= vdr[`DR_LENGTH-1:0];
			read_increment <= 0;
		end			
        else if (rdata_enable_d[6] & !rdata_enable_d[7] & read_increment)
			raddr_reg <= raddr_reg + 1;

        if (latch_wdata && wdata_enable)
		begin
            wdata_out <= vdr[`DR_LENGTH-1:0];
			write_increment <= 1;
		end			

		// Ram write strobe is timed off tck to ensure correct relationship with address/data
		wram_enable <= wdata_enable_d[4] & !wdata_enable_d[5];

		// Auto increment address. This delays the strobes until after read/write has occurred
		// in system.v NB these are named "data" since they occur on IRDATA and IRDATA instructions,
		rdata_enable_d <= { rdata_enable_d[6:0], capture_dr & rdata_enable };
		wdata_enable_d <= { wdata_enable_d[6:0], latch_wdata & wdata_enable };

		// if ((latch_wdata && shift_dr && wdata_enable) || !wdata_enable)	// !wdata_enable is to resync (in case sync is lost)
		if (!wdata_enable ||	// resync when not IWDATA (NB in normal shift operation counter wraps to 0 by design)
				capture_dr )	// and this (in case there were additional shifts leaving it misaligned)
			wr_count <= 0;
		else if (shift_dr && wdata_enable)
			wr_count <= wr_count + 1;

		latch_wdata_reg <= (wr_count == 31);
    end


endmodule
