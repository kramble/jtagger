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

        input [`DR_LENGTH-1:0]  	rdata_in,
        output reg [`DR_LENGTH-1:0]	wdata_out = 0,
        // output reg [`DR_LENGTH-1:0]	raddr_out = 'h99,	// Have LEDs display something at init so show bitstream has loaded
        output     [`DR_LENGTH-1:0]	raddr_out,				// The above does not seem to work, see DEBUGGING LEDs below
        output reg [`DR_LENGTH-1:0]	waddr_out = 0
    );

	reg [`DR_LENGTH-1:0] raddr_reg = 'h99;	// DEBUGGING LEDs ... this DOES work, it seems Quartus 10.1 ignores the
	assign raddr_out = raddr_reg;			// initial value for port registers

    reg [`DR_LENGTH-1:0] vdr = 0;

	// Shift register adjusts timing of auto increment (TODO experiment with SR length)
	reg [7:0] rdata_enable_d = 0;
	reg [7:0] wdata_enable_d = 0;

	// Flags control auto increment (disabled after address update, enabled after data update/capture)
	reg read_increment = 0;
	reg write_increment = 0;

    assign vdr_tdo = vdr[0];

    always @(posedge tck) 
    begin
        if (shift_dr) 
            vdr       <= { tdi, vdr[`DR_LENGTH-1:1] };

		// Set an identifier so we can recognise this specific device configuration from jtagger
		// Anything will do, but I used a random value (obtained from md5sum of a source file)
        if (capture_dr & ident_enable)
            vdr       <= 32'h97d2f9ce;	// NB must use the same value in jtagger/src/usercode.c

        if (capture_dr & rdata_enable)
		begin
            vdr       <= rdata_in;
			read_increment <= 1;
		end

        if (update_dr && waddr_enable)
		begin
            waddr_out <= vdr[`DR_LENGTH-1:0];
			write_increment <= 0;
		end			
        else if (wdata_enable_d[6] & !wdata_enable_d[7] & write_increment)
			waddr_out <= waddr_out + 1;

        if (update_dr && raddr_enable)
		begin
            // raddr_out <= vdr[`DR_LENGTH-1:0];		// DEBUGGING LEDs
            raddr_reg <= vdr[`DR_LENGTH-1:0];
			read_increment <= 0;
		end			
        else if (rdata_enable_d[6] & !rdata_enable_d[7] & read_increment)
			// raddr_out <= raddr_out + 1;
			raddr_reg <= raddr_reg + 1;					// DEBUGGING LEDs

        if (update_dr && wdata_enable)
		begin
            wdata_out <= vdr[`DR_LENGTH-1:0];
			write_increment <= 1;
		end			

		// Auto increment address. This delays the strobes until after read/write has occurred
		// in system.v NB these are named "data" since they occur on IRDATA and IRDATA instructions,
		rdata_enable_d <= { rdata_enable_d[6:0], capture_dr & rdata_enable };
		wdata_enable_d <= { wdata_enable_d[6:0], update_dr & wdata_enable };
    end


endmodule
