`include "defines.v"

module jtag_top (
    input   wire [`DR_LENGTH-1:0] rdata_in,
    output  wire [`DR_LENGTH-1:0] wdata_out,
    output  wire [`DR_LENGTH-1:0] raddr_out,
    output  wire [`DR_LENGTH-1:0] waddr_out,
	output	wire wram_enable
);

    wire [`IR_LENGTH-1:0]  ir;

    wire        capture_dr, shift_dr, update_dr;
    wire        tck;
    wire        tdi;

    jtag_tap
    u_jtag_tap
    (
        .tck(tck),
        .tdi(tdi),
        .tdo(tdo2tap),
        .ir(ir),
        .capture_dr(capture_dr),
        .shift_dr(shift_dr),
        .update_dr(update_dr)
    );

    reg bypass_tdo = 0;
    always @(posedge tck)
    begin
        bypass_tdo <= tdi;
    end

    wire tdo2tap;
    wire vdr_tdo;
	wire bypass_enable = (ir == `IBYPASS);
	wire ident_enable = (ir == `IIDENT);
	wire raddr_enable = (ir == `IRADDR);
	wire waddr_enable = (ir == `IWADDR);
	wire rdata_enable = (ir == `IRDATA);
	wire wdata_enable = (ir == `IWDATA);

	assign wram_enable = wdata_enable & update_dr;

    assign tdo2tap = bypass_enable ? bypass_tdo : vdr_tdo;

    wire dr_tdo;

    jtag_vdr u_jtag_vdr (
        .tck                (tck),
        .tdi                (tdi),
        .vdr_tdo            (vdr_tdo),

        .capture_dr         (capture_dr),
        .shift_dr           (shift_dr),
        .update_dr          (update_dr),

		.ident_enable	(ident_enable),
		.raddr_enable	(raddr_enable),
		.waddr_enable	(waddr_enable),
		.rdata_enable	(rdata_enable),
		.wdata_enable	(wdata_enable),

		.rdata_in		(rdata_in),
		.wdata_out		(wdata_out),
		.raddr_out		(raddr_out),
		.waddr_out		(waddr_out)
    );

endmodule
