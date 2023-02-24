`include "defines.v"

module jtag_top (
    input   wire [`DR_LENGTH-1:0] rdata_in,
    input   wire [`DR_LENGTH-1:0] aaddr_in,
    output  wire [`DR_LENGTH-1:0] wdata_out,
    output  wire [`DR_LENGTH-1:0] raddr_out,
    output  wire [`DR_LENGTH-1:0] waddr_out,
    output  wire [`DR_LENGTH-1:0] flags_out,
    output  wire [`DR_LENGTH-1:0] adur_out,
    output  wire [7:0] tapsigs_out,
	output	wire wram_enable
);

    wire [`IR_LENGTH-1:0]  ir;

    wire        capture_dr, shift_dr, exit1_dr, pause_dr, update_dr;
    wire        tck;
    wire        tdi;
    wire        tms; 	// For timing mode (debugging)

    jtag_tap
    u_jtag_tap
    (
        .tck(tck),
        .tdi(tdi),
        .tms(tms),
        .tdo(tdo2tap),
        .ir(ir),
        .capture_dr(capture_dr),
        .shift_dr(shift_dr),
        .exit1_dr(exit1_dr),
        .pause_dr(pause_dr),
        .update_dr(update_dr)
    );

	assign tapsigs_out = { capture_dr, shift_dr, pause_dr, update_dr, tdo2tap, tdi, tms, tck };

    reg bypass_tdo = 0;
    always @(posedge tck)
    begin
        bypass_tdo <= tdi;
    end

    wire tdo2tap;
    wire vdr_tdo;
	wire ident_enable = (ir == `IIDENT);
	wire raddr_enable = (ir == `IRADDR);
	wire waddr_enable = (ir == `IWADDR);
	wire rdata_enable = (ir == `IRDATA);
	wire wdata_enable = (ir == `IWDATA);
	wire flags_enable = (ir == `IFLAGS);
	wire audio_enable = (ir == `IAUDIO);

	// ug_virtualjtag.pdf states BYPASS must be the default for unused IR opcodes
	wire bypass_enable = !(ident_enable | raddr_enable | waddr_enable | rdata_enable | wdata_enable |
								flags_enable | audio_enable);

    assign tdo2tap = bypass_enable ? bypass_tdo : vdr_tdo;

    wire dr_tdo;

    jtag_vdr u_jtag_vdr (
        .tck                (tck),
        .tdi                (tdi),
        .vdr_tdo            (vdr_tdo),

        .capture_dr         (capture_dr),
        .shift_dr           (shift_dr),
        .update_dr          (update_dr),
        // .update_dr          (exit1_dr),			// try using exit1dr for update

		.ident_enable	(ident_enable),
		.raddr_enable	(raddr_enable),
		.waddr_enable	(waddr_enable),
		.rdata_enable	(rdata_enable),
		.wdata_enable	(wdata_enable),
		.flags_enable	(flags_enable),
		.audio_enable	(audio_enable),
		.wram_enable	(wram_enable),

		.aaddr_in		(aaddr_in),
		.rdata_in		(rdata_in),
		.wdata_out		(wdata_out),
		.raddr_out		(raddr_out),
		.waddr_out		(waddr_out),
		.flags_out		(flags_out),
		.adur_out		(adur_out)
    );

endmodule
