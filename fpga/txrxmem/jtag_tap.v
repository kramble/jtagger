`include "defines.v"

module jtag_tap (
        output wire                 tck,
        output wire                 tdi,
        output wire                 tms,
        input  wire                 tdo,

        output wire [`IR_LENGTH-1:0]   ir,

        output wire                 capture_dr,
        output wire                 shift_dr,
        output wire                 exit1_dr,
        output wire                 update_dr,
        output wire                 pause_dr
    );

`ifndef SIM
    sld_virtual_jtag u_vjtag (
        .tck                (tck),
        .tdi                (tdi),
        .tdo                (tdo),
        .tms                (tms),	// For timing mode (debugging)

        .ir_out             (),
        .ir_in              (ir),

        .virtual_state_cdr  (capture_dr),
        .virtual_state_cir  ( ),
        .virtual_state_e1dr (exit1_dr),
        .virtual_state_e2dr ( ),
        .virtual_state_pdr  (pause_dr),
        .virtual_state_sdr  (shift_dr),
        .virtual_state_udr  (update_dr),
        .virtual_state_uir  ( )
        );

    defparam
        u_vjtag.sld_auto_instance_index = "YES",
        u_vjtag.sld_instance_index = 0,
        u_vjtag.sld_ir_width = `IR_LENGTH;
`else

	reg rtck = 0;
	reg rtdi = 0;
	reg rtms = 0;
	reg rcapture_dr = 0;
	reg rshift_dr = 0;
	reg rupdate_dr = 0;
	reg rexit1_dr = 0;
	reg rpause_dr = 0;
	reg [`IR_LENGTH-1:0] rir = 0;

	assign tck = rtck;
	assign tdi = rtdi;
	assign tms = rtms;
	assign capture_dr = rcapture_dr;
	assign shift_dr = rshift_dr;
	assign update_dr = rupdate_dr;
	assign exit1_dr = rexit1_dr;
	assign pause_dr = rpause_dr;
	assign ir = rir;

	// NB simulating with tck same as clk_50_ but in actual device tck is not
	//    synchronous with clk_50_ (clock crossing is handled in system.v)
	always #5 rtck = !rtck;

	initial begin
	// Approximate timings, TODO use more accurate sequences (probably will only
	// fix this if the decvice does not behave as expected on jtagger testing)
	#	 6	// Sync signals to (just after) tck rising edge
	#   10 rir = `IIDENT;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
	#   10 rshift_dr = 1;
	#  320 rshift_dr = 0;
	#    0 rupdate_dr = 1;
	#   10 rupdate_dr = 0;

	#   10 rir = `IRADDR;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
	#   10 rshift_dr = 1;
	#   50 rtdi = 1;
	#   50 rtdi = 0;
	#   50 rtdi = 1;
	#  250 rshift_dr = 0;
	#    0 rupdate_dr = 1;
	#   10 rupdate_dr = 0;

	#   10 rir = `IWADDR;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
	#   10 rshift_dr = 1;
	#   50 rtdi = 1;
	#   50 rtdi = 0;
	#   50 rtdi = 1;
	#  250 rshift_dr = 0;
	#    0 rupdate_dr = 1;
	#   10 rupdate_dr = 0;

	// This needs to be a bit more accurate since we are counting shifts
	#  110 rir = `IWDATA;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
	#   10 rshift_dr = 1;
	#   80 rtdi = 1;
	#   80 rtdi = 0;
	#   20 rtdi = 1;
	#   20 rtdi = 0;
	#  120 rshift_dr = 0;
	#   30 rupdate_dr = 1;
	#   10 rupdate_dr = 0;

	#  110 rir = `IRDATA;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
	#   10 rshift_dr = 1;
	#   50 rtdi = 1;
	#   50 rtdi = 0;
	#   50 rtdi = 1;
	#  250 rshift_dr = 0;
	#    0 rupdate_dr = 1;
	#   10 rupdate_dr = 0;

	// Set read address to 0 so we can see timing write below
	#   10 rir = `IRADDR;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
		   rtdi = 0;
	#   10 rshift_dr = 1;
	#  320 rshift_dr = 0;
	#    0 rupdate_dr = 1;
	#   10 rupdate_dr = 0;

	// Set timing mode
	#  110 rir = `IFLAGS;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
		   rtdi = 0;
	#   10 rshift_dr = 1;
	#  160 rtdi = 1;
	#   10 rtdi = 0;
	#  150 rshift_dr = 0;
	#   30 rupdate_dr = 1;
	#   10 rupdate_dr = 0;

	// Log this
	// NB for simulation tapsigs_d (and hence log) does not show tck toggling since sampled on
	//    rising clk edge. TODO de-sync tck and clk to fix this.
	#  110 rir = `IWADDR;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
	#   10 rshift_dr = 1;
	#   80 rtdi = 1;
	#   80 rtdi = 0;
	#   20 rtdi = 1;
	#   20 rtdi = 0;
	#  120 rshift_dr = 0;
	#   30 rupdate_dr = 1;
	#   10 rupdate_dr = 0;



  end

`endif

endmodule
