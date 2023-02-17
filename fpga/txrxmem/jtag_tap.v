`include "defines.v"

module jtag_tap (
        output wire                 tck,
        output wire                 tdi,
        input  wire                 tdo,

        output wire [`IR_LENGTH-1:0]   ir,

        output wire                 capture_dr,
        output wire                 shift_dr,
        output wire                 update_dr
    );

`ifndef SIM
    sld_virtual_jtag u_vjtag (
        .tck                (tck),
        .tdi                (tdi),
        .tdo                (tdo),

        .ir_out             (),
        .ir_in              (ir),

        .virtual_state_cdr  (capture_dr),
        .virtual_state_cir  ( ),
        .virtual_state_e1dr ( ),
        .virtual_state_e2dr ( ),
        .virtual_state_pdr  ( ),
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
	reg rcapture_dr = 0;
	reg rshift_dr = 0;
	reg rupdate_dr = 0;
	reg [`IR_LENGTH-1:0] rir = 0;

	assign tck = rtck;
	assign tdi = rtdi;
	assign capture_dr = rcapture_dr;
	assign shift_dr = rshift_dr;
	assign update_dr = rupdate_dr;
	assign ir = rir;

	// NB simulating with tck same as clk_50_ but in actual device tck is not
	//    synchronous with clk_50_ (clock crossing is handled in system.v)
	always #5 rtck = !rtck;

	initial begin
	// Approximate timings, TODO use more accurate sequences (probably will only
	// fix this if the decvice does not behave as expected on jtagger testing)

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

	#  110 rir = `IWDATA;
	#   10 rcapture_dr = 1;
	#   10 rcapture_dr = 0;
	#   10 rshift_dr = 1;
	#   80 rtdi = 1;
	#   80 rtdi = 0;
	#   20 rtdi = 1;
	#   20 rtdi = 0;
	#  250 rshift_dr = 0;
	#    0 rupdate_dr = 1;
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


  end

`endif

endmodule
