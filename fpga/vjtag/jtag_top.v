`define JTAG_TAP_ALTERA
// NB JTAG_TAP_GENERIC is NOT defined, so that section is omitted

`include "jtag_tap_defines.v"

`default_nettype none

module jtag_top #(
        parameter NR_GPIOS = 4
    ) 
    (
    input   wire        reset_,

`ifdef JTAG_TAP_GENERIC
    input   wire        trst_,
    input   wire        tck,
    input   wire        tms,
    input   wire        tdi,
    output  wire        tdo,
`endif

    output  wire [NR_GPIOS-1:0] ir_outputs,
    output  wire [NR_GPIOS-1:0] gpio_outputs
);

    wire [`IR_LENGTH-1:0]  ir;

    wire        capture_dr, shift_dr, update_dr;

`ifdef JTAG_TAP_ALTERA
    wire        tck;
    wire        tdi;

    jtag_tap_altera #(
        .IR_BITS(`IR_LENGTH)
    ) 
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

`endif

`ifdef JTAG_TAP_GENERIC
    wire gpio_data_ir, gpio_config_ir;

    jtag_tap_generic u_jtag_tap
    (
        .trst_pad_i(!trst_),
        .tck_pad_i(tck),
        .tms_pad_i(tms),
        .tdi_pad_i(tdi),
        .tdo_pad_o(tdo),

        .ir_o(ir),

        .tdo_i(tdo2tap),

        .capture_dr_o(capture_dr),
        .shift_dr_o(shift_dr),
        .update_dr_o(update_dr)
    );
`endif

    reg bypass_tdo;
    always @(posedge tck)
    begin
        bypass_tdo <= tdi;
    end

    wire scan_n_ir, extest_ir;
    assign scan_n_ir = (ir == `SCAN_N);
    assign extest_ir = (ir == `EXTEST);
	assign ir_outputs = ir;

    wire tdo2tap;
    assign tdo2tap = (scan_n_ir | extest_ir) ? gpios_tdo 
                                             : bypass_tdo;

    wire gpios_tdo;

    jtag_gpios #(
        .NR_GPIOS(NR_GPIOS)
    )
    u_jtag_gpios (
        .reset_             (reset_),
        .tck                (tck),
        .tdi                (tdi),
        .gpios_tdo          (gpios_tdo),

        .capture_dr         (capture_dr),
        .shift_dr           (shift_dr),
        .update_dr          (update_dr),

        .scan_n_ir          (scan_n_ir),
        .extest_ir          (extest_ir),

        .gpio_outputs       (gpio_outputs)
    );


endmodule
