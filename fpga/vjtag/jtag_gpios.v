
`default_nettype none

module jtag_gpios
    #(
        parameter NR_GPIOS = 4
    )
    (
        // Non-jtag reset
        input               reset_,

        // In the case come either straight from the IO pins or from the
        // virtual jtag TAP.
        input   wire        tck,
        input   wire        tdi,

        // Output of the GPIO status registers.
        // The real or the virtual JTAG TAP will select this when the GPIO
        // scan chain is selected by the TAP.
        output              gpios_tdo,

        // TAP states
        input               capture_dr,
        input               shift_dr,
        input               update_dr,

        // Current active instruction
        input               scan_n_ir,
        input               extest_ir,

        output reg [NR_GPIOS-1:0]   gpio_outputs
    );

    // There is only 1 shift register because we use capture_dr for both
    // IRs. So since the shift register updated before the scan operation
    // anyway, the previous value of the shift register doesn't matter.
    //
    // The MSB of gpio_dr is used only during the update_dr phase and determines
    // whether or not the data that has been scanned in is actually used
    // to update the target register.
    // By setting it to 0, read-only operations are possible.
    reg [NR_GPIOS:0]  gpio_dr;

    // Currently selected register: 0 -> config, 1 -> data
    reg scan_n;

    always @(posedge tck) 
    begin
        if (scan_n_ir) begin
            if (shift_dr) begin
                scan_n  <= tdi;
            end
        end

        // DATA
        if (extest_ir) begin
			// MJ ??? quartus warns case statement has overlapping (blah) dont care bits (blah)
/*
            case(1'b1) // synthesis parallel_case
                capture_dr: begin
                    gpio_dr         <= gpio_inputs;
                end
                shift_dr: begin
                    gpio_dr         <= { tdi, gpio_dr[NR_GPIOS:1] };
                end
                update_dr: begin
                    if (gpio_dr[NR_GPIOS]) begin
                        gpio_outputs    <= gpio_dr[NR_GPIOS-1:0];
                    end
                end
            endcase
*/
                if (capture_dr) begin
                    gpio_dr         <= 4'd9;	// arbitary
				end
                if (shift_dr) begin
                    gpio_dr         <= { tdi, gpio_dr[NR_GPIOS:1] };
				end
                if (update_dr)
				begin
                    if (gpio_dr[NR_GPIOS]) begin
                        gpio_outputs    <= gpio_dr[NR_GPIOS-1:0];
                    end
                end
        end


    end

    assign gpios_tdo = scan_n_ir ? scan_n
                                 : gpio_dr[0];

endmodule
