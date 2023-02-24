// tb.v - iverilog testbench

module tb;

system u (

	.clk_50_ (clk),
	.nano_led_ (led)

);

	wire [7:0] led;

  // NB Stimulus is applied in jtag_tap.v
  initial begin
	$dumpfile("zsystem.vcd");
    $dumpvars(0, tb.u);

	# 5000 $finish;
  end

  reg clk = 0;
  always #5 clk = !clk;

/*
initial
     $monitor("%5t, clk=%d led=%b ir=%h sh=%b ud=%b rae=%b\nvdr=%b rd_addr=%b\n", $time, clk, led,
			u.u_jtag_top.ir, u.u_jtag_top.shift_dr, u.u_jtag_top.update_dr, u.u_jtag_top.raddr_enable,
			u.u_jtag_top.u_jtag_dr.vdr, u.ram_rd_addr);
*/

endmodule
