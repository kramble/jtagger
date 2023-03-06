// tb.v - iverilog testbench

module tb;

de0_nano u (

	.clk_i			(clk),
	.rstn_i			(rstn),
	.gpio_o			(led),
	.uart0_rxd_i	(uart0_rxd_i),
	.uart0_txd_o	(uart0_txd_o)


);

	wire [7:0] led;

  // NB Stimulus is applied in jtag_tap.v
  initial begin
	$dumpfile("zsystem.vcd");
    $dumpvars(0, tb.u);

	# 250000 $finish;
  end

  reg clk = 0;
  reg rstn = 1;
  reg uart0_rxd_i = 0;

  always #5 clk = !clk;

/*
initial
     $monitor("%5t, clk=%d led=%b ir=%h sh=%b ud=%b rae=%b\nvdr=%b rd_addr=%b\n", $time, clk, led,
			u.u_jtag_top.ir, u.u_jtag_top.shift_dr, u.u_jtag_top.update_dr, u.u_jtag_top.raddr_enable,
			u.u_jtag_top.u_jtag_dr.vdr, u.ram_rd_addr);
*/

endmodule
