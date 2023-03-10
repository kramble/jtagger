/* neorv32_test_setup_verilog_top.v

This is a wrapper that calls VHDL neorv32_test_setup_bootloader from neorv32_test_setup_bootloader.vhd

NB changed de0-nano-test-setup.qsf to add 
	set_global_assignment -name VERILOG_FILE neorv32_test_setup_verilog_top.v

and change
	set_global_assignment -name TOP_LEVEL_ENTITY neorv32_test_setup_bootloader
to
	set_global_assignment -name TOP_LEVEL_ENTITY neorv32_test_setup_verilog_top
*/

`include "defines.v"

module neorv32_test_setup_verilog_top (
	// -- Global control --
	input	clk_i,		// 50MHz clock (TODO add pll)
	input	rstn_i,		// Button, active low (KEY0 the right-hand one)

	// -- GPIO --
	output [7:0] gpio_o,

	// -- UART0 -- (UNUSED, see below. TODO remove from .qsf)
	input	uart0_rxd_i,
	output	uart0_txd_o
);

	localparam BAUDRATE = 19200;

`ifndef SIM
	localparam CLK_FREQUENCY = 50000000;

	neorv32_test_setup_bootloader u_riscv (
		.clk_i			(clk_i),
		.rstn_i			(rstn_i & flags_run),
		.gpio_o			(cpu_gpio_o),
		.uart0_rxd_i	(cpu_uart0_rxd_i),
		.uart0_txd_o	(cpu_uart0_txd_o)
	);

`else
	localparam CLK_FREQUENCY = BAUDRATE * 32;	// Speed up simulation

	reg r  = 1'b1;
	assign cpu_uart0_txd_o = r;

	initial begin
	// Randomish uart rx signal data (320ns per bit period)
	#5		r = 0;	// start bit
	#320	r = 1;	// toggle data
	#640	r = 0;
	#640	r = 1;
	#640	r = 0;
	#640	r = 1;	// stop bit (with some preceeding data)

	#640	r = 0;	// start bit
	#640	r = 1;	// toggle data
	#960	r = 0;
	#960	r = 1;	// data end and eventually stop bit
	end

`endif

	wire clk = clk_i;
	wire cpu_uart0_rxd_i, cpu_uart0_txd_o;
	wire [7:0] cpu_gpio_o;

	assign uart0_txd_o = uart0_rxd_i;	// Just so Quartus does not complain about unused pins (TODO remove from qsf)
	assign gpio_o = nano_led;

    jtag_top u_jtag_top
    (
		.uart_state		(uart_state),
		.rdata_in		(rdata_in),
		.wdata_out		(wdata_out),
		.raddr_out		(raddr_out),
		.waddr_out		(waddr_out),
		.flags_out		(flags_out),
		.jtag_tx_out	(jtag_tx_data),
		.tapsigs_out	(tapsigs_unused),
		.wram_enable	(wram_enable)
    );

	wire [7:0]	tapsigs_unused;

	// Dual port ram
	reg [31:0] ram[0:2047];			// 8kB currently used to log rx_data, inits to 0 on fpga (not simulation though)
									// TODO attach to wishbone so neorv32 cpu can access it (mux for simultaneous use
									// so can use it to transfer data)
	reg [10:0] ram_rd_addr_reg = 0;
	reg [10:0] ram_wr_addr_reg = 0;
	reg [31:0] ram_wr_data_reg = 0;

	// Shift register adjusts timing of ram write strobe (delayed slightly wrt addr/data ensures properly syncd)
`define WE_LEN 3
	reg [`WE_LEN-1:0] wram_enable_d = 0;

	assign rdata_in = ram[ram_rd_addr_reg];

	wire [`DR_LENGTH-1:0] uart_state;
	wire [`DR_LENGTH-1:0] jtag_tx_data;
	wire [`DR_LENGTH-1:0] rx_mem_data;
	wire [`DR_LENGTH-1:0] rdata_in;
	wire [`DR_LENGTH-1:0] wdata_out;
	wire [`DR_LENGTH-1:0] raddr_out;
	wire [`DR_LENGTH-1:0] waddr_out;
	wire [`DR_LENGTH-1:0] flags_out;
	reg  [`DR_LENGTH-1:0] flags_d = 0;

	wire ram_write_strobe = flags_urx ?	wram_enable_rxd
										:	wram_enable_d[`WE_LEN-2] & !wram_enable_d[`WE_LEN-1];

	wire [10:0] ram_wr_addr = flags_urx	?	rx_addr[12:2]
										:	ram_wr_addr_reg;

	wire [31:0] ram_wr_data = flags_urx	?	rx_mem_data
										:	ram_wr_data_reg;

	wire wram_enable;

    always @(posedge clk)
	begin
		// Synchronize
		ram_rd_addr_reg <= flags_utxb_d ? tx_addr : raddr_out[10:0];
		ram_wr_addr_reg <= waddr_out[10:0];
		ram_wr_data_reg <= wdata_out;
		flags_d <= flags_out;

		// Shift register synchronizes and retimes write strobe
		wram_enable_d <= { wram_enable_d[`WE_LEN-2:0], wram_enable };

		// Write to ram
		if (ram_write_strobe)
		begin
			ram[ram_wr_addr] <= ram_wr_data;
		end
    end

	// wire flags_debug = flags_d[10:8] == 1;	// Not currently used
	// wire flags_raddr = flags_d[10:8] == 2;
	// wire flags_waddr = flags_d[10:8] == 3;
	// wire flags_rdata = flags_d[10:8] == 4;
	// wire flags_wdata = flags_d[10:8] == 5;

	// wire flags_tmode = flags_d[16];	// legacy from fpga/audio
	// wire flags_wrap  = flags_d[17];
	// wire flags_audio = flags_d[18];

	wire flags_run	= flags_d[19];	// Applied to nrst, active low, so fpga starts in reset mode
//	wire flags_urx  = flags_d[20];	// Enable rx buffer (moved, see below)
	wire flags_utx  = flags_d[21];	// Strobe tx
	wire flags_utxm = flags_d[22];	// Multi-byte tx mode (4 bytes per strobe)
	wire flags_utxb = flags_d[23];	// Block tx mode

`ifndef SIM
	wire flags_urx = flags_d[20];	// (LAZY) Enable rx buffer (moved from above for simulation)
`else
	wire flags_urx = 1'b1;
`endif

	wire [7:0] nano_led;
	assign nano_led = cpu_gpio_o;

	reg [12:0] rx_addr = 0;		// NB LSB 2 bits count chars in ram word
	wire [7:0] rx_data;
	wire [2:0] txstate;			// Flags returned over jtag
	reg [7:0] rx_data_d = 0;
	reg addr_increment = 0;
	reg wram_enable_rxd = 0;
	reg rxd_valid_d = 0;

    always @(posedge clk)
	begin
		rx_data_d <= rx_data;
		rxd_valid_d <= rxd_valid;

		if (flags_urx && rxd_valid && !rxd_valid_d)
		begin
			wram_enable_rxd <= 1;
			rx_data_d <= rx_data_d;
			addr_increment <= 1;
		end
		else
		begin
			wram_enable_rxd <= 0;
			addr_increment <= 0;
		end

		if (!flags_urx)
			rx_addr <= 0;
		else if (addr_increment)
			rx_addr <= rx_addr + 13'd1;
	end

	wire txd, txd_busy;
	reg [`DR_LENGTH-1:0] txd_data = 0;
	reg txd_start = 1'b0;

	wire rxd, rxd_valid, rxd_idle, rxd_endofpacket;
	wire [7:0] rxd_data;
	reg  [`DR_LENGTH-1:0] rxd_data_reg = 0;

	assign rxd = cpu_uart0_txd_o;
	assign cpu_uart0_rxd_i = txd;

uart_tx #(.FREQUENCY(CLK_FREQUENCY), .BAUD(BAUDRATE)) u_tx
	(
		.clk	(clk),
		.start	(txd_start),
		.data	(txd_data[7:0]),
		.tx		(txd),
		.busy	(txd_busy)
	);

uart_rx #(.FREQUENCY(CLK_FREQUENCY), .BAUD(BAUDRATE)) u_rx
	(
		.clk	(clk),
		.rx		(rxd),
		.data	(rxd_data),
		.valid	(rxd_valid)
	);

	reg flags_utx_d = 1'b0;
	reg flags_utxm_d = 1'b0;
	reg flags_utxb_d = 1'b0;
	reg tx_block_run = 1'b0;
	reg [2:0] tx_block_next = 3'd0;	// shift reg for timing
	reg [2:0] tx_count = 3'd0;
	reg [10:0] tx_addr = 10'd0;
	reg [10:0] tx_end = 10'd0;
	wire  [`DR_LENGTH-1:0] tx_data_mux = flags_utxb ? rdata_in : jtag_tx_data;

    always @(posedge clk)
	begin
		if (rxd_valid && !rxd_valid_d)
			rxd_data_reg <= { rxd_data, rxd_data_reg[31:8]  };	// FIFO positions data correctly in buffer

		// BEWARE there is no interlocking, so the jtagrisc driver must ensure that only ONE of the flags_utx or
		// flags_utxm/b strobe is applied and wait for txd_busy to clear before applying another (there is a short
		// delay from strobe to txd_busy, but that is swamped by the jtag delay, so there is no race condition)

		flags_utx_d <= flags_utx;
		flags_utxm_d <= flags_utxm;
		flags_utxb_d <= flags_utxb;

		txd_start <= 1'b0;			// Default action is to clear start strobe

		tx_block_next <= { tx_block_next[1:0], 1'b0 };

		if (tx_addr == tx_end)
			tx_block_run <= 1'b0;	// Overriden below when (flags_utxb && !flags_utxb_d)

		if (flags_utxb && !flags_utxb_d)
		begin
			tx_addr <= 11'd0;
			tx_end <= jtag_tx_data[10:0];
			tx_block_run <= 1'b1;
		end
		else if (tx_count == 1 && txd_start)	// Arbitary timing (ie sometime after txd_data was latched)
			tx_addr <= tx_addr + 11'd1;

		if ((flags_utxm && !flags_utxm_d) | tx_block_next[2])
		begin
			// Transmit four chars on flags_utxm
			tx_count <= 3'd4;
		end
		else
		begin
			// Transmit single char on flags_utx
			if (flags_utx && !flags_utx_d)
			begin
				txd_start <= 1'b1;			// Generate 1 cycle strobe on positive edge (delayed wrt jtag_tx_data)
				txd_data <= tx_data_mux;
			end
		end

		if (tx_count != 0)
		begin
			if (!txd_busy && !txd_start)	// txd_start is needed as txd_busy takes an extra cycle to assert
			begin
				txd_start <= 1'b1;
				tx_count <= tx_count - 3'd1;
				if (tx_count == 4)
					txd_data <= tx_data_mux;
				else
					txd_data <= { 8'd0, txd_data[31:8] };
			end
		end
		else if (tx_block_run && tx_addr != tx_end && tx_block_next == 3'd0)
			tx_block_next <= 3'd1;

	end

	wire txd_busym = txd_busy |
					 txd_start |		// txd_start avoids a one-cycle glitch at end of tx_count
					 (flags_utxb & (tx_addr != tx_end)) |
					 (tx_count != 0);

	assign rx_mem_data = rxd_data_reg;
	assign rx_data = rxd_data_reg[7:0];
	assign txstate = { flags_run, tx_block_run, txd_busym };	// txd state flags (only txd_busym is needed, but may as well
															// include the others instead of just zeros, can reuse if needed)

	// State is returned over jtag on IUART vir
	assign uart_state = { txstate[2:0], rx_addr[12:0], rxd_data_reg[23:16] , rxd_data_reg[31:24] };	// Newest rx char in LSB byte

endmodule
