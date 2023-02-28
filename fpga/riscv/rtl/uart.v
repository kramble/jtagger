// uart.c - simple uart, only suitable for neorv32 (crude sampling, no error handling at all)

module uart_tx (
	input		clk,
	input		start,
	input [7:0]	data,
	output		tx,
	output		busy
);

	parameter FREQUENCY = 50000000;	// 50MHz
	parameter BAUD = 19200;
	parameter PERIOD = FREQUENCY / BAUD - 1;

	reg [15:0]	timer = 0;		// OK for current baud rate, fails if FREQUENCY/BAUD > 2^16
	reg [3:0]	counter = 0;
	reg [7:0]	data_d = 0;

	// Use explicit regs as Quartus 10.1 port regs do not initialise correctly
	reg			tx_reg = 1;
	reg			busy_reg = 0;

	assign tx = tx_reg;
	assign busy = busy_reg;

	always @(posedge clk)
	begin
		if (start && counter == 0)
		begin
			timer <= 0;
			counter <= counter + 4'd1;
			data_d <= data;
			tx_reg <= 0;
			busy_reg <= 1;
		end
		else if (timer >= PERIOD)
		begin
			timer <= 0;
			if (counter != 0 && counter < 9)
			begin
				counter <= counter + 4'd1;
				data_d <= { 1'b0, data_d[7:1] };
				tx_reg <= data_d[0];
			end
			else if (counter != 0 && counter < 11)
			begin
				counter <= counter + 4'd1;
				tx_reg <= 1;
			end
			else
			begin
				counter <= 0;
				busy_reg <= 0;
			end
		end
		else
			timer <= timer + 16'd1;
	end
endmodule

module uart_rx (
	input				clk,
	input				rx,
	output [7:0]		data,
	output				valid
);

	parameter FREQUENCY = 50000000;	// 50MHz
	parameter BAUD = 19200; 
	parameter PERIOD = FREQUENCY / BAUD / 8 - 1;	// oversampling to sync with start pulse edge

	reg [18:0]	timer = 0;		// OK for current baud rate, fails if FREQUENCY/BAUD > 2^16
	reg [6:0]	counter = 0;		// Needs to count up to 'd72, see below
	reg [7:0]	data_d = 0;

	// Use explicit regs as Quartus 10.1 port regs do not initialise correctly
	reg			valid_reg = 0;

	assign data = data_d;
	assign valid = valid_reg;

	// reg debug = 0;

	always @(posedge clk)
	begin
		// debug <= 0;
		if (timer >= PERIOD)
		begin
			timer <= 0;
			if (counter == 0)
			begin
				if (!rx)
				begin
					counter <= counter + 7'd1;
					data_d <= 0;		// not neccessary but tidier for simulation
					valid_reg <= 0;
					// we don't check for short start pulse
				end
			end
			else if (counter > 72)
			begin
				counter <= 0;
				valid_reg <= 1;
				// we don't check for stop bits
			end
			else if (counter[2:0] == 4)	// sample data at (roughly) mid-bit
			begin
				counter <= counter + 7'd1;
				data_d <= { rx, data[7:1] };
				// debug <= 1;
			end
			else
			begin
				counter <= counter + 7'd1;
			end
		end
		else
			timer <= timer + 19'd1;
	end
endmodule

