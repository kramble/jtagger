/* dac.v

Includes code from https://github.com/hamsternz/second_order_sigma_delta_DAC
Licensed as follows ...

MIT License

Copyright (c) 2020 Mike Field

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

module dac
    (
		input clk,
		input [15:0] data_in,
		output audio_out
	);

wire dac_reset = 1'b1;	// Active low reset, hold it constant high
wire dac_ce = 1'b1;		// Clock enable, hold it constant high
wire dac_out_so;		// Output audio

second_order_dac dac_so (clk, dac_reset, dac_ce, data_in, dac_out_so);

assign audio_out = dac_out_so;

endmodule

// ********* BEGIN hamsternz (slightly modified) ***********

module second_order_dac(
  input wire i_clk,
  input wire i_res,
  input wire i_ce,
  input wire [15:0] i_func, 
  output wire o_DAC
);

  reg this_bit;
 
  reg [19:0] DAC_acc_1st = 0;
  reg [19:0] DAC_acc_2nd = 0;
  reg [19:0] i_func_extended = 0;
   
  assign o_DAC = this_bit;

  always @(*)
     i_func_extended = {i_func[15],i_func[15],i_func[15],i_func[15],i_func};
    
  always @(posedge i_clk or negedge i_res)
    begin
      if (i_res==0)
        begin
          // DAC_acc_1st<=16'd0;	// Changed these as quartus WARNS about mixing blocking and nonblocking
          // DAC_acc_2nd<=16'd0;	// Reset is tied high in system, so it won't matter anyway
          DAC_acc_1st = 16'd0;
          DAC_acc_2nd = 16'd0;
          this_bit = 1'b0;
        end
      else if(i_ce == 1'b1) 
        begin
          if(this_bit == 1'b1)
            begin
              DAC_acc_1st = DAC_acc_1st + i_func_extended - 20'h8000; // (2**15);	// Replaced (2**15) with 20'h8000 to
              DAC_acc_2nd = DAC_acc_2nd + DAC_acc_1st     - 20'h8000; // (2**15);	// fix truncation warnings in quartus
            end
          else
            begin
              DAC_acc_1st = DAC_acc_1st + i_func_extended + 20'h8000; // (2**15);
              DAC_acc_2nd = DAC_acc_2nd + DAC_acc_1st + 20'h8000; // (2**15);
            end
          // When the high bit is set (a negative value) we need to output a 0 and when it is clear we need to output a 1.
          this_bit = ~DAC_acc_2nd[19];
        end
    end
endmodule

// ********* END hamsternz ***********


