// defines.v

// `timescale 1ns / 1ps
`timescale 1ns / 1ns

// Define SIM to configure for simulation vs synthesis, set automatically from the
// simulator via the VERILATOR or __ICARUS__ macros. (I'm using SIM since I have used
// that in the past with the Quartus ModelSim GUI compiler dialog/macro definition)
// TODO automate that too, though my google-fu is currently failing on that

`ifdef VERILATOR
`define SIM
`endif

`ifdef __ICARUS__
`define SIM
`endif

`define	IR_LENGTH	4
`define	DR_LENGTH	32

`define IIDENT    4'b0000
`define IRADDR    4'b0001
`define IWADDR    4'b0010
`define IRDATA    4'b0101
`define IWDATA    4'b0110
`define IUART     4'b1100
`define IAUDIO    4'b1101
`define IFLAGS    4'b1110
`define IBYPASS   4'b1111


