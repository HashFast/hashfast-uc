`ifndef __HF_COMMON_H__
`define __HF_COMMON_H__

`ifndef __TIMESCALE
//`timescale 1ns/10ps
`timescale 1ns/1ns
`define __TIMESCALE
`endif

`ifdef SIMULATION
`define D #1
`else
`define D
`endif

//
// Default clock and reset edges for ASIC
//
`ifndef RESET_EDGE
`define RESET_EDGE         negedge
`endif
`ifndef CLOCK_EDGE
`define CLOCK_EDGE         posedge
`endif
`ifndef CLOCK_NEDGE
`define CLOCK_NEDGE        negedge
`endif
`ifndef RESET_LEVEL
`define RESET_LEVEL        1'b0
`endif
`ifndef CLOCK_SENS
`define CLOCK_SENS        `CLOCK_EDGE clk
`endif
`ifndef NCLOCK_SENS
`define NCLOCK_SENS       `CLOCK_NEDGE clk
`endif
`ifndef CLOCK_RESET_SENS
`define CLOCK_RESET_SENS  `CLOCK_EDGE clk or `RESET_EDGE reset
`endif
`endif

