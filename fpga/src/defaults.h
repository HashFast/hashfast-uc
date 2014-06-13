`define DEFAULT_VERSION    5
`define DEFAULT_PLL_BYPASS 1'b1
`define DEFAULT_BAUD_RATE  3'b000


`ifndef VERSION
`define VERSION `DEFAULT_VERSION
`endif

`ifndef MAGIC
`define MAGIC 8'd42
`endif
