// simple SPI-SLAVE
// Only does CPOL=1, CPHA=1
//
`include "hf-common.h"

module spi_slave (input            clk,
                  input            reset,

                  input            spi_clk,
                  input            spi_en,
                  input            spi_din,
                  output reg       spi_dout,
                  output           spi_dout_en,

                  output reg       reg_we,
                  output reg       reg_re,
                  output reg [6:0] reg_addr,
                  output reg [7:0] reg_wd,
                  input [7:0]      reg_rd
                  );


assign spi_dout_en = spi_en;

// ------------------------------------------------------------------------------
// input sampling
//
reg [2:0] bit_cnt;
reg       writing;
reg [7:0] shift_in;
reg [6:0] shift_out;
reg       addr_byte;  // handling an incoming address byte

// ------------------------------------------------------------------------------
// sample the clock & enable
//
reg       sclk_meta, sclk;
reg       en_meta, en;
wire      redge = sclk_meta & ~sclk;  // spi-clk rising edge.

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      {sclk, sclk_meta, en, en_meta} <= `D 4'b00;
    else
      {sclk, sclk_meta, en, en_meta} <= `D {sclk_meta, spi_clk, en_meta, spi_en};
  end

// ------------------------------------------------------------------------------
// outputs and control
//
always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        bit_cnt <= `D 5'd0;
        writing <= `D 1'd0;
        addr_byte <= `D 1'b1;
        reg_addr <= `D 7'b0;
        reg_wd <= `D 8'b0;
        {reg_we, reg_re} <= `D 3'b0;
      end
    else
      begin
        {reg_we, reg_re} <= `D 2'b0;

        if (reg_re || reg_we)
          reg_addr <= `D reg_addr + 7'd1; // increment in prep for another cycle

        if (!en)
          begin
            bit_cnt <= `D 5'd0;
            writing <= `D 1'd0;
            addr_byte <= `D 1'b1;
            {reg_we, reg_re} <= `D 2'b0;
          end
        else
        if (redge)
          begin
            bit_cnt <= `D bit_cnt + 3'd1;

            if (addr_byte == 1'b1 && bit_cnt == 3'd0)
              writing <= `D ~shift_in[0];

            if (bit_cnt == 3'd7)
              begin
                addr_byte <= `D 'b0;
                reg_we <= `D ~addr_byte && writing;
                reg_re <= `D ~writing;
                reg_wd <= `D shift_in;

                if (addr_byte)
                  reg_addr <= `D shift_in[6:0];
              end
          end
      end
  end

// ------------------------------------------------------------------------------
// receive side shifter
//
always @(posedge spi_clk /* or negedge spi_en */ or `RESET_EDGE reset)
  begin
    if (reset == `RESET_LEVEL)
      shift_in <= `D 8'd0;
    else
      shift_in <= `D {shift_in[6:0], spi_din};
  end

// ------------------------------------------------------------------------------
// output side

always @(negedge spi_clk or negedge spi_en)
  begin
    if (spi_en == 1'b0)
      begin
        shift_out <= `D 7'b0;
        spi_dout <= `D 'b0;
      end
    else
      begin
        if (bit_cnt == 3'b0)
          begin
            spi_dout <= `D reg_rd[7];
            shift_out <= `D reg_rd[6:0];
          end
        else
          begin
            spi_dout <= `D shift_out[6];
            shift_out <= `D {shift_out[5:0], 1'b0};
          end
      end
  end

endmodule
