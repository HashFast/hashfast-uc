// ------------------------------------------------------------------------------
// PSU sync gen.
// ------------------------------------------------------------------------------
// outputs four clock phases 360/16 degrees out of phase.
//

`include "hf-common.h"

module sync_gen (input      clk,        // 8 or 16Mhz
                 input      reset,
                 input      clk16,      // true if clk is 16Mhz
                 output reg ph1,
                 output reg ph2,
                 output reg ph3,
                 output reg ph4
                 );



// ------------------------------------------------------------------------------
// Four 250khz outputs, 2 cycles delayed for an 8Mhz clock, 4 cycles for a 
// 16Mhz clock.
//
reg [5:0]                    phase;

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        phase <= `D 6'd0;
        {ph1, ph2, ph3, ph4} <= `D 4'd0;
      end
    else
      begin
        phase <= `D phase + 6'd1;
        if (clk16 == 1'b1)
          begin
            case (phase[4:0])
              5'd0:
                ph1 <= `D phase[5] == 1'b0;
              5'd4:
                ph2 <= `D phase[5] == 1'b0;
              5'd8:
                ph3 <= `D phase[5] == 1'b0;
              5'd12:
                ph4 <= `D phase[5] == 1'b0;
              default:
                {ph1, ph2, ph3, ph4} <= `D {ph1, ph2, ph3, ph4};
            endcase
          end
        else
          begin
            case (phase[3:0])
              4'd0:
                ph1 <= `D phase[4] == 1'b0;
              4'd2:
                ph2 <= `D phase[4] == 1'b0;
              4'd4:
                ph3 <= `D phase[4] == 1'b0;
              4'd6:
                ph4 <= `D phase[4] == 1'b0;
              default:
                {ph1, ph2, ph3, ph4} <= `D {ph1, ph2, ph3, ph4};
            endcase
          end
      end
  end
endmodule
