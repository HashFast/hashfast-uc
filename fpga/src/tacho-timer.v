`include "hf-common.h"

module tacho_timer (input        clk,               // 8 or 16 mhz
                    input        reset,
                    input        one_hz,            // high one cycle per second
                    input        tacho,             // we count negative edges on this
                    output [7:0] pulses_per_second  // average of the last two seconds
                    );

reg [8:0]  average;
reg [7:0]  prev_count1;
reg [7:0]  prev_count2;
reg [7:0]  count;
reg        trig0, trig1, trig_meta;

assign pulses_per_second = average[8:1];


always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        count <= `D 8'd0;
        average <= `D 'b0;
        prev_count1 <= `D 8'd0;
        prev_count2 <= `D 8'd0;
        {trig0, trig1, trig_meta} <= `D 3'b0;
      end
    else
      begin
        average <= `D prev_count1 + prev_count2;
        if (one_hz)
          begin
            count <= `D (trig0 && !trig1)?8'd1:8'd0;
            {prev_count2, prev_count1} <= `D {prev_count1, count};
          end
        else
        if (trig0 && !trig1 && count != 8'hff)
          count <= `D count + 8'd1;

        {trig0, trig1, trig_meta} <= `D {trig1, trig_meta, tacho};
      end
  end
endmodule

