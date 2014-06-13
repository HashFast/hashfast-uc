`include "hf-common.h"
`include "defaults.h"

// Bring up/debug hacks...
//`define CLOCK_HACK
//`define SLOW_HACK
//`define SYNC_HACK
//`define TRIG_HACK1
//`define TRIG_HACK2

`ifdef TRIG_HACK1
`define TRIG_HACK
`else
`ifdef TRIG_HACK2
`define TRIG_HACK
`endif
`endif

module top (input            clk,           // 8 or 16 mhz, assume 8 at reset
            input            reset_in,

            // regulator control interface
            output [3:0]     reg_sync, // Regulator SYNC outputs
            input [3:0]      pgood, // power good signals
            output reg [3:0] reg_en, // enables for each regulator

            // asic & board to board serial links
            input            sin_uc, // serial link into the uC
            input            sin_up, // serial link input from up
            input            sin_down, // from down
            input [3:0]      sin, // from the asic

            output           sout_uc, // serial link from the uC
            output           sout_up, // serial link output to up
            output           sout_down, // to down
            output [3:0]     sout, // to the asic

            // ASIC mode controls
            output reg       pll_bypass, // asic pll-mode
            output reg       asic_reset_n, // active low asic reset
            output reg [2:0] baud_rate, // level translated output

            //
            // SPI slave interface for access by the uC
            // The SPI clock needs to be at least 6 clk periods. clk is
            // assumed to be 8mhz at start up, 1mhz is probably an Ok
            // initial value for the SPI clock frequency and it could
            // be increased to 2Mhz is clk is 16mhz.
            //
            input            spi_clk, // spi slave connection to
            input            spi_en_bar, // the uC
            input            spi_din,
            output           spi_dout,

            // I2C master interface which polls the ADCs
            inout            scl, // i2c bus for the monitoring
            inout            sda,

`ifdef BLINKY
            output reg [4:0] blinky,
`endif
`ifndef CLOCK_HACK
            // misc inputs
            input [7:0]      config_sw,
            input [3:0]      tacho,
`else
            input [7:1]      config_sw,
            input [2:0]      tacho,
            input            fclk,
            output           clk_o,
`endif

`ifdef TRIG_HACK
            output reg [1:0] trig,
`endif

            // misc outputs
            output           fpga_initd // present/interrupt signal
            );

// ------------------------------------------------------------------------------
// hacks used thru bring up.
//
`ifdef CLOCK_HACK
`ifdef SLOW_HACK

assign clk_o = clk;
`else

reg [4:0]                    co_div;
reg                          co;

assign clk_o = co;

always @(posedge fclk or negedge reset)
  begin
    if (reset == `RESET_LEVEL)
      begin
        co <= `D 'b0;
        co_div <= 'd0;
      end
    else
      begin
        co <= `D co_div == 'd0 || co_div == 'd1;
        if (co_div == 'd3)
          begin
            co_div <= `D 'd0;
          end
        else
          co_div <= `D co_div + 'd1;
      end
  end
`endif
`endif

`ifdef HAVE_ADC_POLLING
wire                         ipend;                     // any interrupt pending
`endif

wire                         spi_dout_i, spi_dout_en;
wire                         sda_en, scl_en;
wire                         sda_in, scl_in;
wire                         ext_reset;
wire [3:0]                   pgood_async;

// ------------------------------------------------------------------------------
// Explicit cells for the non-trivial IOs.
//
SB_IO  #(
         .PIN_TYPE(6'b101000) // simple tristate output
         ) spi_dout_pad (
                        .PACKAGE_PIN(spi_dout),
                        .LATCH_INPUT_VALUE(),
                        .CLOCK_ENABLE(),
                        .INPUT_CLK(),
                        .OUTPUT_CLK(),
                        .OUTPUT_ENABLE(spi_dout_en),
                        .D_OUT_0(spi_dout_i),
                        .D_OUT_1(),
                        .D_IN_0(),
                        .D_IN_1()
                        );

SB_IO  #(
         .PIN_TYPE(6'b101000) // tristate output, registered input
         ) sda_pad (
                    .PACKAGE_PIN(sda),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(),
                    .INPUT_CLK(clk),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(sda_en),
                    .D_OUT_0(1'b0),
                    .D_OUT_1(1'b0),
                    .D_IN_0(sda_in),
                    .D_IN_1()
                    );

SB_IO  #(
         .PIN_TYPE(6'b101000), // tristate output, registered input
         .PULLUP(1)
         ) scl_pad (
                    .PACKAGE_PIN(scl),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(1'b1),
                    .INPUT_CLK(clk),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(scl_en),
                    .D_OUT_0(1'b0),
                    .D_OUT_1(1'b0),
                    .D_IN_0(scl_in),
                    .D_IN_1()
                    );

// add a pull up to the reset input
SB_IO #(
        .PIN_TYPE(6'b000001),  // simple input
        .PULLUP(1'b1)          // with a pull up.
        ) reset_pad (
                    .PACKAGE_PIN(reset_in),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(),
                    .INPUT_CLK(),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(),
                    .D_OUT_0(),
                    .D_OUT_1(),
                    .D_IN_0(ext_reset),
                    .D_IN_1()
                     );

// present signal.
SB_IO #(
        .PIN_TYPE(6'b101000),  // simple tristate output
        .PULLUP(1)             // with a pull up
        ) fpga_initd_pad (
                    .PACKAGE_PIN(fpga_initd),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(),
                    .INPUT_CLK(),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(ipend),
                    .D_OUT_0(1'b0),
                    .D_OUT_1(),
                    .D_IN_0(),
                    .D_IN_1()
                     );

// register the pgood inputs
SB_IO #(
        .PIN_TYPE(6'b000000)  // no output, registered input
        ) pgood_pad_0 (
                    .PACKAGE_PIN(pgood[0]),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(1'b1),
                    .INPUT_CLK(clk),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(),
                    .D_OUT_0(),
                    .D_OUT_1(),
                    .D_IN_0(pgood_async[0]),
                    .D_IN_1()
                     );


SB_IO #(
        .PIN_TYPE(6'b000000)  // no output, registered input
        ) pgood_pad_1 (
                    .PACKAGE_PIN(pgood[1]),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(1'b1),
                    .INPUT_CLK(clk),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(),
                    .D_OUT_0(),
                    .D_OUT_1(),
                    .D_IN_0(pgood_async[1]),
                    .D_IN_1()
                     );


SB_IO #(
        .PIN_TYPE(6'b000000)  // no output, registered input
        ) pgood_pad_2 (
                    .PACKAGE_PIN(pgood[2]),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(1'b1),
                    .INPUT_CLK(clk),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(),
                    .D_OUT_0(),
                    .D_OUT_1(),
                    .D_IN_0(pgood_async[2]),
                    .D_IN_1()
                     );


SB_IO #(
        .PIN_TYPE(6'b000000)  // no output, registered input
        ) pgood_pad_3 (
                    .PACKAGE_PIN(pgood[3]),
                    .LATCH_INPUT_VALUE(),
                    .CLOCK_ENABLE(1'b1),
                    .INPUT_CLK(clk),
                    .OUTPUT_CLK(),
                    .OUTPUT_ENABLE(),
                    .D_OUT_0(),
                    .D_OUT_1(),
                    .D_IN_0(pgood_async[3]),
                    .D_IN_1()
                     );


// ------------------------------------------------------------------------------
// Accessible registers.
//
// Address Mode Name	     Description
// ------------------------------------------------------------------------------
//   0      R   Magic        magic number
//
//   1      R   Version      fpga code version, 0x01 initially
//
//   2      R   Switches     Dip switch values
//
//   3      RW  Config       0,0,0,0,i2c_rate[1:0], big_endian,clk16
//
//                           default 00
//                           i2c_rate 00 => 100khz
//                                    01 => 200khz
//                                    1x => 400khz
//                           big_endian == 1 => byte swap 16 bit values on read & write
//                           clk16 == 1 => clk is 16 mhz not 8mhz
//
//   4      RW	AsicCntl     0,0,0,reset, pll-bypass, baud2, baud1, baud0
//
//          reset time value 8'b0000<`DEFAULT_PLL_BYPASS><`DEFAULT_BUAD_RATE>
//
//   5      R   RegStatus    pgood[3:0], reg_en[3:0]
//   5      W   RegEnable    {4'bxxxx, reg_en[3:0]}
//
//          reset time value 8'b0, all regulators disabled
//
//   6      RW  VADC-config  {2'b0, 60hz-reject, 50hz-reject, fast-mode, gain[2:0]} 
//   7      RW  IADC-config  {2'b0, 60hz-reject, 50hz-reject, fast-mode, gain[2:0]}
//
//          ADC configuration, 50/60hz-reject enable rejection filters for those frequencies, probably
//                             makes no difference in our application, included be they're available.
//                             fast-mode sets the converters into 15hz sample rate mode. 
//                             In fast-mode the ADC auto calibration function is disabled. The total
//                             unadjusted error is typically 15uV. For the voltage ADC the maximum useful 
//                             gain is 1x, giving a FS reading of 1.25 volts, 15uV error is less than
//                             2 lsbs.
//
//                             For the current ADCs at maximum gain, 3.255 amps full-scale, 15uV 
//                             represents about 0.3% error.
//
//                             Gain depends on fast mode.
//
//                                fast-mode gain PGA gain
//
//                                    0     000      1x
//                                    0     001      4x
//                                    0     010      8x
//                                    0     011     16x
//                                    0     100     32x
//                                    0     001     64x
//                                    0     110    128x
//                                    0     111    256x
//                                 
//                                    1     000      1x
//                                    1     001      2x
//                                    1     010      4x
//                                    1     011      8x
//                                    1     100     16x
//                                    1     001     32x
//                                    1     110     64x
//                                    1     111    128x
//
//                            Affective full scale and LSB weights of the Voltage ADCs
//
//                                 gain   FS (volts),  LSB weight
//                                  1     1.25         19.0734uV
//                                  2     0.625         9.5367uV
//
//                            Affective full scale and LSB weights of the Current ADCs
//
//                                 gain   FS (amps),        LSB weight (milliamps)
//
//                                   1      833.3333        12.718156575
//                                   2      416.6666         6.357828776 
//                                   4      208.3333         3.1789143880
//                                   8      104.1666         1.5894571940
//                                  16       52.083333       0.7947285970
//                                  32       26.041666666    0.39736429850
//                                  64       13.020833333    0.19868214925130
//                                 128        6.510416666    0.0993410746256510
//                                 256        3.255208333    0.0496705373128255
//
//   8/9    RW  OCMsb        Most significant byte of the over current reference
//   9/8    RW  OCLsb        Least significant byte of the over current reference
//
//                           16 bit over current reference. If non-zero, an over
//                           current warning interrupt is generated if a regulator
//                           total current exceeds this value.
//
//  10/11   RW PhOCMsb       Most significant byte of the per phase over current reference       
//  11/10   RW PhOCMsb       Least significant byte of the per phass over current reference
//
//                           16 bit over current reference. If non-zero, an over
//                           current warning interrupt is generated if a phase
//                           current exceeds ths value.
//
//  12-15   R  Tacho0 .. 3   Current value of the four tacho timers. Average number
//                           of pulses during the last 8 seconds.
//------------------------------------------------------------------------------
// Routing controls, the three LSBs select the
// the data source for each serial input.
//
//         value
//         ==================
//          000    set inative (high)
//          001    uc-sout
//          010    down-sout
//          011    up-sout
//          100    die0-sout
//          101    die1-sout
//          110    die2-sout
//          111    die3-sout
//
//
//   16     RW  uc-sin-sel
//   17     RW  down-sin-sel
//   18     RW  up-sin-sel
//   19     RW  die0-sin-sel
//   20     RW  die1-sin-sel
//   21     RW  die2-sin-sel
//   22     RW  die3-sin-sel
//
//------------------------------------------------------------------------------
//   30     W  Interrupt Enable  
//             {4'bx, Samples, I2CErrors, Overflows, OverCurrent}
//
//   30     R  Interrupt Pending 
//             {2'b0, Overrun, Samples, I2CErrors, Overflows, OverCurrent}
//
//              Enable and pending bits
//
//                    Samples:     new ADC samples available
//                    I2CErrors:   i2c errors occurred in the most
//                                 recent sample period
//                    Overflows:   ADC overflows occurred in the
//                                 most recent sample period.
//                    OverCurrent: Current limits were exceeded
//                                 in the most recent sample period.
//
//              Read only status bits.
//
//                    Overrun:     True if the monitoring adc sample buffer
//                                 has been overrun
//
//   31     W   ignored
//          R   {4'b0, sticky-pgood}
//
//              For each enabled reg set one by a poll and clear should
//              pgood ever fall.
//
// ------------------------------------------------------------------------------
//
// During configuration all flops are held in reset. If there is no external
// reset at the time configuration ends everything starts running from that
// point. This means that anything which needs to start up with a one
// in a flop won't work. 
//
// The following generates an internal reset which will be active coming
// out of configuration.
//
//
wire                         reset;                    // reset to the world
reg                          internal_reset_n;         // local always active low reset
reg [3:0]                    reset_release_delay;      // delay until it is released
reg [8:0]                    reset_assert_delay;       // delay until it is asserted

assign reset = (internal_reset_n == 'b0)?`RESET_LEVEL:~`RESET_LEVEL;

always @(`CLOCK_EDGE clk)
  begin
    if (ext_reset == `RESET_LEVEL)
      begin
        if (reset_assert_delay > 'd0)
          reset_assert_delay <= `D reset_assert_delay - 'd1;
        else
          internal_reset_n <= `D 0;
        reset_release_delay <= `D 'b0;
      end
    else
    if (reset_release_delay != 'd15)
      begin
        internal_reset_n <= `D 'b0;
        reset_release_delay <= `D reset_release_delay + 'd1;
      end
    else
      begin
        internal_reset_n <= `D 'b1;
        reset_assert_delay <= `D 'd511;
      end
  end

`ifdef TRIG_HACK1
always @(*)
  begin
    trig[0] = ext_reset;
    trig[1] = asic_reset_n;
  end
`endif

// ------------------------------------------------------------------------------
// Accessible registers which are not directly outputs at the top level.
//
reg        uninitialised;

// config

reg  [3:0] conf;
reg  [3:0] sticky_pgood;
reg  [3:0] pgood_meta;
wire       clk16 = conf[0] ;
wire       big_endian = conf[1];

`ifdef HAVE_ADC_POLLING
wire [1:0] i2c_div = conf[3:2];

// interrupts

reg [3:0]  interrupt_enable;
reg [3:0]  interrupt_pending;
reg        mbuf_overrun;
wire       mbuf_empty;

assign ipend = (interrupt_pending & interrupt_enable) != 4'b0 || uninitialised;
`else

// voltage adc configuration
reg [5:0]  vadc_config;
// current adc configuration
reg [5:0]  iadc_config;
assign ipend = 'b0;
`endif

reg [15:0] oc_ref;      // over current threshold
reg [15:0] ph_oc_ref;   // individual phase OC threshold

wire       have_oc_ref = oc_ref != 16'b0;
wire       have_ph_oc_ref = ph_oc_ref != 16'b0;

always @(`CLOCK_SENS)
  pgood_meta <= `D pgood_async;

`ifdef HAVE_ADC_POLLING
// ------------------------------------------------------------------------------
// i2c phase generator, 5 phases per i2c cycle. Cycles at 100/200/400khz
//
reg        i2c_phase;
reg [4:0]  i2c_counter;

wire [4:0] i2c_last_count = 5'h1f >> i2c_div;

assign poop = i2c_phase;

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        i2c_phase <= `D 1'b0;
        i2c_counter <= `D 'd0;
      end
    else
      begin
        i2c_phase <= `D 'b0;
        i2c_counter <= `D i2c_counter + 'd1;

        if (i2c_counter == ((clk16)?i2c_last_count:i2c_last_count >> 1))
          begin
            i2c_phase <= `D 'b1;
            i2c_counter <= `D 'd0;
          end
      end
  end
`endif

// ------------------------------------------------------------------------------
// 10 hz timer
//
localparam [21:0] TEN_HZ_16 = 1599998;
localparam [21:0] TEN_HZ_8  = 799998;

reg [20:0] ten_hz_counter;
reg        ten_hz;

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        ten_hz <= `D 1'b0;
        ten_hz_counter <= `D 21'd0;
      end
    else
    if ((clk16)?(ten_hz_counter == TEN_HZ_16):(ten_hz_counter == TEN_HZ_8))
      begin
        ten_hz_counter <= `D 21'd0;
        ten_hz <= `D 'd1;
      end
    else
      begin
        ten_hz_counter <= `D ten_hz_counter + 21'd1;
        ten_hz <= `D 'd0;
      end
  end


// ------------------------------------------------------------------------------
// A one hz timer.
//
reg [3:0]  one_hz_counter;
reg        one_hz;

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        one_hz <= `D 1'b0;
        one_hz_counter <= `D 4'd0;
      end
    else
    if (ten_hz)
      begin
        if (one_hz_counter == 4'd9)
          begin
            one_hz_counter <= `D 4'd0;
            one_hz <= `D 'd1;
          end
        else
          one_hz_counter <= `D one_hz_counter + 4'd1;
      end
    else
      one_hz <= `D 'd0;
  end

// ------------------------------------------------------------------------------
// Routing. Apart from the 7 registers used to configure it, routing
// is purely combinatorial.
//
reg [2:0]  route_select [0:6];

wire [6:0] s_out;
wire [7:0] s_in = {sin, sin_down, sin_up, sin_uc, 1'b0};
assign {sout, sout_down, sout_up, sout_uc} = s_out;

generate
  genvar     X;
  for (X = 0; X < 7; X = X + 1)
    begin:make_routes
      assign s_out[X] = s_in[route_select[X]];
    end
endgenerate


// ------------------------------------------------------------------------------
// tacho interfaces
//
wire [7:0] tach [0:3];

tacho_timer tacho0 (.clk(clk),
                    .reset(reset),
                    .one_hz(one_hz),
                    .tacho(tacho[0]),
                    .pulses_per_second(tach[0]));

tacho_timer tacho1 (.clk(clk),
                    .reset(reset),
                    .one_hz(one_hz),
                    .tacho(tacho[1]),
                    .pulses_per_second(tach[1]));

tacho_timer tacho2 (.clk(clk),
                    .reset(reset),
                    .one_hz(one_hz),
                    .tacho(tacho[2]),
                    .pulses_per_second(tach[2]));
`ifndef CLOCK_HACK
tacho_timer tacho3 (.clk(clk),
                    .reset(reset),
                    .one_hz(one_hz),
                    .tacho(tacho[3]),
                    .pulses_per_second(tach[3]));
`endif

// ------------------------------------------------------------------------------
// Writing slave registers.
//
wire       reg_we;
wire       reg_re;
wire [6:0] reg_addr;
wire [7:0] reg_wd;
reg  [7:0] hreg_rd;

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        conf <= `D 2'b0;
        {asic_reset_n, pll_bypass, baud_rate} <= `D {1'b0, `DEFAULT_PLL_BYPASS, `DEFAULT_BAUD_RATE};
        vadc_config <= `D 6'b0;
        iadc_config <= `D 6'b0;
        oc_ref <= `D 16'd0;
        ph_oc_ref <= `D 16'd0;
`ifdef HAVE_ADC_POLLING
        interrupt_enable <= `D 'b0;
`endif
`ifdef BLINKY
        blinky <= `D 'b0;
`endif
      end
    else
    if (reg_we)
      begin
`ifdef BLINKY
        blinky[4] <= 'b1;
`endif
        if (reg_addr[6:4] == 3'b000)
          begin
            case (reg_addr[3:0])
              4'd3:
                conf <= `D reg_wd[3:0];

              4'd4:
                {asic_reset_n, pll_bypass, baud_rate} <= `D reg_wd[4:0];

              4'd5:
                begin
`ifdef BLINKY
                  blinky[3:0] <= reg_wd;
`endif
                end

              4'd6:
                vadc_config <= `D reg_wd[5:0];

              4'd7:
                iadc_config <= `D reg_wd[5:0];

              4'd8:
                oc_ref <= (big_endian)?{reg_wd, oc_ref[7:0]}:{oc_ref[15:8],reg_wd};
          
              4'd9:
                oc_ref <= (big_endian)?{oc_ref[15:0], reg_wd}:{reg_wd, oc_ref[7:0]};

              4'd10:
                ph_oc_ref <= (big_endian)?{reg_wd, ph_oc_ref[7:0]}:{ph_oc_ref[15:8],reg_wd};
          
              4'd11:
                ph_oc_ref <= (big_endian)?{ph_oc_ref[15:0], reg_wd}:{reg_wd, ph_oc_ref[7:0]};
            endcase
          end
`ifdef HAVE_ADC_POLLING
        else
        if (reg_addr == 7'd31)
          interrupt_enable <= `D reg_wd[3:0];
`endif
        else
        if (reg_addr[6:3] == 4'b0010)
          begin
            route_select[reg_addr[2:0]] <= `D reg_wd[2:0];
          end
      end
`ifdef BLINKY
    else
      begin
        if (one_hz)
          blinky <= 'd0;
      end
`endif
  end

//
// reg-en is split out from the other registers so that it can be reset
// by the undelayed external reset signal.
//
always @(`CLOCK_EDGE clk or `RESET_EDGE ext_reset)
  begin
    if (ext_reset == `RESET_LEVEL)
      reg_en <= `D 'b0;
    else
    if (reg_we && reg_addr == 'd5)
      reg_en <= `D reg_wd[3:0];
  end

// ------------------------------------------------------------------------------
// slave read interface.
//
always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      hreg_rd <= `D 8'h00;
    else
      begin
        if (reg_re)
          begin
            case (reg_addr)
              7'h00:
                hreg_rd <= `D `MAGIC;

              7'h01:
                hreg_rd <= `D 8'd`VERSION;

              7'h02:
                hreg_rd <= `D config_sw;

              7'h03:
                hreg_rd <= `D {6'b0, big_endian, clk16};

              7'h04:
                hreg_rd <= `D {3'b0, asic_reset_n, pll_bypass, baud_rate};

              7'h05:
                hreg_rd <= `D {pgood_meta, reg_en};

              7'h06:
                hreg_rd <= `D {2'b0, vadc_config};

              7'h07:
                hreg_rd <= `D {2'b0, iadc_config};

              7'h8:
                hreg_rd <= `D (big_endian)?oc_ref[15:8]:oc_ref[7:0];

              7'h9:
                hreg_rd <= `D (big_endian)?oc_ref[7:0]:oc_ref[15:8];

              7'ha:
                hreg_rd <= `D (big_endian)?ph_oc_ref[15:8]:ph_oc_ref[7:0];

              7'hb:
                hreg_rd <= `D (big_endian)?ph_oc_ref[7:0]:ph_oc_ref[15:8];

              7'hc:
                hreg_rd <= `D tach[0];

              7'hd:
                hreg_rd <= `D tach[1];

              7'he:
                hreg_rd <= `D tach[2];

              7'hf:
                hreg_rd <= `D tach[3];

              7'b0010xxx:
                hreg_rd <= `D route_select[reg_addr[2:0]];

              7'd30:
                hreg_rd <= `D {4'b0, sticky_pgood};

`ifdef HAVE_ADC_POLLING
              7'd31:
                hreg_rd <= `D {3'b0, mbuf_overrun, interrupt_pending};
`endif

              default:
                hreg_rd <= `D 8'h00;
            endcase
          end
      end
  end

// ------------------------------------------------------------------------------
// sticky pgood
//
wire [3:0] falling_pgood_edge;
assign falling_pgood_edge = pgood_meta & ~pgood_async;

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      sticky_pgood <= `D 4'b0;
    else
    if (reg_we && reg_addr == 7'd5)
      sticky_pgood <= `D reg_wd[3:0];
    else
    if (reg_re && reg_addr == 7'd30)
      sticky_pgood <= `D reg_en;
    else
      sticky_pgood <= `D sticky_pgood & ~falling_pgood_edge;
  end

`ifdef TRIG_HACK2
always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      trig <= `D 2'b0;
    else
      begin
        trig[0] <= `D falling_pgood_edge;
        trig[1] <= `D sticky_pgood;
      end
  end
`endif

 
`ifdef HAVE_ADC_POLLING
// ------------------------------------------------------------------------------
// interrupt handling
//
wire update_complete;
wire i2c_error;
wire of_error;
wire oc_error;

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        interrupt_pending <= `D 4'b0;
        uninitialised <= `D 'b1;
      end
    else
      begin
        if (reg_re && reg_addr == 7'd1)
          uninitialised <= `D 'b0;
        if (update_complete)
          begin
            interrupt_pending[3] <= `D 1'b1;
            interrupt_pending[2] <= `D interrupt_pending[2] | i2c_error;
            interrupt_pending[1] <= `D interrupt_pending[1] | of_error;
            interrupt_pending[0] <= `D interrupt_pending[0] | oc_error;
          end
        else
        if (reg_re && reg_addr == 7'd31)
          interrupt_pending <= 4'b0;
      end
  end

// ------------------------------------------------------------------------------
// monitoring ADC buffer.
//
wire [15:0] mbuf_rd;
wire [15:0] mbuf_wd;
wire [4:0]  mbuf_waddr;
wire        mbuf_we;

reg [2:0]   mbuf_rp;
reg [2:0]   mbuf_wp;
wire        mbuf_full = (mbuf_wp + 3'd1) == mbuf_rp;
assign      mbuf_empty = mbuf_wp == mbuf_rp;

SB_RAM256x16 madc_buffer (.RCLK(clk),
                          .RCLKE(1'b1),
                          .WCLK(clk),
                          .WCLKE(1'b1),

                          .RADDR({mbuf_rp, reg_addr[4:0]}),
                          .RE(reg_re && reg_addr[6:5] == 2'b01),
                          .RDATA(mbuf_rd),

                          .WADDR({mbuf_wp, mbuf_waddr}),
                          .WDATA(mbuf_wd),
                          .WE(mbuf_we),
                          .MASK()
                          );

always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      begin
        mbuf_rp <= `D 3'b0;
        mbuf_wp <= `D 3'b0;
        mbuf_overrun <= `D 'b0;
      end
    else
      begin
        if (update_complete) 
          begin
            if (reg_re && reg_addr == 7'd31)
              mbuf_overrun <= `D 'b0;

            if (!mbuf_full || (reg_re && reg_addr == 7'b0111100))
              mbuf_wp <= `D mbuf_wp + 3'd1;
            else
              mbuf_overrun <= `D 1'b1;
          end

        if (reg_re && reg_addr == 7'b0111100)
          begin
            if (!mbuf_empty || update_complete)
              mbuf_rp <= `D mbuf_rp + 3'd1;
          end
      end
  end

// ------------------------------------------------------------------------------
// final selection of slave read data.. chooses between the flop based hreg_rd
// and the sample data buffer ram.
//
reg        reg_src_sel;
wire [7:0] reg_rd;

// sample the MSB of the address at every read in order
// to select between the buffer ram and the local registers.
//
always @(`CLOCK_RESET_SENS)
  begin
    if (reset == `RESET_LEVEL)
      reg_src_sel <= `D 'b0;
    else
    if (reg_re)
      reg_src_sel <= `D reg_addr[6];
  end

wire mbuf_rd_byte = (reg_addr[0] == big_endian)?mbuf_rd[7:0]:mbuf_rd[15:8];      
assign reg_rd = (reg_src_sel)?mbuf_rd_byte:hreg_rd;

`else // !HAVE_ADC_POLLING
wire [7:0] reg_rd;
assign     reg_rd = hreg_rd;
`endif

// ------------------------------------------------------------------------------
// SPI controller
//
spi_slave spi (
               .clk(clk),
               .reset(reset),

               // outside world interface
               .spi_clk(spi_clk),
               .spi_en(~spi_en_bar),
               .spi_din(spi_din),
               .spi_dout(spi_dout_i),
               .spi_dout_en(spi_dout_en),

               // register reading and writing
               .reg_we(reg_we),
               .reg_re(reg_re),
               .reg_addr(reg_addr),
               .reg_rd(reg_rd),
               .reg_wd(reg_wd)
               );

`ifdef HAVE_ADC_POLLING
// ------------------------------------------------------------------------------
// ADC poller
//
adc_poller adcp (
                 .reset(reset),
                 .clk(clk),
                 .i2c_phase(i2c_phase),
                 .ten_hz(ten_hz),
                 .trig1(trig1),
                 .trig2(trig2),
                 .vadc_config(vadc_config),
                 .iadc_config(iadc_config),
                 .oc_ref(oc_ref),
                 .ph_oc_ref(ph_oc_ref),
                 .mbuf_we(mbuf_we),
                 .mbuf_wd(mbuf_wd),
                 .mbuf_waddr(mbuf_waddr),
                 .update_complete(update_complete),
                 .i2c_error(i2c_error),
                 .oc_error(oc_error),
                 .of_error(of_error),
                 .scl_en(scl_en),
                 .scl_in(scl_in),
                 .sda_en(sda_en),
                 .sda_in(sda_in)
                 );
`endif

// ------------------------------------------------------------------------------
// PSU clock generator
//
`ifndef SYNC_HACK
sync_gen sync (
               .clk(clk),
               .reset(reset),
               .clk16(clk16),
               .ph1(reg_sync[0]),
               .ph2(reg_sync[1]),
               .ph3(reg_sync[2]),
               .ph4(reg_sync[3])
               );
`else
assign reg_sync = 4'hf;
`endif

endmodule

// ------------------------------------------------------------------------------
`ifdef TEST
module test;

reg clk;
reg reset;
reg clk16;

wire scl;
wire sda;
reg  sda_en;

pullup scl_pup (scl);
pullup sda_pup (sda);

assign sda = (sda_en)?'b0:'bz;

initial
  begin
    clk = 0;
    reset = `RESET_LEVEL;
    clk16 = 1;
    #50;
    reset = ~reset;
  end

always
  begin
    if (clk16)
      clk = #31.25 ~clk;
    else
      clk = #62.5 ~clk;
  end

integer   bcnt;
reg       got_stop;
reg [7:0] din;
reg       prev_scl;
reg       prev_sda;

initial
  begin
    prev_scl = 1'b0;
    prev_sda = 1'b0;
    sda_en = 0;
  end

always @(`CLOCK_SENS)
  begin
    prev_scl <= `D scl;
    prev_sda <= `D sda;
  end

reg [6:0] address;
reg       rw;

task byte_in;
  begin
    din = 0;
    got_stop = 0;

    for (bcnt = 0; !got_stop && bcnt < 8; bcnt = bcnt + 1)
      begin
        while (prev_scl || ~scl)
          @(`CLOCK_SENS);
        //$display($time, ": sample: %1d", sda);
        din <= {din[6:0], sda};
        @(`CLOCK_SENS);

        while (scl && !got_stop)
          begin
            if (!prev_sda && sda && scl)
              begin
                got_stop = 1;
                $display($time, ": stop");
              end
            @(`CLOCK_SENS);
          end
      end
  end
endtask

task cmd;
  begin
    address = -1;
    rw = 1;

    byte_in;
    if (!got_stop)
      begin
        address = din[7:1];
        rw = din[0];
      end
  end
endtask;

task nack;
  begin
    while (prev_scl || ~scl)
      @(`CLOCK_SENS);
    $display($time, ": %sack", (sda)?"n":"");

    while (scl && !got_stop)
      begin
        if (!prev_sda && sda)
          begin
            got_stop = 1;
            $display($time, ": stop");
          end
        @(`CLOCK_SENS);
      end
  end
endtask;

task ack;
  begin
    sda_en = 1;
    nack;
    sda_en = 0;
  end
endtask;

always @(`CLOCK_SENS)
  begin
    while (reset == `RESET_LEVEL)
      @(`CLOCK_SENS);

    got_stop = 0;
    cmd;
    if (!got_stop)
      begin
        $display($time, ": cmd received: address: %02x, op: %s", address, (rw)?"read":"write");
        if (address == 'h14)
          begin
            ack;
            byte_in;
            $display($time, ": byte: %02x", din);
            ack;
            byte_in;
            $display($time, ": byte: %02x", din);
            ack;
          end
        else
          nack;
      end
  end
  
top dut(.clk(clk), 
        .reset_in(reset),
        .reg_sync(),
        .pgood(4'b1010),
        .reg_en(),
        .sin_uc(1'b1),
        .sin_up(1'b1),
        .sin_down(1'b1),
        .sin(4'hf),
        .sout_uc(),
        .sout_down(),
        .sout_up(),
        .sout(),
        .pll_bypass(),
        .asic_reset_n(),
        .baud_rate(),
        .spi_clk(1'b1),
        .spi_en_bar(),
        .spi_din(1'b1),
        .spi_dout(),
        .scl(scl),
        .sda(sda),
  `ifdef BLINKY
        .blinky(),
  `endif
        .config_sw(),
        .tacho(),
        .fpga_initd()
        );


endmodule
`endif
