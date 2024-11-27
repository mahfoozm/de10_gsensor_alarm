/*
 * -----------------------------------------------------------------------------
 * Team5_EECS3216_Project.v
 * -----------------------------------------------------------------------------
 *   This module serves as the main file for our EECS3216 project.
 *   It interfaces with the ADXL345 accelerometer sensor via SPI to obtain
 *   X-axis acceleration data. The module processes the data to calculate tilt
 *   angles, displays the results on 7-segment displays, displays tilt magnitude
 *   on the on board LEDs, and activates a buzzer when the tilt exceeds specified
 *   thresholds. It also provides a freeze feature through KEY0, freezing the
 *   values of the display and LEDs when pushed.
 * -----------------------------------------------------------------------------
 */

module Team5_EECS3216_Project (
    input  wire       CLOCK_50,      // 50 MHz clock input
    input  wire [1:0] KEY,           // push buttons (KEY[1]: reset, KEY[0]: freeze)
    inout  wire       GSENSOR_SDI,   // SPI serial data input to accelerometer (MOSI)
    inout  wire       GSENSOR_SDO,   // SPI serial data output from accelerometer (MISO)
    output wire       GSENSOR_CS_N,  // SPI chip select (active low)
    output wire       GSENSOR_SCLK,  // SPI clock output
    output wire [7:0] HEX0,          // 7-segment display for tenths digit
    output wire [7:0] HEX1,          // 7-segment display for ones digit
    output wire [7:0] HEX2,          // 7-segment display for tens digit
    output wire [7:0] HEX3,          // 7-segment display for sign ('-' or blank)
    output wire [7:0] HEX4,          // unused 7-segment display (turned off)
    output wire [7:0] HEX5,          // unused 7-segment display (turned off)
    output reg  [9:0] LEDR,          // 10 LEDs indicating tilt angle
    output reg        buzzer         // buzzer output
);

  // ============================================================================
  // Parameters and Constants
  // ============================================================================

  // 7-Segment display encoding
  parameter [7:0] SEG_0 = 8'b11000000;
  parameter [7:0] SEG_1 = 8'b11111001;
  parameter [7:0] SEG_2 = 8'b10100100;
  parameter [7:0] SEG_3 = 8'b10110000;
  parameter [7:0] SEG_4 = 8'b10011001;
  parameter [7:0] SEG_5 = 8'b10010010;
  parameter [7:0] SEG_6 = 8'b10000010;
  parameter [7:0] SEG_7 = 8'b11111000;
  parameter [7:0] SEG_8 = 8'b10000000;
  parameter [7:0] SEG_9 = 8'b10010000;
  parameter [7:0] SEG_MINUS = 8'b10111111;
  parameter [7:0] SEG_BLANK = 8'b11111111;

  // ============================================================================
  // Signal Declarations
  // ============================================================================

  // Control signals
  wire              reset_n;  // reset button signal
  wire              freeze;  // freeze button signal

  // Clock signals
  wire              clk;  // 25 MHz system clock
  wire              spi_clk;  // 2 MHz SPI clock
  wire              spi_clk_out;  // 2 MHz SPI clock with 270-degree phase shift

  // SPI data signals
  wire       [15:0] data_x;  // accelerometer X-axis data
  wire              data_update;  // data update pulse from SPI controller

  // angle calculation variables
  reg signed [19:0] angle_scaled;  // scaled angle value
  reg signed [19:0] angle_filtered;  // filtered angle value using EMA
  reg [3:0] tens, ones, tenths;  // BCD digits for display
  reg            sign;  // sign flag (1 for negative, 0 for positive)

  // buzzer control variables
  reg     [14:0] buzzer_counter;  // counter for buzzer frequency
  reg     [14:0] toggle_count;  // toggle count for buzzer frequency control
  integer        abs_angle_filtered;  // absolute value of the filtered angle

  // ============================================================================
  // Assignments and Signal Initializations
  // ============================================================================

  // reset and freeze button assignments
  assign reset_n = KEY[1];     // zero out when KEY[1] is pressed
  assign freeze  = ~KEY[0];    // freeze LEDs and display when KEY[0] is pressed

  // unused 7-segment displays turned off
  assign HEX5 = SEG_BLANK;  // HEX5 display set to blank
  assign HEX4 = SEG_BLANK;  // HEX4 display set to blank

  // ============================================================================
  // Clock Generation (PLL Instantiation)
  // ============================================================================

  // instantiate PLL IP to generate required clocks from 50 MHz input
  pll pll_inst (
      .inclk0(CLOCK_50),    // 50 MHz input clock
      .c0    (clk),         // 25 MHz system clock output
      .c1    (spi_clk),     // 2 MHz SPI clock output
      .c2    (spi_clk_out)  // 2 MHz SPI clock with 270-degree phase shift
  );

  // ============================================================================
  // SPI Controller Instantiation
  // ============================================================================

  // instantiate SPI controller to read data from accelerometer over SPI instead of I2C
  spi_controller #(
      .SPI_CLK_FREQ(2_000_000),  // SPI clock frequency in Hz
      .UPDATE_FREQ (50)          // data update frequency in Hz
  ) gsensor (
      .reset_n    (reset_n),       // active-low reset
      .clk        (clk),           // 25 MHz system clock
      .spi_clk    (spi_clk),       // 2 MHz SPI clock
      .spi_clk_out(spi_clk_out),   // 2 MHz SPI clock with phase shift
      .data_update(data_update),   // data update pulse
      .data_x     (data_x),        // 16-bit X-axis data output
      .SPI_SDI    (GSENSOR_SDI),   // SPI MOSI
      .SPI_SDO    (GSENSOR_SDO),   // SPI MISO
      .SPI_CSN    (GSENSOR_CS_N),  // SPI Chip Select (active low)
      .SPI_CLK    (GSENSOR_SCLK)   // SPI Clock Output
  );

  // ============================================================================
  // Angle Calculation and Filtering
  // ============================================================================

  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      angle_scaled   <= 0;
      angle_filtered <= 0;
    end else if (!freeze) begin
      angle_scaled   <= (900 * $signed(data_x)) / 256;
      // Apply Exponential Moving Average (EMA) filter with alpha = 0.25
      angle_filtered <= (angle_scaled >>> 2) + (angle_filtered - (angle_filtered >>> 2));
    end
    // when freeze is active, hold angle_scaled and angle_filtered
  end

  // ============================================================================
  // BCD Conversion for 7-Segment Display
  // ============================================================================

  // convert the filtered angle to BCD digits for display
  always @(*) begin
    integer temp;
    if (angle_filtered < 0) begin
      temp = -angle_filtered;  // take absolute value
      sign = 1;  // set sign flag for negative
    end else begin
      temp = angle_filtered;
      sign = 0;  // set sign flag for positive
    end

    // extract tens, ones, and tenths digits
    tens   = temp / 100;
    temp   = temp % 100;
    ones   = temp / 10;
    tenths = temp % 10;
  end

  // ============================================================================
  // Display Outputs (assignments to HEX displays)
  // ============================================================================

  // display sign on HEX3
  assign HEX3 = (sign == 1) ? SEG_MINUS : SEG_BLANK;

  // display tens digit on HEX2
  assign HEX2 = (tens == 4'd0) ? SEG_0 :
                  (tens == 4'd1) ? SEG_1 :
                  (tens == 4'd2) ? SEG_2 :
                  (tens == 4'd3) ? SEG_3 :
                  (tens == 4'd4) ? SEG_4 :
                  (tens == 4'd5) ? SEG_5 :
                  (tens == 4'd6) ? SEG_6 :
                  (tens == 4'd7) ? SEG_7 :
                  (tens == 4'd8) ? SEG_8 :
                  (tens == 4'd9) ? SEG_9 : SEG_BLANK;

  // display ones digit on HEX1 with decimal point ON
  assign HEX1 = ((ones == 4'd0) ? SEG_0 :
                   (ones == 4'd1) ? SEG_1 :
                   (ones == 4'd2) ? SEG_2 :
                   (ones == 4'd3) ? SEG_3 :
                   (ones == 4'd4) ? SEG_4 :
                   (ones == 4'd5) ? SEG_5 :
                   (ones == 4'd6) ? SEG_6 :
                   (ones == 4'd7) ? SEG_7 :
                   (ones == 4'd8) ? SEG_8 :
                   (ones == 4'd9) ? SEG_9 : SEG_BLANK) & 8'b01111111; // decimal point ON

  // display tenths digit on HEX0
  assign HEX0 = (tenths == 4'd0) ? SEG_0 :
                  (tenths == 4'd1) ? SEG_1 :
                  (tenths == 4'd2) ? SEG_2 :
                  (tenths == 4'd3) ? SEG_3 :
                  (tenths == 4'd4) ? SEG_4 :
                  (tenths == 4'd5) ? SEG_5 :
                  (tenths == 4'd6) ? SEG_6 :
                  (tenths == 4'd7) ? SEG_7 :
                  (tenths == 4'd8) ? SEG_8 :
                  (tenths == 4'd9) ? SEG_9 : SEG_BLANK;

  // ============================================================================
  // LED Angle Indicator, lights up LEDs based on the magnitude of the angle
  // ============================================================================

  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      LEDR <= 10'b0;
    end else if (!freeze) begin
      LEDR = 10'b0;  // Clear LEDs
      // right tilt (negative angles)
      if (angle_filtered < 0) begin
        integer abs_angle;
        abs_angle = -angle_filtered;
        if (abs_angle >= 160) LEDR[4] = 1;
        if (abs_angle >= 320) LEDR[3] = 1;
        if (abs_angle >= 480) LEDR[2] = 1;
        if (abs_angle >= 640) LEDR[1] = 1;
        if (abs_angle >= 800) LEDR[0] = 1;
      end  // left tilt (positive angles)
      else if (angle_filtered > 0) begin
        if (angle_filtered >= 160) LEDR[5] = 1;
        if (angle_filtered >= 320) LEDR[6] = 1;
        if (angle_filtered >= 480) LEDR[7] = 1;
        if (angle_filtered >= 640) LEDR[8] = 1;
        if (angle_filtered >= 800) LEDR[9] = 1;
      end
    end
    // when freeze is active, hold the current state of LEDs
  end

  // ============================================================================
  // Buzzer Control, controls buzzer frequency based on the angle
  // ============================================================================

  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      buzzer         <= 0;
      buzzer_counter <= 0;
    end else begin
      abs_angle_filtered = (angle_filtered < 0) ? -angle_filtered : angle_filtered;

      if (abs_angle_filtered >= 400) begin  // start beeping at +/-40 degrees
        if (abs_angle_filtered < 500) begin
          toggle_count = 12500;  // 1 kHz
        end else if (abs_angle_filtered < 600) begin
          toggle_count = 10000;  // 1.25 kHz
        end else if (abs_angle_filtered < 700) begin
          toggle_count = 8333;  // 1.5 kHz
        end else if (abs_angle_filtered < 800) begin
          toggle_count = 7143;  // 1.75 kHz
        end else if (abs_angle_filtered < 900) begin
          toggle_count = 6250;  // 2 kHz
        end else begin
          toggle_count = 5000;  // 2.5 kHz
        end

        buzzer_counter <= buzzer_counter + 1;
        if (buzzer_counter >= toggle_count) begin
          buzzer_counter <= 0;
          buzzer         <= ~buzzer;
        end
      end else begin
        buzzer         <= 0;
        buzzer_counter <= 0;
      end
    end
  end

endmodule
