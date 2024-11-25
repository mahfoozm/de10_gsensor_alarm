module Team5_EECS3216_Project (
    input wire CLOCK_50,                  // 50 MHz clock
    input wire [1:0] KEY,                 // freeze button
    input wire [1:0] GSENSOR_INT,         // accelerometer interrupt
    inout wire GSENSOR_SDI,               // SPI input
    inout wire GSENSOR_SDO,               // SPI output
    output wire GSENSOR_CS_N,             // SPI chip select
    output wire GSENSOR_SCLK,             // SPI clock
    output wire [7:0] HEX0,               // tenths decimal digit
    output wire [7:0] HEX1,               // ones digit
    output wire [7:0] HEX2,               // tens digit
    output wire [7:0] HEX3,               // sign digit (-)
    output wire [7:0] HEX4,               // turned off
    output wire [7:0] HEX5,               // turned off
    output reg [9:0] LEDR,                // LEDs for tilt angle
    output reg buzzer                     // buzzer output
);

    wire reset_n;
    wire freeze;
    assign reset_n = KEY[1];    // zero LEDs and display
    assign freeze  = ~KEY[0];   // freeze button (save state of LEDs and display)

    wire clk;          // 25 MHz clock
    wire spi_clk;      // 2 MHz SPI clock
    wire spi_clk_out;  // 2 MHz SPI clock  (270 degree phase shift)

    // accelerometer data
    wire [15:0] data_x;
    wire [15:0] data_y;
    wire data_update;

    // angle calculation
    reg signed [19:0] angle_scaled;              // angle value (0 to 999)
    reg [3:0] tens, ones, tenths;                // BCD digits
    reg sign;                                    // sign flag (1 for negative, 0 for positive)

    assign HEX5 = 8'b11111111; // set to blank
    assign HEX4 = 8'b11111111; // set to blank

    // instantiate PLL IP to generate required clocks
    pll pll_inst (
        .inclk0(CLOCK_50),
        .c0(clk),             // 25 MHz clock
        .c1(spi_clk),         // 2 MHz SPI clock
        .c2(spi_clk_out)      // 2 MHz SPI clock  (270 degree phase shift)
    );
    
    // instantiate spi_control IP
    spi_control #(
        .SPI_CLK_FREQ  (2_000_000),
        .UPDATE_FREQ   (50)
    ) spi_ctrl (
        .reset_n    (reset_n),
        .clk        (clk),
        .spi_clk    (spi_clk),
        .spi_clk_out(spi_clk_out),
        .data_update(data_update),
        .data_x     (data_x),
        .data_y     (data_y),
        .SPI_SDI    (GSENSOR_SDI),
        .SPI_SDO    (GSENSOR_SDO),
        .SPI_CSN    (GSENSOR_CS_N),
        .SPI_CLK    (GSENSOR_SCLK),
        .interrupt  (GSENSOR_INT)
    );

    // convert raw g-sensor data to angle
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            angle_scaled <= 0;
        end else if (!freeze) begin
            angle_scaled <= (900 * $signed(data_x)) / 256;
        end
        // when freeze is active, hold angle_scaled
    end
    
    // convert angle to BCD for 7-segment display
    always @(angle_scaled) begin
        integer temp;
        if (angle_scaled < 0) begin
            temp = -angle_scaled; // abs value
            sign = 1;
        end else begin
            temp = angle_scaled;
            sign = 0;
        end

        tens = temp / 100;
        temp = temp % 100;
        ones = temp / 10;
        tenths = temp % 10;
    end

    // HEX display patterns
    parameter [7:0] SEG_0     = 8'b11000000;
    parameter [7:0] SEG_1     = 8'b11111001;
    parameter [7:0] SEG_2     = 8'b10100100;
    parameter [7:0] SEG_3     = 8'b10110000;
    parameter [7:0] SEG_4     = 8'b10011001;
    parameter [7:0] SEG_5     = 8'b10010010;
    parameter [7:0] SEG_6     = 8'b10000010;
    parameter [7:0] SEG_7     = 8'b11111000;
    parameter [7:0] SEG_8     = 8'b10000000;
    parameter [7:0] SEG_9     = 8'b10010000;
    parameter [7:0] SEG_MINUS = 8'b10111111;
    parameter [7:0] SEG_BLANK = 8'b11111111;

    // sign display on HEX3
    assign HEX3 = (sign == 1) ? SEG_MINUS : SEG_BLANK;

    // tens digit on HEX2
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

    // ones digit on HEX1
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

    // tenths decimal digit on HEX0
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

    // LED angle indicator
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            LEDR <= 10'b0;
        end else if (!freeze) begin
            LEDR = 10'b0;
            // right tilt (negative angles)
            if (angle_scaled < 0) begin
                integer abs_angle;
                abs_angle = -angle_scaled;
                if (abs_angle >= 160) LEDR[4] = 1;
                if (abs_angle >= 320) LEDR[3] = 1;
                if (abs_angle >= 480) LEDR[2] = 1;
                if (abs_angle >= 640) LEDR[1] = 1;
                if (abs_angle >= 800) LEDR[0] = 1;
            end
            // left tilt (positive angles)
            else if (angle_scaled > 0) begin
                if (angle_scaled >= 160) LEDR[5] = 1;
                if (angle_scaled >= 320) LEDR[6] = 1;
                if (angle_scaled >= 480) LEDR[7] = 1;
                if (angle_scaled >= 640) LEDR[8] = 1;
                if (angle_scaled >= 800) LEDR[9] = 1;
            end
        end
    end

    // buzzer activation on critical angle
    reg [23:0] buzzer_counter;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            buzzer <= 0;
            buzzer_counter <= 0;
        end else begin
            if (angle_scaled <= -800 || angle_scaled >= 800) begin // threshold at +/-80 degrees
                buzzer_counter <= buzzer_counter + 1;
                if (buzzer_counter == 24'd12_500_000) begin
                    buzzer <= ~buzzer;
                    buzzer_counter <= 0;
                end
            end else begin
                buzzer <= 0;
                buzzer_counter <= 0;
            end
        end
    end

endmodule
