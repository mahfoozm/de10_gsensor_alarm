/*
 * -----------------------------------------------------------------------------
 * spi_controller.v
 * -----------------------------------------------------------------------------
 *   This module manages SPI communication to and from the on board ADXL345
 *   accelerometer sensor. It periodically initiates SPI transactions to
 *   read X-axis acceleration data at the specified update frequency. The module
 *   handles the serialization and deserialization of data, manages SPI state
 *   transitions, and provides synchronized data update signals to indicate when
 *   new data is available.
 *
 * NOTE: https://www.ece.ucdavis.edu/~bbaas/180/tutorials/accelerometer.html was
 * used as a reference, however, the module itself was implemented from the ground
 * up, implementing only the functionality that is required to read x axis data
 * from the ADXL345 sensor.
 * -----------------------------------------------------------------------------
 */

module spi_controller #(
    parameter SPI_CLK_FREQ = 2_000_000,  // SPI clock frequency
    parameter UPDATE_FREQ  = 50          // data update frequency
) (
    // host side signals
    input reset_n,     // reset
    input clk,         // system clock for non-SPI logic (in phase with spi_clk)
    input spi_clk,     // SPI clock for internal logic (in phase with clk)
    input spi_clk_out, // SPI clock output (phase shifted 270 degrees)

    // data interface
    output        data_update,  // pulse indicating new data is available
    output [15:0] data_x,       // output data from accelerometer

    // SPI interface
    output SPI_SDI,  // SPI serial data input to slave (MOSI)
    input  SPI_SDO,  // SPI serial data output from slave (MISO)
    output SPI_CSN,  // SPI chip select (active low)
    output SPI_CLK   // SPI clock output
);

  // ============================================================================
  // Signal Declarations
  // ============================================================================

  // number of SPI clock cycles between updates (SPI_CLK_FREQ / UPDATE_FREQ)
  localparam integer TIMECOUNT = SPI_CLK_FREQ / UPDATE_FREQ;  // 40,000

  // state machine states for SPI Control
  // CTRL_IDLE = 0, CTRL_TRANSFER = 1, CTRL_INTERACT = 2
  reg [1:0] spi_state;  // current state of SPI control state machine

  // state machine states for SPI serializer/deserializer
  // SERDES_IDLE = 0, SERDES_WRITE = 1, SERDES_READ = 2, SERDES_STALL = 3
  reg [1:0] serdes_state;  // current state of SPI SerDes state machine

  // SPI transaction signals
  reg start;  // start signal for SPI transaction
  reg [15:0] data_tx;  // data to transmit over SPI
  reg [1:0] read_index;  // index for read commands (0 to 1)
  reg [7:0] read_command;  // current read command
  reg data_update_internal;  // internal data update signal
  reg [3:0] serdes_count;  // bit counter for SPI transfer
  reg [15:0] data_tx_reg;  // register for data to transmit (shifted)
  reg serdes_read;  // flag indicating read operation
  reg [7:0] data_rx;  // data received from SPI
  wire spi_active;  // SPI active flag
  wire done;  // SPI transaction done flag

  // data storage
  reg [7:0] data_storage[1:0];  // storage for received data (X-axis LSB and MSB)
  assign data_x = {data_storage[1], data_storage[0]};  // concatenate MSB and LSB

  // sample timer
  reg  [15:0] sample_count;  // 16-bit counter for sampling interval
  wire        sample;  // signal indicating it's time to sample

  // clock domain crossing for data_update
  reg  [ 1:0] data_update_shift;  // shift register for data_update synchronization

  // ============================================================================
  // SPI Interface Assignments
  // ============================================================================

  // SPI active when in READ state
  assign spi_active = (serdes_state == 2'd2) || (serdes_state == 2'd1); // SERDES_READ or SERDES_WRITE

  // SPI chip select (active low)
  assign SPI_CSN = ~(spi_active || start);

  // SPI clock Output
  assign SPI_CLK = spi_active ? spi_clk_out : 1'b1;

  // SPI serial data input (MOSI)
  assign SPI_SDI = (serdes_state == 2'd1) ? data_tx_reg[serdes_count] : 1'b1;  // SERDES_WRITE

  // SPI transaction done signal
  assign done = (serdes_state == 2'd3);  // SERDES_STALL

  // data update pulse (synchronized to clk)
  assign data_update = (data_update_shift == 2'b01);

  // ============================================================================
  // Sample Timer
  // ============================================================================

  // generate sample signal every 'TIMECOUNT' SPI clock cycles to control update frequency
  assign sample = (sample_count == (TIMECOUNT - 1));
  always @(posedge spi_clk or negedge reset_n) begin
    if (!reset_n) begin
      sample_count <= 0;
    end else begin
      if (sample) begin
        sample_count <= 0;
      end else begin
        sample_count <= sample_count + 1'b1;
      end
    end
  end

  // ============================================================================
  // Read Commands, define read commands to read data from accelerometer
  // ============================================================================

  // reading X-axis data registers (LSB and MSB)
  always @(*) begin
    case (read_index)
      0: read_command = 8'b10_110010;  // read X-axis LSB (READ_MODE + DATA_X0)
      1: read_command = 8'b10_110011;  // read X-axis MSB (READ_MODE + DATA_X1)
      default: read_command = 8'b10_000000;  // no op (READ_MODE + 8'h00)
    endcase
  end

  // ============================================================================
  // SPI Control State Machine
  // ============================================================================

  always @(posedge spi_clk or negedge reset_n) begin
    if (!reset_n) begin
      // reset state
      start                <= 1'b0;
      spi_state            <= 2'd0;  // CTRL_IDLE
      read_index           <= 0;
      data_update_internal <= 1'b0;
    end else begin
      // normal operation: periodic data reading
      case (spi_state)
        2'd0: begin  // CTRL_IDLE
          // wait for sample interval
          data_update_internal <= 1'b0;
          read_index           <= 0;
          start                <= 1'b0;
          if (sample) begin
            spi_state <= 2'd2;  // CTRL_INTERACT
          end
        end
        2'd2: begin  // CTRL_INTERACT
          // prepare for read command
          data_tx[15:8] <= read_command;  // load read command into data_tx
          data_tx[7:0]  <= 8'h00;  // no data for read op
          if (read_index > 0) begin
            // store received data from previous read
            data_storage[read_index-1] <= data_rx;
          end
          start     <= 1'b1;  // initiate SPI transaction
          spi_state <= 2'd1;  // CTRL_TRANSFER
        end
        2'd1: begin  // CTRL_TRANSFER
          if (done) begin
            start <= 1'b0;
            if (read_index == 2) begin  // all reads completed
              data_update_internal <= 1'b1;
              spi_state            <= 2'd0;  // return to CTRL_IDLE
            end else begin
              // move to next read command
              read_index <= read_index + 1;
              spi_state  <= 2'd2;  // CTRL_INTERACT
            end
          end
        end
        default: spi_state <= 2'd0;  // default to CTRL_IDLE
      endcase
    end
  end

  // ============================================================================
  // SPI SerDes State Machine
  // ============================================================================

  always @(posedge spi_clk or negedge reset_n) begin
    if (!reset_n) begin
      serdes_state <= 2'd0;  // SERDES_IDLE
    end else begin
      case (serdes_state)
        2'd0: begin  // SERDES_IDLE
          serdes_count <= 4'hF;  // start from bit 15
          if (start) begin
            // start SPI transaction
            serdes_read  <= data_tx[15];  // determine if read op
            data_tx_reg  <= data_tx;  // load data to transmit
            serdes_state <= 2'd1;  // SERDES_WRITE
          end
        end
        2'd1: begin  // SERDES_WRITE
          // transmit data (MOSI)
          serdes_count <= serdes_count - 1;
          if (serdes_read && (serdes_count == 4'h8)) begin
            // switch to read state after sending address (for read op)
            serdes_state <= 2'd2;  // SERDES_READ
          end else if (serdes_count == 4'h0) begin
            serdes_state <= 2'd3;  // SERDES_STALL
          end
        end
        2'd2: begin  // SERDES_READ
          // receive data (MISO)
          serdes_count <= serdes_count - 1;
          data_rx <= {data_rx[6:0], SPI_SDO};  // shift in received bit
          if (serdes_count == 4'h0) begin
            // read op done
            serdes_state <= 2'd3;  // SERDES_STALL
          end
        end
        2'd3: begin  // SERDES_STALL
          // stall for one cycle to assert done signal
          serdes_state <= 2'd0;  // SERDES_IDLE
        end
      endcase
    end
  end

  // synchronize data_update_internal to clk
  always @(posedge clk) begin
    data_update_shift <= {data_update_shift[0], data_update_internal};
  end

endmodule
