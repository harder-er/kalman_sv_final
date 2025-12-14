module DDR4_Reader_InitialParams #(
    parameter int  STATE_DIM     = 12,
    parameter logic [31:0] ADDR_X00_BASE = 32'h0030_0000,
    parameter logic [31:0] ADDR_P00_BASE = 32'h0040_0000
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             start,

    // AXI Read (512-bit, single-beat)
    output logic [31:0]      axi_araddr,
    output logic [7:0]       axi_arlen,
    output logic [2:0]       axi_arsize,
    output logic [1:0]       axi_arburst,
    output logic             axi_arvalid,
    input  logic             axi_arready,

    input  logic [511:0]     axi_rdata,
    input  logic             axi_rvalid,
    output logic             axi_rready,

    output logic [63:0]      X_00 [STATE_DIM-1:0],
    output logic [63:0]      P_00 [STATE_DIM-1:0][STATE_DIM-1:0],
    output logic             params_ready
);

  // fixed AXI attributes: single-beat 512b read
  assign axi_arlen   = 8'd0;
  assign axi_arsize  = 3'b110; // log2(64B)=6
  assign axi_arburst = 2'b01;  // INCR

  localparam int X_BEATS = (STATE_DIM + 7) / 8;
  localparam int P_ELEMS = STATE_DIM * STATE_DIM;
  localparam int P_BEATS = (P_ELEMS + 7) / 8;

  typedef enum logic [2:0] {IDLE, AR_X, R_X, AR_P, R_P, DONE} state_t;
  state_t st;

  int unsigned x_beat;
  int unsigned p_beat;

  // helper: write X beat
  task automatic store_x(input int unsigned beat, input logic [511:0] data);
    for (int i = 0; i < 8; i++) begin
      int idx = beat*8 + i;
      if (idx < STATE_DIM)
        X_00[idx] <= data[i*64 +: 64];
    end
  endtask

  // helper: write P beat (row-major)
  task automatic store_p(input int unsigned beat, input logic [511:0] data);
    for (int i = 0; i < 8; i++) begin
      int lin = beat*8 + i;
      if (lin < P_ELEMS) begin
        int row = lin / STATE_DIM;
        int col = lin % STATE_DIM;
        P_00[row][col] <= data[i*64 +: 64];
      end
    end
  endtask

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st          <= IDLE;
      axi_arvalid <= 1'b0;
      axi_araddr  <= 32'h0;
      axi_rready  <= 1'b0;
      params_ready<= 1'b0;
      x_beat      <= 0;
      p_beat      <= 0;
    end else begin
      // defaults
      axi_rready <= 1'b0;

      case (st)
        IDLE: begin
          params_ready <= 1'b0;
          x_beat <= 0;
          p_beat <= 0;
          if (start) begin
            axi_araddr  <= ADDR_X00_BASE;
            axi_arvalid <= 1'b1;      // keep until arready
            st          <= AR_X;
          end
        end

        AR_X: begin
          if (axi_arvalid && axi_arready) begin
            axi_arvalid <= 1'b0;
            st          <= R_X;
          end
        end

        R_X: begin
          axi_rready <= 1'b1;
          if (axi_rvalid) begin
            store_x(x_beat, axi_rdata);

            if (x_beat + 1 >= X_BEATS) begin
              // move to P
              axi_araddr  <= ADDR_P00_BASE;
              axi_arvalid <= 1'b1;
              st          <= AR_P;
              p_beat      <= 0;
            end else begin
              x_beat      <= x_beat + 1;
              axi_araddr  <= ADDR_X00_BASE + (x_beat + 1) * 32'd64;
              axi_arvalid <= 1'b1;
              st          <= AR_X;
            end
          end
        end

        AR_P: begin
          if (axi_arvalid && axi_arready) begin
            axi_arvalid <= 1'b0;
            st          <= R_P;
          end
        end

        R_P: begin
          axi_rready <= 1'b1;
          if (axi_rvalid) begin
            store_p(p_beat, axi_rdata);

            if (p_beat + 1 >= P_BEATS) begin
              params_ready <= 1'b1;
              st           <= DONE;
            end else begin
              p_beat      <= p_beat + 1;
              axi_araddr  <= ADDR_P00_BASE + (p_beat + 1) * 32'd64;
              axi_arvalid <= 1'b1;
              st          <= AR_P;
            end
          end
        end

        DONE: begin
          params_ready <= 1'b1;
          if (!start) begin
            st <= IDLE;
          end
        end

        default: st <= IDLE;
      endcase
    end
  end

endmodule
