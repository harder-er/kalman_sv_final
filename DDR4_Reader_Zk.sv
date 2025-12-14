module DDR4_Reader_Zk #(
    parameter int  MEASURE_DIM    = 6,
    parameter int  MAX_ITERATIONS = 100,
    parameter logic [31:0] ADDR_ZK_BASE = 32'h0070_0000,
    parameter int  PREFETCH_DEPTH = 4
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             start_read,
    input  logic             request_next_zk,

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

    output logic [63:0]      Z_k_out [MEASURE_DIM-1:0],
    output logic             Z_k_valid_out,
    output logic             all_Z_k_read
);

  // fixed AXI attributes
  assign axi_arlen   = 8'd0;
  assign axi_arsize  = 3'b110; // 64B
  assign axi_arburst = 2'b01;  // INCR

  // each Zk occupies MEASURE_DIM*8 bytes; round up to 64B stride
  localparam int ZK_BYTES  = MEASURE_DIM * 8;
  localparam int ZK_STRIDE = ((ZK_BYTES + 63) / 64) * 64;

  // FIFO for prefetch (store full 512-bit beat)
  logic [511:0] fifo_mem [PREFETCH_DEPTH-1:0];
  int unsigned  fifo_wptr, fifo_rptr, fifo_count;

  int unsigned  fetch_idx;     // next Zk index to fetch from DDR
  int unsigned  consume_idx;   // number of Zk popped to output

  typedef enum logic [1:0] {IDLE, AR, WAIT_R} fsm_t;
  fsm_t st;

  logic running;

  // pop -> output one cycle pulse
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (int i=0;i<MEASURE_DIM;i++) Z_k_out[i] <= '0;
      Z_k_valid_out <= 1'b0;
      all_Z_k_read  <= 1'b0;
      fifo_wptr     <= 0;
      fifo_rptr     <= 0;
      fifo_count    <= 0;
      fetch_idx     <= 0;
      consume_idx   <= 0;
      axi_arvalid   <= 1'b0;
      axi_araddr    <= 32'h0;
      axi_rready    <= 1'b0;
      st            <= IDLE;
      running       <= 1'b0;
    end else begin
      Z_k_valid_out <= 1'b0;  // pulse
      axi_rready    <= 1'b0;

      // start control
      if (!running) begin
        if (start_read) begin
          running     <= 1'b1;
          all_Z_k_read<= 1'b0;
          fetch_idx   <= 0;
          consume_idx <= 0;
          fifo_wptr   <= 0;
          fifo_rptr   <= 0;
          fifo_count  <= 0;
          axi_arvalid <= 1'b0;
          st          <= IDLE;
        end
      end else begin
        // pop on request_next_zk if FIFO not empty
        if (request_next_zk && (fifo_count > 0)) begin
          logic [511:0] word;
          word = fifo_mem[fifo_rptr];

          for (int i = 0; i < MEASURE_DIM; i++) begin
            Z_k_out[i] <= word[i*64 +: 64];
          end
          Z_k_valid_out <= 1'b1;

          fifo_rptr   <= (fifo_rptr + 1) % PREFETCH_DEPTH;
          fifo_count  <= fifo_count - 1;
          consume_idx <= consume_idx + 1;

          if (consume_idx + 1 >= MAX_ITERATIONS) begin
            all_Z_k_read <= 1'b1;
          end
        end

        // stop condition: consumed all
        if (all_Z_k_read) begin
          // keep running until start_read deasserted
          if (!start_read) running <= 1'b0;
        end
      end

      // ==========================================
      // Prefetch FSM: keep fetching while
      //  - running
      //  - fifo not full
      //  - fetch_idx < MAX_ITERATIONS
      // ==========================================
      if (running && !all_Z_k_read) begin
        case (st)
          IDLE: begin
            if ((fifo_count < PREFETCH_DEPTH) && (fetch_idx < MAX_ITERATIONS)) begin
              axi_araddr  <= ADDR_ZK_BASE + fetch_idx * ZK_STRIDE;
              axi_arvalid <= 1'b1;   // hold until arready
              st          <= AR;
            end
          end

          AR: begin
            if (axi_arvalid && axi_arready) begin
              axi_arvalid <= 1'b0;
              st          <= WAIT_R;
            end
          end

          WAIT_R: begin
            // accept read data when rvalid
            axi_rready <= 1'b1;
            if (axi_rvalid) begin
              // push into FIFO
              if (fifo_count < PREFETCH_DEPTH) begin
                fifo_mem[fifo_wptr] <= axi_rdata;
                fifo_wptr  <= (fifo_wptr + 1) % PREFETCH_DEPTH;
                fifo_count <= fifo_count + 1;
                fetch_idx  <= fetch_idx + 1;
              end
              st <= IDLE;
            end
          end

          default: st <= IDLE;
        endcase
      end else begin
        // not running or done
        axi_arvalid <= 1'b0;
        st <= IDLE;
      end
    end
  end

endmodule
