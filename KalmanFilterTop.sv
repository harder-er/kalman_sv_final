module KalmanFilterTop #(
    parameter int  STATE_DIM      = 12,
    parameter int  MEASURE_DIM    = 6,
    parameter real deltat         = 0.01,
    parameter int  MAX_ITERATIONS = 100
) (
    // system
    input  logic clk,
    input  logic rst_n,

    // control
    input  logic start,

    // ===== AXI Master 0 : Initial Reader (Read Only) =====
    output logic [31:0]  m0_axi_araddr,
    output logic [7:0]   m0_axi_arlen,
    output logic [2:0]   m0_axi_arsize,
    output logic [1:0]   m0_axi_arburst,
    output logic         m0_axi_arvalid,
    input  logic         m0_axi_arready,
    input  logic [511:0] m0_axi_rdata,
    input  logic         m0_axi_rvalid,
    output logic         m0_axi_rready,

    // ===== AXI Master 1 : Zk Reader (Read Only, Prefetch) =====
    output logic [31:0]  m1_axi_araddr,
    output logic [7:0]   m1_axi_arlen,
    output logic [2:0]   m1_axi_arsize,
    output logic [1:0]   m1_axi_arburst,
    output logic         m1_axi_arvalid,
    input  logic         m1_axi_arready,
    input  logic [511:0] m1_axi_rdata,
    input  logic         m1_axi_rvalid,
    output logic         m1_axi_rready,

    // ===== AXI Master 2 : Writer (Write Only) =====
    output logic [31:0]  m2_axi_awaddr,
    output logic [7:0]   m2_axi_awlen,
    output logic [2:0]   m2_axi_awsize,
    output logic [1:0]   m2_axi_awburst,
    output logic         m2_axi_awvalid,
    input  logic         m2_axi_awready,

    output logic [511:0] m2_axi_wdata,
    output logic [63:0]  m2_axi_wstrb,
    output logic         m2_axi_wvalid,
    input  logic         m2_axi_wready,
    output logic         m2_axi_wlast,

    input  logic [1:0]   m2_axi_bresp,
    input  logic         m2_axi_bvalid,
    output logic         m2_axi_bready,

    // legacy input (unused)
    input  logic [63:0]  z_data,
    input  logic         z_valid,

    // interrupt/status
    output logic filter_done
);

  // =========================
  // internal data
  // =========================
  logic [63:0] X_00 [STATE_DIM-1:0];
  logic [63:0] P_00 [STATE_DIM-1:0][STATE_DIM-1:0];

  logic [63:0] Z_k_internal [MEASURE_DIM-1:0];
  logic        Z_k_valid_internal;
  logic        Z_k_request_next;
  logic        all_Z_k_read;

  logic [63:0] Q_k_internal [STATE_DIM-1:0][STATE_DIM-1:0];
  logic [63:0] R_k_internal [MEASURE_DIM-1:0][MEASURE_DIM-1:0];

  logic [63:0] X_kk_out_internal [STATE_DIM-1:0];
  logic [63:0] P_kk_out_internal [STATE_DIM-1:0][STATE_DIM-1:0];

  logic initial_params_ready;
  logic noise_matrices_ready;

  logic filter_start;
  assign filter_start = start && initial_params_ready && noise_matrices_ready;

  // 写回：先用 filter_done 触发（后续你可加 writer_done 做更完整握手）
  logic write_results_en;
  assign write_results_en = filter_done;

  // =========================
  // Reader0: initial params
  // =========================
  DDR4_Reader_InitialParams #(
      .STATE_DIM(STATE_DIM),
      .ADDR_X00_BASE(32'h0030_0000),
      .ADDR_P00_BASE(32'h0040_0000)
  ) ddr4_reader_initial (
      .clk(clk),
      .rst_n(rst_n),
      .start(start),

      .axi_araddr (m0_axi_araddr),
      .axi_arlen  (m0_axi_arlen),
      .axi_arsize (m0_axi_arsize),
      .axi_arburst(m0_axi_arburst),
      .axi_arvalid(m0_axi_arvalid),
      .axi_arready(m0_axi_arready),
      .axi_rdata  (m0_axi_rdata),
      .axi_rvalid (m0_axi_rvalid),
      .axi_rready (m0_axi_rready),

      .X_00(X_00),
      .P_00(P_00),
      .params_ready(initial_params_ready)
  );

  // =========================
  // Noise generator
  // =========================
  NoiseGenerator #(
      .STATE_DIM(STATE_DIM),
      .MEASURE_DIM(MEASURE_DIM),
      .deltat(deltat)
  ) noise_gen (
      .clk(clk),
      .rst_n(rst_n),
      .enable_gen(start),
      .Q_k(Q_k_internal),
      .R_k(R_k_internal),
      .matrices_ready(noise_matrices_ready)
  );

  // =========================
  // Zk request pacing (simple, no comb loop)
  // - 每当上一帧 Zk 弹出(valid)后，再发下一次 request
  // - 你后续可以用 kalman_core 的“需要下一帧”信号替换
  // =========================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      Z_k_request_next <= 1'b0;
    end else begin
      if (!filter_start || filter_done || all_Z_k_read) begin
        Z_k_request_next <= 1'b0;
      end else begin
        // 如果当前没有挂起请求，就发起一次
        if (!Z_k_request_next)
          Z_k_request_next <= 1'b1;

        // 当 Zk Reader 给出一帧 valid(表示已经弹出并输出)，撤销本次请求
        if (Z_k_valid_internal)
          Z_k_request_next <= 1'b0;
      end
    end
  end

  // =========================
  // Reader1: Zk (prefetch FIFO)
  // =========================
  DDR4_Reader_Zk #(
      .MEASURE_DIM(MEASURE_DIM),
      .MAX_ITERATIONS(MAX_ITERATIONS),
      .ADDR_ZK_BASE(32'h0070_0000),
      .PREFETCH_DEPTH(4)
  ) ddr4_reader_zk (
      .clk(clk),
      .rst_n(rst_n),
      .start_read(filter_start),
      .request_next_zk(Z_k_request_next),

      .axi_araddr (m1_axi_araddr),
      .axi_arlen  (m1_axi_arlen),
      .axi_arsize (m1_axi_arsize),
      .axi_arburst(m1_axi_arburst),
      .axi_arvalid(m1_axi_arvalid),
      .axi_arready(m1_axi_arready),
      .axi_rdata  (m1_axi_rdata),
      .axi_rvalid (m1_axi_rvalid),
      .axi_rready (m1_axi_rready),

      .Z_k_out(Z_k_internal),
      .Z_k_valid_out(Z_k_valid_internal),
      .all_Z_k_read(all_Z_k_read)
  );

  // =========================
  // Writer: results
  // =========================
  DDR4_Writer #(
      .STATE_DIM(STATE_DIM),
      .ADDR_X_RESULT_BASE(32'h0050_0000),
      .ADDR_P_RESULT_BASE(32'h0060_0000)
  ) ddr4_writer (
      .clk(clk),
      .rst_n(rst_n),
      .write_en(write_results_en),

      .X_kk_in(X_kk_out_internal),
      .P_kk_in(P_kk_out_internal),

      .axi_awaddr (m2_axi_awaddr),
      .axi_awlen  (m2_axi_awlen),
      .axi_awsize (m2_axi_awsize),
      .axi_awburst(m2_axi_awburst),
      .axi_awvalid(m2_axi_awvalid),
      .axi_awready(m2_axi_awready),

      .axi_wdata  (m2_axi_wdata),
      .axi_wstrb  (m2_axi_wstrb),
      .axi_wvalid (m2_axi_wvalid),
      .axi_wready (m2_axi_wready),
      .axi_wlast  (m2_axi_wlast),

      .axi_bresp  (m2_axi_bresp),
      .axi_bvalid (m2_axi_bvalid),
      .axi_bready (m2_axi_bready)
  );

  // =========================
  // kalman core (修正 start 连接)
  // =========================
  kalman_core #(
      .STATE_DIM(STATE_DIM),
      .MEASURE_DIM(MEASURE_DIM)
  ) kalman (
      .clk         (clk),
      .rst_n       (rst_n),
      .start       (filter_start),          // ✅ 原来你是悬空 ()
      .Q_k         (Q_k_internal),
      .R_k         (R_k_internal),
      .Z_k         (Z_k_internal),
      .En_MDI      (Z_k_valid_internal),
      .X_00        (X_00),
      .P_00        (P_00),
      .filter_done (filter_done),
      .X_kkout     (X_kk_out_internal),
      .P_kkout     (P_kk_out_internal)
  );

  // unused legacy inputs
  logic unused_legacy;
  assign unused_legacy = ^{z_data, z_valid};

endmodule
