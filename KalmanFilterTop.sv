`timescale 1ns / 1ps

module KalmanFilterTop #(
    parameter int  STATE_DIM      = 12,
    parameter int  MEASURE_DIM    = 6,
    parameter real deltat         = 0.01,
    parameter int  MAX_ITERATIONS = 100,

    // ★End_valid 触发门限（由 core 内部实现：all_Z_k_read 连续�?的周期数�?    
    // parameter int  END_VALID_STABLE_CYCLES = 50
    parameter int  END_VALID_STABLE_CYCLES = 10
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

  // =========================
  // start sticky + ready handshake
  // =========================
  logic start_d;
  logic start_seen;
  logic filter_started;
  logic filter_active;

  logic filter_start_pulse;
  wire  start_rise = start & ~start_d;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      start_d        <= 1'b0;
      start_seen     <= 1'b0;
      filter_started <= 1'b0;
      filter_active  <= 1'b0;
    end else begin
      start_d <= start;

      if (start_rise)
        start_seen <= 1'b1;

      if (filter_start_pulse) begin
        filter_started <= 1'b1;
        filter_active  <= 1'b1;
      end

      if (filter_done) begin
        start_seen     <= 1'b0;
        filter_started <= 1'b0;
        filter_active  <= 1'b0;
      end
    end
  end

  assign filter_start_pulse =
      start_seen && !filter_started && initial_params_ready && noise_matrices_ready;

  wire filter_start = filter_start_pulse;

  // 写回：先�?filter_done 触发
  logic write_results_en;
  assign write_results_en = filter_done;

  // =========================================================
  // ★新增：来自 core 的关键信�?  // - SP_Done: 状态预测完�?  // - iter_done_pulse: 迭代完成脉冲（SCU_done 上升�?1 拍）
  // =========================================================
  logic core_sp_done;     // ★新增：�?kalman_core 导出�?SP_Done
  logic iter_done_pulse;

  // =========================================================
  // ★修改：MDI valid 保持型（�?SP_Done 拉高，由 iter_done_pulse 拉低�?  
  // - SP_Done 上升后拉高（确保测量数据已准备好�?  // - iter_done_pulse 到达时拉低（本次迭代消费完毕�? 
  // - 确保测量数据有效信号与迭代周期同?  // =========================================================
  logic mdi_valid_hold;
  logic sp_done_d;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mdi_valid_hold <= 1'b0;
      sp_done_d      <= 1'b0;
    end else begin
      sp_done_d <= core_sp_done;  // 延迟一拍用于边沿检�?
      if (!filter_active || filter_done) begin
        mdi_valid_hold <= 1'b0;
      end else if (filter_start_pulse) begin
        mdi_valid_hold <= 1'b0;
      end else if (iter_done_pulse) begin
        mdi_valid_hold <= 1'b0;     // 本次迭代结束，清零等待下一�?SP_Done
      end else if (core_sp_done && !sp_done_d) begin
        // SP_Done 上升沿：状态预测完成，拉高测量有效
        mdi_valid_hold <= 1'b1;
      end
    end
  end

  // =========================================================
  // ★Zk request pacing：只在“启�?迭代完成”时请求一�?  // request 是电平保持：直到 Reader 真正吐出 Z_k_valid_out
  // =========================================================
  logic zk_req_pending;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      zk_req_pending <= 1'b0;
    end else begin
      if (filter_start_pulse) begin
        // 启动时先要第一�?Zk（为�?次迭代准备）
        zk_req_pending <= 1'b1;
      end else if (!filter_active || filter_done || all_Z_k_read) begin
        zk_req_pending <= 1'b0;
      end else begin
        // 收到 Zk_valid 后清 pending（本次请求完成）
        if (Z_k_valid_internal) begin
          zk_req_pending <= 1'b0;
        end
        // 只有当一次迭代完成，才请求下一�?Zk
        else if (!zk_req_pending && iter_done_pulse) begin
          zk_req_pending <= 1'b1;
        end
      end
    end
  end

  assign Z_k_request_next = zk_req_pending;

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
      .start(start_seen),

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
      .enable_gen(start_seen),
      .Q_k(Q_k_internal),
      .R_k(R_k_internal),
      .matrices_ready(noise_matrices_ready)
  );

  // =========================
  // Reader1: Zk (prefetch FIFO)
  // - start_read �?filter_active 电平
  // - request_next_zk 用我们新�?Z_k_request_next（只在迭代完成后触发�?  // =========================
  DDR4_Reader_Zk #(
      .MEASURE_DIM(MEASURE_DIM),
      .MAX_ITERATIONS(MAX_ITERATIONS),
      .ADDR_ZK_BASE(32'h0070_0000),
      .PREFETCH_DEPTH(4)
  ) ddr4_reader_zk (
      .clk(clk),
      .rst_n(rst_n),
      .start_read(filter_active),
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
  // kalman core
  // - En_MDI 改用 mdi_valid_hold（保持型�?  // - 取回 iter_done_pulse（用于触发下一�?Zk 请求�?  // - AXI Master 2 用于输出 P_kk �?X_k1k
  // =========================
  kalman_core #(
      .STATE_DIM(STATE_DIM),
      .MEASURE_DIM(MEASURE_DIM),
      .END_VALID_STABLE_CYCLES(END_VALID_STABLE_CYCLES)
  ) kalman (
      .clk            (clk),
      .rst_n          (rst_n),
      .start          (filter_start),

      .Q_k            (Q_k_internal),
      .R_k            (R_k_internal),

      .Z_k            (Z_k_internal),
      .En_MDI         (mdi_valid_hold),     // ★保持型测量有效
      .X_00           (X_00),
      .P_00           (P_00),

      .all_Z_k_read   (all_Z_k_read),

      .SP_Done        (core_sp_done),       // ★新增：接收 SP_Done 用于控制 mdi_valid_hold
      .iter_done_pulse(iter_done_pulse),    // ★新增：迭代完成脉冲

      .filter_done    (filter_done),
      .X_kkout        (X_kk_out_internal),
      .P_kkout        (P_kk_out_internal),
      
      // AXI Master 2: Write interface from StateCovarainceOutput
      .m2_axi_awaddr  (m2_axi_awaddr),
      .m2_axi_awlen   (m2_axi_awlen),
      .m2_axi_awsize  (m2_axi_awsize),
      .m2_axi_awburst (m2_axi_awburst),
      .m2_axi_awvalid (m2_axi_awvalid),
      .m2_axi_awready (m2_axi_awready),
      .m2_axi_wdata   (m2_axi_wdata),
      .m2_axi_wstrb   (m2_axi_wstrb),
      .m2_axi_wvalid  (m2_axi_wvalid),
      .m2_axi_wready  (m2_axi_wready),
      .m2_axi_wlast   (m2_axi_wlast),
      .m2_axi_bresp   (m2_axi_bresp),
      .m2_axi_bvalid  (m2_axi_bvalid),
      .m2_axi_bready  (m2_axi_bready)
  );

endmodule
