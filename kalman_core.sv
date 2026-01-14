`timescale 1ns / 1ps

module kalman_core #(
    parameter int  STATE_DIM  = 12,
    parameter int  MEASURE_DIM= 6,
    parameter real deltat     = 0.01,

    // End_valid：最后一次迭代完成后，再延迟 N 个周期才拉高
    parameter int  END_VALID_STABLE_CYCLES = 50
)(
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         start,          // ★建议接 Top �?filter_start_pulse�? 拍）

    input  logic [63:0]                  Q_k [STATE_DIM-1:0][STATE_DIM-1:0],
    input  logic [63:0]                  R_k [MEASURE_DIM-1:0][MEASURE_DIM-1:0],

    input  logic [63:0]                  Z_k [MEASURE_DIM-1:0],
    input  logic                         En_MDI,         // ★来自 Top 的 mdi_valid_hold
    input  logic [63:0]                  X_00 [STATE_DIM-1:0],
    input  logic [63:0]                  P_00 [STATE_DIM-1:0][STATE_DIM-1:0],

    input  logic                         all_Z_k_read,

    output logic                         SP_Done,        // ★新增：状态预测完成信?   
     output logic                         iter_done_pulse,

    output logic [63:0]                  X_kkout [STATE_DIM-1:0],
    output logic [63:0]                  P_kkout [STATE_DIM-1:0][STATE_DIM-1:0],
    output logic                         filter_done,
    
    // ===== AXI Master 2 : Write Output (from StateCovarainceOutput) =====
    output logic [31:0]   m2_axi_awaddr,
    output logic [7:0]    m2_axi_awlen,
    output logic [2:0]    m2_axi_awsize,
    output logic [1:0]    m2_axi_awburst,
    output logic          m2_axi_awvalid,
    input  logic          m2_axi_awready,
    
    output logic [511:0]  m2_axi_wdata,
    output logic [63:0]   m2_axi_wstrb,
    output logic          m2_axi_wvalid,
    input  logic          m2_axi_wready,
    output logic          m2_axi_wlast,
    
    input  logic [1:0]    m2_axi_bresp,
    input  logic          m2_axi_bvalid,
    output logic          m2_axi_bready
);

    // ---------------- internal ----------------
    logic [63:0] X_k1k [STATE_DIM-1:0];
    logic [63:0] X_kk1 [STATE_DIM-1:0];
    logic [63:0] X_kk  [STATE_DIM-1:0];

    logic [63:0] P_kk1  [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [63:0] P_kk   [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [63:0] P_k1k1 [STATE_DIM-1:0][STATE_DIM-1:0];

    logic [63:0] K_k [STATE_DIM-1:0][MEASURE_DIM-1:0];
    logic [63:0] F   [STATE_DIM-1:0][STATE_DIM-1:0];

    // handshake signals to CU
    logic Init_Valid;
    logic SCU_Done_s;
    logic SCU_Done_p;
    logic CKG_Done;
    logic SCO_Valid;
    logic MDI_Valid;
    logic End_valid;

    logic en_init, en_sp, en_ckg, en_scu, en_sco, finish;

    // MDI valid from Top (hold)
    assign MDI_Valid = En_MDI;

    // SCO_Valid from StateCovarainceOutput (will replace hardcoded 0)
    logic sco_done_signal;
    logic [7:0] iteration_cnt_sco;
    logic all_iter_done;
    assign SCO_Valid = sco_done_signal;
    
    // AXI write address/data signals from StateCovarainceOutput
    logic [31:0]   sco_axi_awaddr;
    logic [7:0]    sco_axi_awlen;
    logic [2:0]    sco_axi_awsize;
    logic [1:0]    sco_axi_awburst;
    logic          sco_axi_awvalid;
    logic          sco_axi_awready;
    logic [511:0]  sco_axi_wdata;
    logic [63:0]   sco_axi_wstrb;
    logic          sco_axi_wvalid;
    logic          sco_axi_wready;
    logic          sco_axi_wlast;
    logic [1:0]    sco_axi_bresp;
    logic          sco_axi_bvalid;
    logic          sco_axi_bready;
    
    // Connect AXI outputs directly from StateCovarainceOutput
    assign m2_axi_awaddr   = sco_axi_awaddr;
    assign m2_axi_awlen    = sco_axi_awlen;
    assign m2_axi_awsize   = sco_axi_awsize;
    assign m2_axi_awburst  = sco_axi_awburst;
    assign m2_axi_awvalid  = sco_axi_awvalid;
    assign sco_axi_awready = m2_axi_awready;
    assign m2_axi_wdata    = sco_axi_wdata;
    assign m2_axi_wstrb    = sco_axi_wstrb;
    assign m2_axi_wvalid   = sco_axi_wvalid;
    assign sco_axi_wready  = m2_axi_wready;
    assign m2_axi_wlast    = sco_axi_wlast;
    assign sco_axi_bresp   = m2_axi_bresp;
    assign sco_axi_bvalid  = m2_axi_bvalid;
    assign m2_axi_bready   = sco_axi_bready;

    // filter_done
    assign filter_done = finish;

    // output export
    genvar gi, gj;
    generate
        for (gi = 0; gi < STATE_DIM; gi++) begin : GEN_OUT_X
            assign X_kkout[gi] = X_kk[gi];
        end
        for (gi = 0; gi < STATE_DIM; gi++) begin : GEN_OUT_P_I
            for (gj = 0; gj < STATE_DIM; gj++) begin : GEN_OUT_P_J
                assign P_kkout[gi][gj] = P_kk[gi][gj];
            end
        end
    endgenerate

    // -------------------------------------------------
    // Init_Valid：start 后延�?10 周期，在 en_init=1 时拉�?    // -------------------------------------------------
    localparam int INIT_DELAY_CYCLES = 10;
    localparam int ICW = (INIT_DELAY_CYCLES < 1) ? 1 : $clog2(INIT_DELAY_CYCLES + 1);
    localparam logic [ICW-1:0] INIT_DELAY_MINUS1 = ICW'((INIT_DELAY_CYCLES <= 0) ? 0 : (INIT_DELAY_CYCLES - 1));

    logic start_d;
    logic start_seen;
    logic [ICW-1:0] init_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_d    <= 1'b0;
            start_seen <= 1'b0;
            init_cnt   <= '0;
            Init_Valid <= 1'b0;
        end else begin
            start_d <= start;

            if (start && !start_d)
                start_seen <= 1'b1;

            if (!en_init) begin
                init_cnt   <= '0;
                Init_Valid <= 1'b0;
            end else begin
                if (start_seen && !Init_Valid) begin
                    if (INIT_DELAY_CYCLES == 0)
                        Init_Valid <= 1'b1;
                    else if (init_cnt == INIT_DELAY_MINUS1)
                        Init_Valid <= 1'b1;
                    else
                        init_cnt <= init_cnt + 1'b1;
                end
            end

            if (finish) begin
                start_seen <= 1'b0;
                init_cnt   <= '0;
                Init_Valid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------
    // iter_done_pulse：一次迭代完成（SCU 两路 done 同时到达）上升沿打一�?    // -------------------------------------------------
    logic scu_done_all_d;
    wire  scu_done_all = (SCU_Done_s & SCU_Done_p);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scu_done_all_d  <= 1'b0;
            iter_done_pulse <= 1'b0;
        end else begin
            scu_done_all_d  <= scu_done_all;
            iter_done_pulse <= scu_done_all & ~scu_done_all_d;
        end
    end

    // -------------------------------------------------
    // End_valid：★只在“最后一次迭代完成后”启动延时计�?    // 条件：iter_done_pulse && all_Z_k_read
    // -------------------------------------------------
    localparam int EWC = 1;
    logic end_valid_d;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            End_valid <= 1'b0;
            end_valid_d <= 1'b0;
        end else begin
            end_valid_d <= all_iter_done;
            
            if (finish) begin
                End_valid <= 1'b0;
            end else if (all_iter_done && !end_valid_d) begin
                // Rising edge of all_iter_done: trigger End_valid
                End_valid <= 1'b1;
            end
        end
    end

    // -------------------------------------------------
    // 控制单元（start �?core �?start：即 filter_start_pulse�?    // -------------------------------------------------
    KF_ControlUnit u_KF_ControlUnit (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),

        .Init_Valid (Init_Valid),
        .SP_Done    (SP_Done),
        .SCU_Done_s (SCU_Done_s),
        .SCU_Done_p (SCU_Done_p),
        .CKG_Done   (CKG_Done),
        .SCO_Valid  (SCO_Valid),
        .MDI_Valid  (MDI_Valid),
        .End_valid  (End_valid),

        .en_init    (en_init),
        .en_sp      (en_sp),
        .en_ckg     (en_ckg),
        .en_scu     (en_scu),
        .en_sco     (en_sco),
        .finish     (finish)
    );

    // -------------------------------------------------
    // F matrix
    // -------------------------------------------------
    logic F_finish;
    F_make u_Fmake (
        .clk    (clk),
        .rst_n  (rst_n),
        .finish (F_finish),
        .deltat (deltat),
        .F      (F)
    );

    // -------------------------------------------------
    // ★关键：模块�?done/enable 接线修正
    // -------------------------------------------------
    // StatePredictor：在 S_SP 运行，输�?SP_Done
    StatePredictor #(
        .VEC_WIDTH(64),
        .MAT_DIM  (12)
    ) u_StatePredictor (
        .clk        (clk),
        .rst_n      (rst_n),
        .K_k        (K_k),
        .Z_k        (Z_k),
        .X_kk1      (X_kk1),
        .X_kk       (X_kk),
        .Init_Valid (en_sp),     // ★用 en_sp 作为 SP 阶段使能
        .SP_DONE    (SP_Done)
    );

    // KalmanGainCalculator：用 SP_Done 启动，输�?CKG_Done
    KalmanGainCalculator #(
        .DWIDTH(64)
    ) u_KalmanGainCalc (
        .clk      (clk),
        .rst_n    (rst_n),
        .delta_t  (deltat),

        .SP_Done  (SP_Done),     // ★必须接真正�?SP_Done
        .CKG_Done (CKG_Done),

        .P_k1k1   (P_k1k1),
        .Q_k      (Q_k),
        .R_k      (R_k),

        .K_k      (K_k)
    );

    // StateUpdate：用 CKG_Done 启动，输�?SCU_Done_s
    StateUpdate u_StateUpdator (
        .clk       (clk),
        .rst_n     (rst_n),
        .F         (F),
        .X_kk      (X_kk),
        .X_k1k     (X_k1k),
        .CKG_Done  (CKG_Done),   // ★接真正 CKG_Done
        .MDI_Valid (MDI_Valid),
        .SCU_Done  (SCU_Done_s)
    );

    // CovarianceUpdate：用 CKG_Done 启动，输�?SCU_Done_p
    CovarianceUpdate #(
        .STATE_DIM(STATE_DIM),
        .DWIDTH   (64)
    ) u_CovUpdate (
        .clk      (clk),
        .rst_n    (rst_n),
        .K_k      (K_k),
        .R_k      (R_k),
        .P_kk1    (P_kk1),
        .P_kk     (P_kk),
        .CKG_Done (CKG_Done),    // ★接真正 CKG_Done
        .SCU_Done (SCU_Done_p)
    );

    // -------------------------------------------------
    // ★新增：StateCovarainceOutput - �?P_kk �?X_k1k 写入 AXI 内存
    // -------------------------------------------------
    StateCovarainceOutput #(
        .STATE_DIM(STATE_DIM),
        .MEASURE_DIM(MEASURE_DIM),
        .DATA_WIDTH(64),
        .MAX_ITER(50)
    ) u_SCO_Output (
        .clk(clk),
        .rst_n(rst_n),
        .en_sco(en_sco),           // From FSM: S_SCO state enable
        .P_kk(P_kk),               // Covariance matrix to write
        .X_k1k(X_k1k),             // State vector to write
        .axi_awaddr(sco_axi_awaddr),
        .axi_awlen(sco_axi_awlen),
        .axi_awsize(sco_axi_awsize),
        .axi_awburst(sco_axi_awburst),
        .axi_awvalid(sco_axi_awvalid),
        .axi_awready(sco_axi_awready),
        .axi_wdata(sco_axi_wdata),
        .axi_wstrb(sco_axi_wstrb),
        .axi_wvalid(sco_axi_wvalid),
        .axi_wready(sco_axi_wready),
        .axi_wlast(sco_axi_wlast),
        .axi_bresp(sco_axi_bresp),
        .axi_bvalid(sco_axi_bvalid),
        .axi_bready(sco_axi_bready),
        .sco_done(sco_done_signal),
        .iteration_out(iteration_cnt_sco),
        .all_done(all_iter_done)
    );

    // -------------------------------------------------
    // Delay units（保持你原结构）
    // -------------------------------------------------
    logic [63:0] Xk1k_delay [STATE_DIM-1:0][0:0];
    logic [63:0] Xkk1_delay [STATE_DIM-1:0][0:0];

    generate
        for (gi = 0; gi < STATE_DIM; gi++) begin : GEN_XK1K_IN
            assign Xk1k_delay[gi][0] = X_k1k[gi];
        end
        for (gi = 0; gi < STATE_DIM; gi++) begin : GEN_XKK1_OUT
            assign X_kk1[gi] = Xkk1_delay[gi][0];
        end
    endgenerate

    DelayUnit #(
        .DELAY_CYCLES(1),
        .ROWS       (12),
        .COLS       (1),
        .DATA_WIDTH (64)
    ) u_DelayX (
        .clk      (clk),
        .rst_n    (rst_n),
        .data_in  (Xk1k_delay),
        .data_out (Xkk1_delay)
    );

    DelayUnit #(
        .DELAY_CYCLES(1),
        .ROWS       (12),
        .COLS       (12),
        .DATA_WIDTH (64)
    ) u_DelayP (
        .clk      (clk),
        .rst_n    (rst_n),
        .data_in  (P_kk),
        .data_out (P_k1k1)
    );

endmodule
