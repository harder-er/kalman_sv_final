`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: kalman_core
// Description: 卡尔曼滤波核心
//////////////////////////////////////////////////////////////////////////////////

module kalman_core #(
    parameter int  STATE_DIM     = 12,
    parameter int  MEASURE_DIM   = 6,
    parameter real deltat        = 0.01,

    // ★新增：all_Z_k_read 连续为1多少个周期后，认为 End_valid=1
    parameter int  END_VALID_STABLE_CYCLES = 50
)(
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         start,

    // 系统模型参数
    input  logic [63:0]                  Q_k [STATE_DIM-1:0][STATE_DIM-1:0],
    input  logic [63:0]                  R_k [MEASURE_DIM-1:0][MEASURE_DIM-1:0],

    // 实时数据接口
    input  logic [63:0]                  Z_k [MEASURE_DIM-1:0],
    input  logic                         En_MDI,
    input  logic [63:0]                  X_00 [STATE_DIM-1:0],
    input  logic [63:0]                  P_00 [STATE_DIM-1:0][STATE_DIM-1:0],

    // 结束条件输入：来自 Top（Zk reader 的 all_Z_k_read）
    input  logic                         all_Z_k_read,

    // 滤波结果输出
    output logic [63:0]                  X_kkout [STATE_DIM-1:0],
    output logic [63:0]                  P_kkout [STATE_DIM-1:0][STATE_DIM-1:0],
    output logic                         filter_done
);

    // -------------------------------------------------
    // 内部信号
    // -------------------------------------------------
    logic [63:0] X_k1k [STATE_DIM-1:0];
    logic [63:0] X_kk1 [STATE_DIM-1:0];
    logic [63:0] X_kk  [STATE_DIM-1:0];

    logic [63:0] P_k1k  [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [63:0] P_kk1  [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [63:0] P_kk   [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [63:0] P_k1k1 [STATE_DIM-1:0][STATE_DIM-1:0];

    logic [63:0] K_k [STATE_DIM-1:0][MEASURE_DIM-1:0];
    logic [63:0] F   [STATE_DIM-1:0][STATE_DIM-1:0];

    // KF_ControlUnit 相关握手信号
    logic Init_Valid;
    logic SP_Done;
    logic SCU_Done_s;
    logic SCU_Done_p;
    logic CKG_Done;
    logic SCO_Valid;
    logic MDI_Valid;
    logic End_valid;

    logic en_init;
    logic en_sp;
    logic en_ckg;
    logic en_scu;
    logic en_sco;
    logic finish;

    // 其他缺失信号补齐
    logic F_finish;

    // -------------------------------------------------
    // MDI_Valid：必须确定驱动
    // -------------------------------------------------
    assign MDI_Valid = En_MDI;

    // -------------------------------------------------
    // End_valid：all_Z_k_read 连续拉高 N 周期（参数化）后置 1
    // - all_Z_k_read=1: 计数累加
    // - all_Z_k_read=0: 计数清零（要求“连续”为1）
    // - 达到门限：End_valid 拉高并保持到下一次 start
    // -------------------------------------------------
    localparam int ECW = (END_VALID_STABLE_CYCLES < 1) ? 1 : $clog2(END_VALID_STABLE_CYCLES + 1);
    localparam int unsigned END_M1_INT = (END_VALID_STABLE_CYCLES < 1) ? 0 : (END_VALID_STABLE_CYCLES - 1);
    localparam logic [ECW-1:0] END_M1 = END_M1_INT[ECW-1:0];

    logic [ECW-1:0] end_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            end_cnt   <= '0;
            End_valid <= 1'b0;
        end else begin
            // 新一轮 start（你这里是 1 拍脉冲）-> 清零
            if (start) begin
                end_cnt   <= '0;
                End_valid <= 1'b0;
            end else if (!End_valid) begin
                if (all_Z_k_read) begin
                    if (end_cnt == END_M1) begin
                        End_valid <= 1'b1;
                    end else begin
                        end_cnt <= end_cnt + {{(ECW-1){1'b0}},1'b1};
                    end
                end else begin
                    end_cnt <= '0; // ★要求连续高
                end
            end
        end
    end

    // -------------------------------------------------
    // SCO_Valid debug：assign SCO_Valid = en_sco（仅用于 debug）
    // 为避免组合环路，这里打一拍
    // -------------------------------------------------
    logic sco_valid_dbg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) sco_valid_dbg <= 1'b0;
        else        sco_valid_dbg <= en_sco;
    end
    assign SCO_Valid = sco_valid_dbg;

    // -------------------------------------------------
    // filter_done：用控制单元 finish
    // -------------------------------------------------
    assign filter_done = finish;

    // -------------------------------------------------
    // 输出结果导出
    // -------------------------------------------------
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
    // ★Init_Valid 自动拉高逻辑（保持你现有规则不变）
    // start 上升沿后延迟 10 周期，在 INIT(en_init=1) 内拉高 Init_Valid
    // 离开 INIT(en_init=0) 清零，等待下一轮 start
    // -------------------------------------------------
    localparam int INIT_DELAY_CYCLES = 10;
    localparam int ICW = (INIT_DELAY_CYCLES < 1) ? 1 : $clog2(INIT_DELAY_CYCLES + 1);
    localparam int unsigned INIT_M1_INT = (INIT_DELAY_CYCLES < 1) ? 0 : (INIT_DELAY_CYCLES - 1);
    localparam logic [ICW-1:0] INIT_DELAY_MINUS1 = INIT_M1_INT[ICW-1:0];

    logic start_d_init;
    logic start_seen_init;
    logic [ICW-1:0] init_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_d_init    <= 1'b0;
            start_seen_init <= 1'b0;
            init_cnt        <= '0;
            Init_Valid      <= 1'b0;
        end else begin
            start_d_init <= start;

            // 记录 start 上升沿
            if (start && !start_d_init)
                start_seen_init <= 1'b1;

            // 只在 INIT(en_init=1) 内工作
            if (!en_init) begin
                init_cnt   <= '0;
                Init_Valid <= 1'b0;
            end else begin
                if (start_seen_init && !Init_Valid) begin
                    if (INIT_DELAY_CYCLES == 0) begin
                        Init_Valid <= 1'b1;
                    end else if (init_cnt == INIT_DELAY_MINUS1) begin
                        Init_Valid <= 1'b1;
                    end else begin
                        init_cnt <= init_cnt + {{(ICW-1){1'b0}},1'b1};
                    end
                end

                // 可选：拉高后清 start_seen_init
                if (Init_Valid)
                    start_seen_init <= 1'b0;
            end
        end
    end

    // -------------------------------------------------
    // 控制单元（按最新状态机图）
    // -------------------------------------------------
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
    // F 矩阵生成
    // -------------------------------------------------
    F_make u_Fmake (
        .clk    (clk),
        .rst_n  (rst_n),
        .finish (F_finish),
        .deltat (deltat),
        .F      (F)
    );

    // -------------------------------------------------
    // 各阶段模块（保持你的连线风格，不做结构性改动）
    // -------------------------------------------------
    StateUpdate u_StateUpdator (
        .clk       (clk),
        .rst_n     (rst_n),
        .F         (F),
        .X_kk      (X_kk),
        .X_k1k     (X_k1k),
        .CKG_Done  (en_ckg),
        .MDI_Valid (MDI_Valid),
        .SCU_Done  (SCU_Done_s)
    );

    KalmanGainCalculator #(
        .DWIDTH(64)
    ) u_KalmanGainCalc (
        .clk      (clk),
        .rst_n    (rst_n),
        .Q_k      (Q_k),
        .delta_t  (deltat),
        .P_k1k1   (P_k1k1),
        .R_k      (R_k),
        .K_k      (K_k),
        .SP_Done  (en_sp),
        .CKG_Done (CKG_Done)
    );

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
        .Init_Valid (en_sp),     // 维持你当前用法：预测态使能
        .SP_DONE    (SP_Done)
    );

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
        .CKG_Done (en_ckg),
        .SCU_Done (SCU_Done_p)
    );

    // -------------------------------------------------
    // 延时单元（保留）
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
