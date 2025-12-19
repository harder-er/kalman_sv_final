//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2025/04/25 19:32:51
// Design Name:
// Module Name: KalmanGainCalculator
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

// =========================================================
// Shared MUL arb (fixed number of fp_multiplier inside)
// =========================================================
module FpMulArb #(
    parameter int DWIDTH      = 64,
    parameter int NUM_CLIENTS = 1
)(
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic [NUM_CLIENTS-1:0] req_valid,
    output logic [NUM_CLIENTS-1:0] req_ready,
    input  logic [DWIDTH-1:0]      req_a [0:NUM_CLIENTS-1],
    input  logic [DWIDTH-1:0]      req_b [0:NUM_CLIENTS-1],

    output logic [NUM_CLIENTS-1:0] resp_valid,
    input  logic [NUM_CLIENTS-1:0] resp_ready,
    output logic [DWIDTH-1:0]      resp_y [0:NUM_CLIENTS-1]
);

    localparam int PTR_W = (NUM_CLIENTS <= 1) ? 1 : $clog2(NUM_CLIENTS);

    logic [PTR_W-1:0] rr_ptr;
    logic             busy;
    logic             resp_pending;
    logic [PTR_W-1:0] cur_client;

    logic [DWIDTH-1:0] a_reg, b_reg;
    logic [DWIDTH-1:0] y_reg;

    logic mul_valid_pulse;
    logic mul_finish;
    logic [DWIDTH-1:0] mul_result;

    // combinational grant
    logic grant;
    logic [PTR_W-1:0] grant_client;

    always_comb begin
        // defaults
        grant        = 1'b0;
        grant_client = rr_ptr;

        for (int c = 0; c < NUM_CLIENTS; c++) begin
            req_ready[c]  = 1'b0;
        end

        if (!busy && !resp_pending) begin
            for (int off = 0; off < NUM_CLIENTS; off++) begin
                int idx;
                idx = rr_ptr + off;
                if (idx >= NUM_CLIENTS) idx = idx - NUM_CLIENTS;

                if (!grant && req_valid[idx]) begin
                    grant        = 1'b1;
                    grant_client = idx[PTR_W-1:0];
                    req_ready[idx] = 1'b1;
                end
            end
        end
    end

    // response mux
    always_comb begin
        for (int c = 0; c < NUM_CLIENTS; c++) begin
            resp_valid[c] = 1'b0;
            resp_y[c]     = '0;
        end

        if (resp_pending) begin
            resp_valid[cur_client] = 1'b1;
            resp_y[cur_client]     = y_reg;
        end
    end

    // multiplier instance (exactly ONE here; you can replicate this module for more)
    fp_multiplier u_mul (
        .clk    (clk),
        .valid  (mul_valid_pulse),
        .finish (mul_finish),
        .a      (a_reg),
        .b      (b_reg),
        .result (mul_result)
    );

    // sequential control
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr         <= '0;
            busy           <= 1'b0;
            resp_pending   <= 1'b0;
            cur_client     <= '0;
            a_reg          <= '0;
            b_reg          <= '0;
            y_reg          <= '0;
            mul_valid_pulse <= 1'b0;
        end else begin
            mul_valid_pulse <= 1'b0;

            // accept response
            if (resp_pending && resp_valid[cur_client] && resp_ready[cur_client]) begin
                resp_pending <= 1'b0;
            end

            // start new op
            if (grant && req_valid[grant_client] && req_ready[grant_client]) begin
                a_reg          <= req_a[grant_client];
                b_reg          <= req_b[grant_client];
                cur_client     <= grant_client;
                busy           <= 1'b1;
                mul_valid_pulse <= 1'b1;

                if (NUM_CLIENTS <= 1) begin
                    rr_ptr <= '0;
                end else begin
                    if (grant_client == (NUM_CLIENTS-1)) rr_ptr <= '0;
                    else rr_ptr <= grant_client + 1'b1;
                end
            end

            // finish
            if (busy && mul_finish) begin
                y_reg        <= mul_result;
                busy         <= 1'b0;
                resp_pending <= 1'b1;
            end
        end
    end

endmodule


module KalmanGainCalculator #(
    parameter int DWIDTH = 64
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic [DWIDTH-1:0]     delta_t,      // 时间间隔

    input  logic                  SP_Done,      // 状态预测完成
    output logic                  CKG_Done,     // 测量更新完成

    input  logic [DWIDTH-1:0]     P_k1k1 [0:12-1][0:12-1],
    input  logic [DWIDTH-1:0]     Q_k    [0:12-1][0:12-1],
    input  logic [DWIDTH-1:0]     R_k    [0:5][0:5],

    output logic [DWIDTH-1:0]     K_k    [0:12-1][0:6-1]
);

    localparam int N = 12;
    localparam int M = 6;

    // 预测协方差矩阵
    logic [DWIDTH-1:0] P_predicted    [0:N-1][0:N-1];
    // P_predicted * H^T (H 取前6列) => 12x6
    logic [DWIDTH-1:0] P_predicted_HT [0:N-1][0:M-1];

    always_comb begin
        for (int i = 0; i < N; i++) begin
            for (int j = 0; j < M; j++) begin
                P_predicted_HT[i][j] = P_predicted[i][j];
            end
        end
    end

    // =========================================================
    // 1) Time params (shared MUL)
    // =========================================================
    logic sp_done_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) sp_done_d <= 1'b0;
        else        sp_done_d <= SP_Done;
    end
    wire tp_start = SP_Done & ~sp_done_d; // edge detect

    logic [DWIDTH-1:0] delta_t2;
    logic [DWIDTH-1:0] dt2;
    logic [DWIDTH-1:0] half_dt2, three2_dt2;
    logic [DWIDTH-1:0] half_dt3, three_dt3, sixth_dt3, two3_dt3;
    logic [DWIDTH-1:0] quarter_dt4, sixth_dt4, twive_dt4, five12_dt4;
    logic [DWIDTH-1:0] six_dt5, twleve_dt5;
    logic [DWIDTH-1:0] thirtysix_dt6;

    logic tp_done, tp_valid;

    // TimeParamsSeq <-> shared mul signals
    logic                  tp_mul_req_valid;
    logic                  tp_mul_req_ready;
    logic [DWIDTH-1:0]     tp_mul_req_a;
    logic [DWIDTH-1:0]     tp_mul_req_b;
    logic                  tp_mul_resp_valid;
    logic                  tp_mul_resp_ready;
    logic [DWIDTH-1:0]     tp_mul_resp_y;

    TimeParamsSeq #(.DWIDTH(DWIDTH)) u_timeparams (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (tp_start),
        .delta_t       (delta_t),

        .mul_req_valid (tp_mul_req_valid),
        .mul_req_ready (tp_mul_req_ready),
        .mul_req_a     (tp_mul_req_a),
        .mul_req_b     (tp_mul_req_b),

        .mul_resp_valid(tp_mul_resp_valid),
        .mul_resp_ready(tp_mul_resp_ready),
        .mul_resp_y    (tp_mul_resp_y),

        .delta_t2      (delta_t2),
        .dt2           (dt2),

        .half_dt2      (half_dt2),
        .three2_dt2    (three2_dt2),

        .half_dt3      (half_dt3),
        .three_dt3     (three_dt3),
        .sixth_dt3     (sixth_dt3),
        .two3_dt3      (two3_dt3),

        .quarter_dt4   (quarter_dt4),
        .sixth_dt4     (sixth_dt4),
        .twive_dt4     (twive_dt4),
        .five12_dt4    (five12_dt4),

        .six_dt5       (six_dt5),
        .twleve_dt5    (twleve_dt5),

        .thirtysix_dt6 (thirtysix_dt6),

        .done          (tp_done),
        .valid         (tp_valid)
    );

    // one shared multiplier arb (1 client for now)
    localparam int MUL_CLIENTS = 1;
    logic [MUL_CLIENTS-1:0] mul_req_valid_bus;
    logic [MUL_CLIENTS-1:0] mul_req_ready_bus;
    logic [DWIDTH-1:0]      mul_req_a_bus [0:MUL_CLIENTS-1];
    logic [DWIDTH-1:0]      mul_req_b_bus [0:MUL_CLIENTS-1];
    logic [MUL_CLIENTS-1:0] mul_resp_valid_bus;
    logic [MUL_CLIENTS-1:0] mul_resp_ready_bus;
    logic [DWIDTH-1:0]      mul_resp_y_bus [0:MUL_CLIENTS-1];

    assign mul_req_valid_bus[0]   = tp_mul_req_valid;
    assign mul_req_a_bus[0]       = tp_mul_req_a;
    assign mul_req_b_bus[0]       = tp_mul_req_b;
    assign tp_mul_req_ready       = mul_req_ready_bus[0];

    assign tp_mul_resp_valid      = mul_resp_valid_bus[0];
    assign tp_mul_resp_y          = mul_resp_y_bus[0];
    assign mul_resp_ready_bus[0]  = tp_mul_resp_ready;

    FpMulArb #(
        .DWIDTH(DWIDTH),
        .NUM_CLIENTS(MUL_CLIENTS)
    ) u_shared_mul (
        .clk       (clk),
        .rst_n     (rst_n),

        .req_valid (mul_req_valid_bus),
        .req_ready (mul_req_ready_bus),
        .req_a     (mul_req_a_bus),
        .req_b     (mul_req_b_bus),

        .resp_valid(mul_resp_valid_bus),
        .resp_ready(mul_resp_ready_bus),
        .resp_y    (mul_resp_y_bus)
    );

    // CMU 统一复位门控：time params 没算好前，CMU 全部保持 reset
    wire rst_cmu = rst_n & tp_valid;

    // =========================================================
    // 2) Matrix inverse（保持你原逻辑）
    // =========================================================
    logic [DWIDTH-1:0] inv_matrix [0:5][0:5];
    logic              inv_complete;

    MatrixInverseUnit #(.DWIDTH(DWIDTH)) u_MatrixInverseUnit (
        .clk        (clk),
        .rst_n      (rst_n),
        .P_k1k1     (P_k1k1),
        .R_k        (R_k),
        .Q_k        (Q_k),
        .valid      (SP_Done),
        .inv_matrix (inv_matrix),
        .finish     (inv_complete)
    );

    // =========================================================
    // 3) CMU 时分复用：一套 CMU + idx FSM，替代三套平铺
    // =========================================================
    localparam int CMU_ITER = 3;
    logic [1:0] cmu_idx;
    logic       cmu_all_done;

    // 当前 idx 对应的一组输入选择（直接用数组索引 + cmu_idx）
    wire [DWIDTH-1:0] t_1_1      = P_k1k1[0 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] t_4_1      = P_k1k1[3 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] t_7_1      = P_k1k1[6 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] t_4_4      = P_k1k1[3 + cmu_idx][3 + cmu_idx];
    wire [DWIDTH-1:0] t_10_1     = P_k1k1[9 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] t_4_7      = P_k1k1[3 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] t_7_7      = P_k1k1[6 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] t_4_10     = P_k1k1[3 + cmu_idx][9 + cmu_idx];
    wire [DWIDTH-1:0] t_7_10     = P_k1k1[6 + cmu_idx][9 + cmu_idx];
    wire [DWIDTH-1:0] t_10_10    = P_k1k1[9 + cmu_idx][9 + cmu_idx];

    wire [DWIDTH-1:0] t_1_4      = P_k1k1[0 + cmu_idx][3 + cmu_idx];
    wire [DWIDTH-1:0] t_1_7      = P_k1k1[0 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] t_1_10     = P_k1k1[0 + cmu_idx][9 + cmu_idx];
    wire [DWIDTH-1:0] t_10_4     = P_k1k1[9 + cmu_idx][3 + cmu_idx];
    wire [DWIDTH-1:0] t_10_7     = P_k1k1[9 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] t_7_4      = P_k1k1[6 + cmu_idx][3 + cmu_idx];

    wire [DWIDTH-1:0] q_1_1      = Q_k[0 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] q_1_4      = Q_k[0 + cmu_idx][3 + cmu_idx];
    wire [DWIDTH-1:0] q_1_7      = Q_k[0 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] q_1_10     = Q_k[0 + cmu_idx][9 + cmu_idx];
    wire [DWIDTH-1:0] q_4_1      = Q_k[3 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] q_4_4      = Q_k[3 + cmu_idx][3 + cmu_idx];
    wire [DWIDTH-1:0] q_4_7      = Q_k[3 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] q_4_10     = Q_k[3 + cmu_idx][9 + cmu_idx];
    wire [DWIDTH-1:0] q_7_1      = Q_k[6 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] q_7_4      = Q_k[6 + cmu_idx][3 + cmu_idx];
    wire [DWIDTH-1:0] q_7_7      = Q_k[6 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] q_7_10     = Q_k[6 + cmu_idx][9 + cmu_idx];
    wire [DWIDTH-1:0] q_10_1     = Q_k[9 + cmu_idx][0 + cmu_idx];
    wire [DWIDTH-1:0] q_10_4     = Q_k[9 + cmu_idx][3 + cmu_idx];
    wire [DWIDTH-1:0] q_10_7     = Q_k[9 + cmu_idx][6 + cmu_idx];
    wire [DWIDTH-1:0] q_10_10    = Q_k[9 + cmu_idx][9 + cmu_idx];

    // 3.1 单套 CMU 实例
    logic [DWIDTH-1:0] phi11_a, phi12_a, phi13_a, phi14_a;
    logic [DWIDTH-1:0] phi21_a, phi22_a, phi23_a, phi24_a;
    logic [DWIDTH-1:0] phi31_a, phi32_a, phi33_a, phi34_a;
    logic [DWIDTH-1:0] phi41_a, phi42_a, phi43_a, phi44_a;
    logic v_phi11, v_phi12, v_phi13, v_phi14;
    logic v_phi21, v_phi22, v_phi23, v_phi24;
    logic v_phi31, v_phi32, v_phi33, v_phi34;
    logic v_phi41, v_phi42, v_phi43, v_phi44;

    CMU_PHi11 #(.DBL_WIDTH(64)) u_CMU_PHi11 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_1   (t_1_1),
        .Theta_4_1   (t_4_1),
        .Theta_7_1   (t_7_1),
        .Theta_4_4   (t_4_4),
        .Theta_10_1  (t_10_1),
        .Theta_4_7   (t_4_7),
        .Theta_7_7   (t_7_7),
        .Theta_4_10  (t_4_10),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_1       (q_1_1),
        .delta_t1    (delta_t2),
        .delta_t2    (dt2),
        .delta_t3    (three_dt3),
        .delta_t4    (twive_dt4),
        .delta_t5    (six_dt5),
        .delta_t6    (thirtysix_dt6),
        .a           (phi11_a),
        .valid_out   (v_phi11)
    );

    CMU_PHi12 #(.DBL_WIDTH(64)) u_CMU_PHi12 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_4   (t_1_4),
        .Theta_1_1   (t_1_1),
        .Theta_1_7   (t_1_7),
        .Theta_4_7   (t_4_7),
        .Theta_1_10  (t_1_10),
        .Theta_4_10  (t_4_10),
        .Theta_7_7   (t_7_7),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_4       (q_1_4),
        .delta_t     (delta_t),
        .half_dt2    (half_dt2),
        .sixth_dt3   (sixth_dt3),
        .five12_dt4  (five12_dt4),
        .twleve_dt5  (twleve_dt5),
        .a           (phi12_a),
        .valid_out   (v_phi12)
    );

    CMU_PHi13 #(.DBL_WIDTH(64)) u_CMU_PHi13 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_7   (t_1_7),
        .Theta_4_7   (t_4_7),
        .Theta_1_10  (t_1_10),
        .Theta_7_7   (t_7_7),
        .Theta_4_10  (t_4_10),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_7       (q_1_7),
        .delta_t     (delta_t),
        .half_dt2    (half_dt2),
        .two3_dt3    (two3_dt3),
        .sixth_dt4   (sixth_dt4),
        .a           (phi13_a),
        .valid_out   (v_phi13)
    );

    CMU_PHi14 #(.DBL_WIDTH(64)) u_CMU_PHi14 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_10  (t_1_10),
        .Theta_4_10  (t_4_10),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_10      (q_1_10),
        .delta_t     (delta_t),
        .half_dt2    (half_dt2),
        .two3_dt3    (two3_dt3),
        .a           (phi14_a),
        .valid_out   (v_phi14)
    );

    CMU_PHi21 #(.DBL_WIDTH(64)) u_CMU_PHi21 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_1    (t_4_1),
        .Theta_7_1    (t_7_1),
        .Theta_4_4    (t_4_4),
        .Theta_1_10   (t_1_10),
        .Theta_4_7    (t_4_7),
        .Theta_7_7    (t_7_7),
        .Theta_4_10   (t_4_10),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_4_1        (q_4_1),
        .delta_t        (delta_t),
        .dt2_half       (half_dt2),
        .dt3_sixth      (sixth_dt3),
        .dt4_twelth     (twive_dt4),
        .dt5_twelth     (twleve_dt5),
        .dt6_thirtysix  (thirtysix_dt6),
        .a            (phi21_a),
        .valid_out    (v_phi21)
    );

    CMU_PHi22 #(.DBL_WIDTH(64)) u_CMU_PHi22 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_4    (t_4_4),
        .Theta_4_7    (t_4_7),
        .Theta_4_10   (t_4_10),
        .Theta_7_7    (t_7_7),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_4_4        (q_4_4),
        .two_dt       (delta_t2),
        .dt2          (dt2),
        .half_dt3     (half_dt3),
        .quarter_dt4  (quarter_dt4),
        .a            (phi22_a),
        .valid_out    (v_phi22)
    );

    CMU_PHi23 #(.DBL_WIDTH(64)) u_CMU_PHi23 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_7    (t_4_7),
        .Theta_7_7    (t_7_7),
        .Theta_4_10   (t_4_10),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_4_7        (q_4_7),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .half_dt3     (half_dt3),
        .a            (phi23_a),
        .valid_out    (v_phi23)
    );

    CMU_PHi24 #(.DBL_WIDTH(64)) u_CMU_PHi24 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_10   (t_4_10),
        .Theta_7_4    (t_7_4),
        .Theta_10_10  (t_10_10),
        .Q_4_10       (q_4_10),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .a            (phi24_a),
        .valid_out    (v_phi24)
    );

    CMU_PHi31 #(.DBL_WIDTH(64)) u_CMU_PHi31 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_1    (t_7_1),
        .Theta_1_10   (t_1_10),
        .Theta_4_7    (t_4_7),
        .Theta_7_7    (t_7_7),
        .Theta_4_10   (t_4_10),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_1        (q_7_1),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .two3_dt3     (two3_dt3),
        .sixth_dt4    (sixth_dt4),
        .a            (phi31_a),
        .valid_out    (v_phi31)
    );

    CMU_PHi32 #(.DBL_WIDTH(64)) u_CMU_PHi32 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_4    (t_7_4),
        .Theta_4_10   (t_4_10),
        .Theta_7_7    (t_7_7),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_4        (q_7_4),
        .delta_t      (delta_t),
        .three2_dt2   (three2_dt2),
        .half_dt3     (half_dt3),
        .a            (phi32_a),
        .valid_out    (v_phi32)
    );

    CMU_PHi33 #(.DBL_WIDTH(64)) u_CMU_PHi33 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_7    (t_7_7),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_7        (q_7_7),
        .two_dt       (delta_t2),
        .dt2          (dt2),
        .a            (phi33_a),
        .valid_out    (v_phi33)
    );

    CMU_PHi34 #(.DBL_WIDTH(64)) u_CMU_PHi34 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_10       (q_7_10),
        .delta_t      (delta_t),
        .a            (phi34_a),
        .valid_out    (v_phi34)
    );

    CMU_PHi41 #(.DBL_WIDTH(64)) u_CMU_PHi41 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_1   (t_10_1),
        .Theta_10_4   (t_10_4),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_10_1       (q_10_1),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .sixth_dt3    (sixth_dt3),
        .a            (phi41_a),
        .valid_out    (v_phi41)
    );

    CMU_PHi42 #(.DBL_WIDTH(64)) u_CMU_PHi42 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_4   (t_10_4),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_10_4       (q_10_4),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .a            (phi42_a),
        .valid_out    (v_phi42)
    );

    CMU_PHi43 #(.DBL_WIDTH(64)) u_CMU_PHi43 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_10_7       (q_10_7),
        .delta_t      (delta_t),
        .a            (phi43_a),
        .valid_out    (v_phi43)
    );

    CMU_PHi44 #(.DBL_WIDTH(64)) u_CMU_PHi44 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_10  (t_10_10),
        .Q_10_10      (q_10_10),
        .a            (phi44_a),
        .valid_out    (v_phi44)
    );

    // 3.2 结果收集 + idx FSM
    logic d_phi11, d_phi12, d_phi13, d_phi14;
    logic d_phi21, d_phi22, d_phi23, d_phi24;
    logic d_phi31, d_phi32, d_phi33, d_phi34;
    logic d_phi41, d_phi42, d_phi43, d_phi44;

    wire cmu_round_done = d_phi11 & d_phi12 & d_phi13 & d_phi14 &
                          d_phi21 & d_phi22 & d_phi23 & d_phi24 &
                          d_phi31 & d_phi32 & d_phi33 & d_phi34 &
                          d_phi41 & d_phi42 & d_phi43 & d_phi44;

    integer r, c;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (r = 0; r < N; r++) begin
                for (c = 0; c < N; c++) begin
                    P_predicted[r][c] <= '0;
                end
            end
            cmu_idx      <= '0;
            cmu_all_done <= 1'b0;
            {d_phi11, d_phi12, d_phi13, d_phi14,
             d_phi21, d_phi22, d_phi23, d_phi24,
             d_phi31, d_phi32, d_phi33, d_phi34,
             d_phi41, d_phi42, d_phi43, d_phi44} <= '0;
        end else begin
            if (tp_start) begin
                cmu_idx      <= '0;
                cmu_all_done <= 1'b0;
                {d_phi11, d_phi12, d_phi13, d_phi14,
                 d_phi21, d_phi22, d_phi23, d_phi24,
                 d_phi31, d_phi32, d_phi33, d_phi34,
                 d_phi41, d_phi42, d_phi43, d_phi44} <= '0;
            end

            if (rst_cmu && !cmu_all_done) begin
                if (!d_phi11 && v_phi11) begin
                    P_predicted[0 + cmu_idx][0 + cmu_idx] <= phi11_a;
                    d_phi11 <= 1'b1;
                end
                if (!d_phi12 && v_phi12) begin
                    P_predicted[0 + cmu_idx][1 + cmu_idx] <= phi12_a;
                    d_phi12 <= 1'b1;
                end
                if (!d_phi13 && v_phi13) begin
                    P_predicted[0 + cmu_idx][2 + cmu_idx] <= phi13_a;
                    d_phi13 <= 1'b1;
                end
                if (!d_phi14 && v_phi14) begin
                    P_predicted[0 + cmu_idx][9 + cmu_idx] <= phi14_a;
                    d_phi14 <= 1'b1;
                end

                if (!d_phi21 && v_phi21) begin
                    P_predicted[3 + cmu_idx][0 + cmu_idx] <= phi21_a;
                    d_phi21 <= 1'b1;
                end
                if (!d_phi22 && v_phi22) begin
                    P_predicted[3 + cmu_idx][3 + cmu_idx] <= phi22_a;
                    d_phi22 <= 1'b1;
                end
                if (!d_phi23 && v_phi23) begin
                    P_predicted[3 + cmu_idx][6 + cmu_idx] <= phi23_a;
                    d_phi23 <= 1'b1;
                end
                if (!d_phi24 && v_phi24) begin
                    P_predicted[3 + cmu_idx][9 + cmu_idx] <= phi24_a;
                    d_phi24 <= 1'b1;
                end

                if (!d_phi31 && v_phi31) begin
                    P_predicted[6 + cmu_idx][0 + cmu_idx] <= phi31_a;
                    d_phi31 <= 1'b1;
                end
                if (!d_phi32 && v_phi32) begin
                    P_predicted[6 + cmu_idx][3 + cmu_idx] <= phi32_a;
                    d_phi32 <= 1'b1;
                end
                if (!d_phi33 && v_phi33) begin
                    P_predicted[6 + cmu_idx][6 + cmu_idx] <= phi33_a;
                    d_phi33 <= 1'b1;
                end
                if (!d_phi34 && v_phi34) begin
                    P_predicted[6 + cmu_idx][9 + cmu_idx] <= phi34_a;
                    d_phi34 <= 1'b1;
                end

                if (!d_phi41 && v_phi41) begin
                    P_predicted[9 + cmu_idx][0 + cmu_idx] <= phi41_a;
                    d_phi41 <= 1'b1;
                end
                if (!d_phi42 && v_phi42) begin
                    P_predicted[9 + cmu_idx][3 + cmu_idx] <= phi42_a;
                    d_phi42 <= 1'b1;
                end
                if (!d_phi43 && v_phi43) begin
                    P_predicted[9 + cmu_idx][6 + cmu_idx] <= phi43_a;
                    d_phi43 <= 1'b1;
                end
                if (!d_phi44 && v_phi44) begin
                    P_predicted[9 + cmu_idx][9 + cmu_idx] <= phi44_a;
                    d_phi44 <= 1'b1;
                end

                if (cmu_round_done) begin
                    if (cmu_idx == CMU_ITER-1) begin
                        cmu_all_done <= 1'b1;
                    end else begin
                        cmu_idx <= cmu_idx + 1'b1;
                    end
                    {d_phi11, d_phi12, d_phi13, d_phi14,
                     d_phi21, d_phi22, d_phi23, d_phi24,
                     d_phi31, d_phi32, d_phi33, d_phi34,
                     d_phi41, d_phi42, d_phi43, d_phi44} <= '0;
                end
            end
        end
    end

    // =========================================================
    // 4) expand inv_matrix -> 12x12
    // =========================================================
    logic [DWIDTH-1:0] inv_matrix12 [0:11][0:11];
    generate
        for (genvar r = 0; r < 12; r++) begin : GEN_INV12_R
            for (genvar c = 0; c < 12; c++) begin : GEN_INV12_C
                assign inv_matrix12[r][c] = (r < 6 && c < 6) ? inv_matrix[r][c] : {DWIDTH{1'b0}};
            end
        end
    endgenerate

    logic [DWIDTH-1:0] P_predicted_HT12 [0:11][0:11];
    generate
        for (genvar r = 0; r < 12; r++) begin : GEN_PHT12_R
            for (genvar c = 0; c < 12; c++) begin : GEN_PHT12_C
                assign P_predicted_HT12[r][c] = (c < 6) ? P_predicted_HT[r][c] : {DWIDTH{1'b0}};
            end
        end
    endgenerate

    // =========================================================
    // 5) start systolic after inv_complete & tp_valid
    //    (FIX: systolic_fired single always_ff driver)
    // =========================================================
    logic inv_lat;
    logic systolic_fired;

    wire systolic_start_pulse = inv_lat & tp_valid & cmu_all_done & ~systolic_fired;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inv_lat        <= 1'b0;
            systolic_fired <= 1'b0;
        end else begin
            if (tp_start) begin
                inv_lat        <= 1'b0;
                systolic_fired <= 1'b0;
            end

            if (inv_complete) begin
                inv_lat <= 1'b1;
            end

            if (systolic_start_pulse) begin
                systolic_fired <= 1'b1;
            end
        end
    end

    // =========================================================
    // 6) 串行 MAC：K = P_predicted_HT12(12x6) * inv_matrix12(6x6)
    //     单乘法器 + 单加法器，三重循环 row/col/k 逐项累加
    // =========================================================
    logic [DWIDTH-1:0] K_k_matrix [0:11][0:11];
    logic              K_done;

    typedef enum logic [1:0] {K_IDLE, K_MUL, K_ADD, K_STORE} k_state_e;
    k_state_e k_state;

    logic [3:0] row_idx;
    logic [2:0] col_idx;
    logic [2:0] k_idx;
    logic [DWIDTH-1:0] acc_reg;

    // 乘法/加法请求脉冲
    logic mul_go, add_go;
    logic [DWIDTH-1:0] mul_a, mul_b;
    logic [DWIDTH-1:0] add_a, add_b;
    logic mul_finish;
    logic add_finish;
    logic [DWIDTH-1:0] mul_result;
    logic [DWIDTH-1:0] add_result;

    fp_multiplier u_kmul (
        .clk    (clk),
        .valid  (mul_go),
        .finish (mul_finish),
        .a      (mul_a),
        .b      (mul_b),
        .result (mul_result)
    );

    fp_adder u_kadd (
        .clk    (clk),
        .valid  (add_go),
        .finish (add_finish),
        .a      (add_a),
        .b      (add_b),
        .result (add_result)
    );

    integer rr, cc;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (rr = 0; rr < 12; rr++) begin
                for (cc = 0; cc < 12; cc++) begin
                    K_k_matrix[rr][cc] <= '0;
                end
            end
            k_state  <= K_IDLE;
            row_idx  <= '0;
            col_idx  <= '0;
            k_idx    <= '0;
            acc_reg  <= '0;
            mul_go   <= 1'b0;
            add_go   <= 1'b0;
            K_done   <= 1'b0;
        end else begin
            // 默认拉低脉冲
            mul_go <= 1'b0;
            add_go <= 1'b0;
            K_done <= 1'b0;

            if (tp_start) begin
                for (rr = 0; rr < 12; rr++) begin
                    for (cc = 0; cc < 12; cc++) begin
                        K_k_matrix[rr][cc] <= '0;
                    end
                end
                k_state <= K_IDLE;
                row_idx <= '0;
                col_idx <= '0;
                k_idx   <= '0;
                acc_reg <= '0;
            end

            case (k_state)
                K_IDLE: begin
                    if (systolic_start_pulse) begin
                        row_idx <= '0;
                        col_idx <= '0;
                        k_idx   <= '0;
                        acc_reg <= '0;
                        mul_a   <= P_predicted_HT12[0][0];
                        mul_b   <= inv_matrix12[0][0];
                        mul_go  <= 1'b1;
                        k_state <= K_MUL;
                    end
                end

                K_MUL: begin
                    if (mul_finish) begin
                        add_a  <= acc_reg;
                        add_b  <= mul_result;
                        add_go <= 1'b1;
                        k_state <= K_ADD;
                    end
                end

                K_ADD: begin
                    if (add_finish) begin
                        acc_reg <= add_result;
                        if (k_idx == 3'd5) begin
                            k_state <= K_STORE;
                        end else begin
                            k_idx   <= k_idx + 1'b1;
                            mul_a   <= P_predicted_HT12[row_idx][k_idx + 1'b1];
                            mul_b   <= inv_matrix12[k_idx + 1'b1][col_idx];
                            mul_go  <= 1'b1;
                            k_state <= K_MUL;
                        end
                    end
                end

                K_STORE: begin
                    K_k_matrix[row_idx][col_idx] <= acc_reg;
                    acc_reg <= '0;
                    k_idx   <= '0;

                    if (col_idx == 3'd5) begin
                        col_idx <= '0;
                        if (row_idx == 4'd11) begin
                            K_done  <= 1'b1;
                            k_state <= K_IDLE;
                        end else begin
                            row_idx <= row_idx + 1'b1;
                            // 启动下一行/第 0 列乘法
                            mul_a   <= P_predicted_HT12[row_idx + 1'b1][0];
                            mul_b   <= inv_matrix12[0][0];
                            mul_go  <= 1'b1;
                            k_state <= K_MUL;
                        end
                    end else begin
                        col_idx <= col_idx + 1'b1;
                        // 启动同一行下一列乘法
                        mul_a   <= P_predicted_HT12[row_idx][0];
                        mul_b   <= inv_matrix12[0][col_idx + 1'b1];
                        mul_go  <= 1'b1;
                        k_state <= K_MUL;
                    end
                end
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            CKG_Done <= 1'b0;
        end else begin
            if (tp_start) CKG_Done <= 1'b0;
            else if (K_done) CKG_Done <= 1'b1;
        end
    end

    generate
        for (genvar r = 0; r < 12; r++) begin : GEN_K_OUT_R
            for (genvar c = 0; c < 6; c++) begin : GEN_K_OUT_C
                assign K_k[r][c] = K_k_matrix[r][c];
            end
        end
    endgenerate

endmodule
