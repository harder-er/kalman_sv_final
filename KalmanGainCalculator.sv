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
    // 1) Time params：顺序复用
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

    TimeParamsSeq #(.DWIDTH(DWIDTH)) u_timeparams (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (tp_start),
        .delta_t       (delta_t),

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

    // CMU 统一复位门控：time params 没算好前，CMU 全部保持 reset
    wire rst_cmu = rst_n & tp_valid;

    // =========================================================
    // 2) Matrix inverse（保持原逻辑）
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
    // 3) CMU 并行块（保持你原来的 generate，只把 rst_n 换成 rst_cmu）
    //    下面 phi*_valid 信号如果你没用到也没关系，声明出来防止编译报错
    // =========================================================
    logic phi11_valid;
    logic phi12_valid_minus1, phi13_valid_minus1, phi14_valid_minus1;
    logic phi21_valid_minus1, phi22_valid_minus1, phi23_valid_minus1, phi24_valid_minus1;
    logic phi31_valid_minus1, phi32_valid_minus1, phi33_valid_minus1, phi34_valid_minus1;
    logic phi41_valid_minus1, phi42_valid_minus1, phi43_valid_minus1, phi44_valid_minus1;

    generate
        genvar i;

        for (i = 0; i < 3; i++) begin : GEN_PHI11
            CMU_PHi11 #(.DBL_WIDTH(64)) u_CMU_PHi11 (
                .clk         (clk),
                .rst_n       (rst_cmu),

                .Theta_1_1   (P_k1k1[0+i][0+i]),
                .Theta_4_1   (P_k1k1[3+i][0+i]),
                .Theta_7_1   (P_k1k1[6+i][0+i]),
                .Theta_4_4   (P_k1k1[3+i][3+i]),
                .Theta_10_1  (P_k1k1[9+i][0+i]),
                .Theta_4_7   (P_k1k1[3+i][6+i]),
                .Theta_7_7   (P_k1k1[6+i][6+i]),
                .Theta_4_10  (P_k1k1[3+i][9+i]),
                .Theta_7_10  (P_k1k1[6+i][9+i]),
                .Theta_10_10 (P_k1k1[9+i][9+i]),

                .Q_1_1       (Q_k[0+i][0+i]),

                .delta_t1    (delta_t2),
                .delta_t2    (dt2),
                .delta_t3    (three_dt3),
                .delta_t4    (twive_dt4),
                .delta_t5    (six_dt5),
                .delta_t6    (thirtysix_dt6),

                .a           (P_predicted[0+i][0+i]),
                .valid_out   (phi11_valid)
            );
        end

        for (i = 0; i < 3; i++) begin : GEN_PHI12
            CMU_PHi12 #(.DBL_WIDTH(64)) u_CMU_PHi12_minus1 (
                .clk         (clk),
                .rst_n       (rst_cmu),

                .Theta_1_4   (P_k1k1[0+i][3+i]),
                .Theta_1_1   (P_k1k1[0+i][0+i]),
                .Theta_1_7   (P_k1k1[0+i][6+i]),
                .Theta_4_7   (P_k1k1[3+i][6+i]),
                .Theta_1_10  (P_k1k1[0+i][9+i]),
                .Theta_4_10  (P_k1k1[3+i][9+i]),
                .Theta_7_7   (P_k1k1[6+i][6+i]),
                .Theta_7_10  (P_k1k1[6+i][9+i]),
                .Theta_10_10 (P_k1k1[9+i][9+i]),
                .Q_1_4       (Q_k[0+i][3+i]),

                .delta_t     (delta_t),
                .half_dt2    (half_dt2),
                .sixth_dt3   (sixth_dt3),
                .five12_dt4  (five12_dt4),
                .twleve_dt5  (twleve_dt5),

                .a           (P_predicted[0+i][1+i]),
                .valid_out   (phi12_valid_minus1)
            );
        end

        for (i = 0; i < 3; i++) begin : GEN_PHI13
            CMU_PHi13 #(.DBL_WIDTH(64)) u_CMU_PHi13 (
                .clk         (clk),
                .rst_n       (rst_cmu),

                .Theta_1_7   (P_k1k1[0+i][6+i]),
                .Theta_4_7   (P_k1k1[3+i][6+i]),
                .Theta_1_10  (P_k1k1[0+i][9+i]),
                .Theta_7_7   (P_k1k1[6+i][6+i]),
                .Theta_4_10  (P_k1k1[3+i][9+i]),
                .Theta_7_10  (P_k1k1[6+i][9+i]),
                .Theta_10_10 (P_k1k1[9+i][9+i]),
                .Q_1_7       (Q_k[0+i][6+i]),

                .delta_t     (delta_t),
                .half_dt2    (half_dt2),
                .two3_dt3    (two3_dt3),
                .sixth_dt4   (sixth_dt4),

                .a           (P_predicted[0+i][2+i]),
                .valid_out   (phi13_valid_minus1)
            );
        end

        for (i = 0; i < 3; i++) begin : GEN_PHI14
            CMU_PHi14 #(.DBL_WIDTH(64)) u_CMU_PHi14_minus1 (
                .clk         (clk),
                .rst_n       (rst_cmu),

                .Theta_1_10  (P_k1k1[0+i][9+i]),
                .Theta_4_10  (P_k1k1[3+i][9+i]),
                .Theta_7_10  (P_k1k1[6+i][9+i]),
                .Theta_10_10 (P_k1k1[9+i][9+i]),
                .Q_1_10      (Q_k[0+i][9+i]),

                .delta_t     (delta_t),
                .half_dt2    (half_dt2),
                .two3_dt3    (two3_dt3),   // ✅ 原来你写成 sixth_dt3 了，这里修正为 two3_dt3

                .a           (P_predicted[0+i][9+i]),
                .valid_out   (phi14_valid_minus1)
            );
        end

        // ……（下面 PHI21~PHI44 你保持原来 generate 内容即可）
        // 只要把每个 CMU 的 .rst_n(rst_n) 改成 .rst_n(rst_cmu) 就行
        // 并保持时间参数端口连接仍然用上面这些（delta_t2 / dt2 / half_dt2 / ...）

    endgenerate

    // =========================================================
    // 4) 扩展 inv_matrix -> 12x12
    // =========================================================
    logic [DWIDTH-1:0] inv_matrix12 [0:11][0:11];
    generate
        for (genvar r = 0; r < 12; r++) begin
            for (genvar c = 0; c < 12; c++) begin
                assign inv_matrix12[r][c] = (r < 6 && c < 6) ? inv_matrix[r][c] : {DWIDTH{1'b0}};
            end
        end
    endgenerate

    logic [DWIDTH-1:0] P_predicted_HT12 [0:11][0:11];
    generate
        for (genvar r = 0; r < 12; r++) begin
            for (genvar c = 0; c < 12; c++) begin
                assign P_predicted_HT12[r][c] = (c < 6) ? P_predicted_HT[r][c] : {DWIDTH{1'b0}};
            end
        end
    endgenerate

    // =========================================================
    // 5) 启动 systolic：锁存 inv_complete，等 tp_valid 后再打一拍 load_en
    // =========================================================
    logic inv_lat, systolic_fired;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inv_lat        <= 1'b0;
            systolic_fired <= 1'b0;
        end else begin
            if (tp_start) begin
                inv_lat        <= 1'b0;
                systolic_fired <= 1'b0;
            end
            if (inv_complete) inv_lat <= 1'b1;
            if (systolic_fired) begin
                // keep
            end
        end
    end

    wire systolic_start_pulse = inv_lat & tp_valid & ~systolic_fired;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            systolic_fired <= 1'b0;
        end else begin
            if (tp_start) systolic_fired <= 1'b0;
            else if (systolic_start_pulse) systolic_fired <= 1'b1;
        end
    end

    // =========================================================
    // 6) SystolicArray：计算 K
    // =========================================================
    logic [DWIDTH-1:0] K_k_matrix [0:11][0:11];
    logic              K_done;

    SystolicArray #(
        .DWIDTH  (64),
        .N       (12),
        .LATENCY (12)
    ) u_systolic (
        .clk        (clk),
        .rst_n      (rst_n),
        .a_row      (P_predicted_HT12),
        .b_col      (inv_matrix12),
        .load_en    (systolic_start_pulse),
        .enb_1      (1'b1),
        .enb_2_6    (1'b1),
        .enb_7_12   (1'b0),
        .c_out      (K_k_matrix),
        .cal_finish (K_done)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) CKG_Done <= 1'b0;
        else begin
            if (tp_start) CKG_Done <= 1'b0;
            else if (K_done) CKG_Done <= 1'b1;  // done level
        end
    end

    generate
        for (genvar r = 0; r < 12; r++) begin
            for (genvar c = 0; c < 6; c++) begin
                assign K_k[r][c] = K_k_matrix[r][c];
            end
        end
    endgenerate

endmodule
