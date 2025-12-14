`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi12
// Description: CMU_PHi11 升级版，支持 PHi12 通道的六级流水计算
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi12 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_1_4,
    input  logic [DBL_WIDTH-1:0]   Theta_1_1,
    input  logic [DBL_WIDTH-1:0]   Theta_1_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_1_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_1_4,
    // —— 时间参数（常量） —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,
    input  logic [DBL_WIDTH-1:0]   half_dt2,   // ½·Δt²
    input  logic [DBL_WIDTH-1:0]   sixth_dt3,  // ⅙·Δt³
    input  logic [DBL_WIDTH-1:0]   five12_dt4, // 5/12·Δt⁴
    input  logic [DBL_WIDTH-1:0]   twleve_dt5, // 1/12·Δt⁵

    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // **中间信号**
    // M1..M3；A1..A4；X1..X6；T1..T5
    logic [DBL_WIDTH-1:0] M1, M2, M3;
    logic [DBL_WIDTH-1:0] A1, A2, A3, A4;
    logic [DBL_WIDTH-1:0] X1, X2, X3, X4, X5, X6;
    logic [DBL_WIDTH-1:0] T1, T2, T3, T4, T5;

    // valid/finish 信号线
    logic multM_valid, finish_M1, finish_M2, finish_M3;
    logic addA_valid, finish_A1, finish_A2, finish_A3, finish_A4;
    logic multX_valid, finish_X1, finish_X2, finish_X3, finish_X4, finish_X5, finish_X6;
    logic addT_valid, finish_T1, finish_T2, finish_T3, finish_T4, finish_T5;
    logic final_valid, finish_final;

    // 管线有效信号
    logic [5:0] valid_pipe;

    // ------------------------------------------------------------------------
    // 常数乘法： M1=3*Θ4,7； M2=3*Θ7,7； M3=4*Θ10,10
    // ------------------------------------------------------------------------
    assign multM_valid = 1'b1;
    fp_multiplier mult_M1 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M1),
        .a      (64'h4008_0000_0000_0000),  // 3.0
        .b      (Theta_4_7),
        .result (M1)
    );
    fp_multiplier mult_M2 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M2),
        .a      (64'h4008_0000_0000_0000),  // 3.0
        .b      (Theta_7_7),
        .result (M2)
    );
    fp_multiplier mult_M3 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M3),
        .a      (64'h4010_0000_0000_0000),  // 4.0
        .b      (Theta_4_10),
        .result (M3)
    );

    // ------------------------------------------------------------------------
    // 加法： A1 = Θ1,1 + Q1,4
    //       A2 = Θ1,7 + Θ1,4
    //       A3 = Θ1,10 + M1
    //       A4 = M2 + M3
    // ------------------------------------------------------------------------
    assign addA_valid = finish_M1 & finish_M2 & finish_M3;
    fp_adder add_A1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A1),
        .a      (Theta_1_1),
        .b      (Q_1_4),
        .result (A1)
    );
    fp_adder add_A2 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A2),
        .a      (Theta_1_7),
        .b      (Theta_1_4),
        .result (A2)
    );
    fp_adder add_A3 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A3),
        .a      (Theta_1_10),
        .b      (M1),
        .result (A3)
    );
    fp_adder add_A4 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A4),
        .a      (M2),
        .b      (M3),
        .result (A4)
    );

    // ------------------------------------------------------------------------
    // 时间乘法：X1..X6
    // ------------------------------------------------------------------------
    assign multX_valid = finish_A1 & finish_A2 & finish_A3 & finish_A4;
    fp_multiplier mult_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      ({1'b0,delta_t[DBL_WIDTH-1:1]}), // 2*Δt
        .b      (Theta_1_4),
        .result (X1)
    );
    fp_multiplier mult_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (half_dt2),
        .b      (A2),
        .result (X2)
    );
    fp_multiplier mult_X3 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X3),
        .a      (sixth_dt3),
        .b      (A3),
        .result (X3)
    );
    fp_multiplier mult_X4 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X4),
        .a      (five12_dt4),
        .b      (A4),
        .result (X4)
    );
    fp_multiplier mult_X5 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X5),
        .a      (twleve_dt5),
        .b      (Theta_7_10),
        .result (X5)
    );
    fp_multiplier mult_X6 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X6),
        .a      (delta_t),     // Δt⁶ 建议外部预计算
        .b      (Theta_10_10),
        .result (X6)
    );

    // ------------------------------------------------------------------------
    // 累加：T1..T5
    // ------------------------------------------------------------------------
    assign addT_valid = finish_X1 & finish_X2 & finish_X3 & finish_X4 & finish_X5 & finish_X6;
    fp_adder add_T1 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T1),
        .a      (A1),
        .b      (X1),
        .result (T1)
    );
    fp_adder add_T2 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T2),
        .a      (X2),
        .b      (X3),
        .result (T2)
    );
    fp_adder add_T3 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T3),
        .a      (X4),
        .b      (X5),
        .result (T3)
    );
    fp_adder add_T4 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T4),
        .a      (T1),
        .b      (X6),
        .result (T4)
    );
    fp_adder add_T5 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T5),
        .a      (T2),
        .b      (T3),
        .result (T5)
    );

    // ------------------------------------------------------------------------
    // 最终累加 a = T5 + T4
    // ------------------------------------------------------------------------
    assign final_valid = finish_T1 & finish_T2 & finish_T3 & finish_T4 & finish_T5;
    fp_adder final_add (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_final),
        .a      (T5),
        .b      (T4),
        .result (a)
    );



always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        valid_pipe <= '0;
    end else begin

        // valid 管线：5 级延迟后输出
        valid_pipe <= { valid_pipe[4:0], 1'b1 };
    end
end

// 输出
assign valid_out = valid_pipe[5]&finish_final;
// 最终 a 也是第5级累加的结果，直接从 stage4_T[4] 输出或再做一次加法



endmodule
