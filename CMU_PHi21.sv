`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi21
// Description: PHi21 通道的 CMU 计算，五级流水计算  
//              a = T4 + T1
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi21 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_4_1,
    input  logic [DBL_WIDTH-1:0]   Theta_7_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_1_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_4_1,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,         // Δt
    input  logic [DBL_WIDTH-1:0]   dt2_half,        // ½·Δt²
    input  logic [DBL_WIDTH-1:0]   dt3_sixth,       // ⅙·Δt³
    input  logic [DBL_WIDTH-1:0]   dt4_twelth,      // ⅟₁₂·Δt⁴
    input  logic [DBL_WIDTH-1:0]   dt5_twelth,      // ⅟₁₂·Δt⁵
    input  logic [DBL_WIDTH-1:0]   dt6_thirtysix,   // ⅟₃₆·Δt⁶ (unused)
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间结果信号
    logic [DBL_WIDTH-1:0] M1, M2, M3, M4, M5;
    logic [DBL_WIDTH-1:0] A1, A2, A3, A4, A5;
    logic [DBL_WIDTH-1:0] X1, X2, X3, X4, X5;
    logic [DBL_WIDTH-1:0] T1, T2, T3, T4;

    // valid/finish 信号
    logic multM_valid,   finish_M1, finish_M2, finish_M3, finish_M4, finish_M5;
    logic addA_valid,    finish_A1, finish_A2, finish_A3, finish_A4, finish_A5;
    logic multX_valid,   finish_X1, finish_X2, finish_X3, finish_X4, finish_X5;
    logic addT_valid,    finish_T1, finish_T2, finish_T3, finish_T4;
    logic final_valid,   finish_final;

    // valid_out 管线
    logic [4:0] valid_pipe;

    // ------------------------------------------------------------------------
    // Stage1: 常量乘法 M1..M5
    // ------------------------------------------------------------------------
    assign multM_valid = 1'b1;
    fp_multiplier mult_M1 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M1),
        .a      (64'h4008_0000_0000_0000), // 3.0
        .b      (Theta_4_7),
        .result (M1)
    );
    fp_multiplier mult_M2 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M2),
        .a      (64'h4008_0000_0000_0000), // 3.0
        .b      (Theta_7_7),
        .result (M2)
    );
    fp_multiplier mult_M3 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M3),
        .a      (64'h4010_0000_0000_0000), // 4.0
        .b      (Theta_4_10),
        .result (M3)
    );
    fp_multiplier mult_M4 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M4),
        .a      (64'h4008_0000_0000_0000), // 3.0
        .b      (Theta_7_10),
        .result (M4)
    );
    fp_multiplier mult_M5 (
        .clk    (clk),
        .valid  (multM_valid),
        .finish (finish_M5),
        .a      (64'h4000_0000_0000_0000), // 2.0
        .b      (Theta_4_7),
        .result (M5)
    );

    // ------------------------------------------------------------------------
    // Stage2: 加法 A1..A5
    // ------------------------------------------------------------------------
    assign addA_valid = finish_M1 & finish_M2 & finish_M3 & finish_M4 & finish_M5;
    fp_adder add_A1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A1),
        .a      (Theta_4_1),
        .b      (Q_4_1),
        .result (A1)
    );
    fp_adder add_A2 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A2),
        .a      (Theta_7_1),
        .b      (Theta_4_4),
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
    fp_adder add_A5 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A5),
        .a      (M4),
        .b      (M5),
        .result (A5)
    );

    // ------------------------------------------------------------------------
    // Stage3: 时间乘法 X1..X5
    // ------------------------------------------------------------------------
    assign multX_valid = finish_A1 & finish_A2 & finish_A3 & finish_A4 & finish_A5;
    fp_multiplier mult_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (delta_t),
        .b      (A2),
        .result (X1)
    );
    fp_multiplier mult_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (dt2_half),
        .b      (A3),
        .result (X2)
    );
    fp_multiplier mult_X3 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X3),
        .a      (dt3_sixth),
        .b      (A4),
        .result (X3)
    );
    fp_multiplier mult_X4 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X4),
        .a      (dt4_twelth),
        .b      (A5),
        .result (X4)
    );
    fp_multiplier mult_X5 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X5),
        .a      (dt5_twelth),
        .b      (Theta_1_10),
        .result (X5)
    );

    // ------------------------------------------------------------------------
    // Stage4: 累加 T1..T4
    // ------------------------------------------------------------------------
    assign addT_valid = finish_X1 & finish_X2 & finish_X3 & finish_X4 & finish_X5;
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
        .valid  (addT_valid & finish_T1),
        .finish (finish_T2),
        .a      (X2),
        .b      (X3),
        .result (T2)
    );
    fp_adder add_T3 (
        .clk    (clk),
        .valid  (addT_valid & finish_T1 & finish_T2),
        .finish (finish_T3),
        .a      (X4),
        .b      (X5),
        .result (T3)
    );
    fp_adder add_T4 (
        .clk    (clk),
        .valid  (addT_valid & finish_T1 & finish_T2 & finish_T3),
        .finish (finish_T4),
        .a      (T2),
        .b      (T3),
        .result (T4)
    );

    // ------------------------------------------------------------------------
    // Stage5: 最终累加 a = T4 + T1
    // ------------------------------------------------------------------------
    assign final_valid = finish_T4;
    fp_adder add_final (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_final),
        .a      (T4),
        .b      (T1),
        .result (a)
    );

    // ------------------------------------------------------------------------
    // valid_out 管线
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid_pipe <= 5'b0;
        else
            valid_pipe <= { valid_pipe[3:0], finish_final };
    end

    assign valid_out = valid_pipe[4];

endmodule
