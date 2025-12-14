`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi22
// Description: PHi22 通道的 CMU 计算，四级流水计算  
//              a = (A1 + X1 + X2 + X3) + X4
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi22 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_4_4,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   two_dt,       // 2·Δt
    input  logic [DBL_WIDTH-1:0]   dt2,          // Δt²
    input  logic [DBL_WIDTH-1:0]   half_dt3,     // ½·Δt³
    input  logic [DBL_WIDTH-1:0]   quarter_dt4,  // ¼·Δt⁴
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] A1, A2, A3;
    logic [DBL_WIDTH-1:0] X1, X2, X3, X4;
    logic [DBL_WIDTH-1:0] T1, T2, T3;

    // valid/finish 信号
    logic addA_valid,    finish_A1, finish_A2, finish_A3;
    logic multX_valid,   finish_X1, finish_X2, finish_X3, finish_X4;
    logic addT_valid,    finish_T1, finish_T2, finish_T3;
    logic final_valid,   finish_final;

    // valid_out 管线
    logic [2:0] valid_pipe;

    // ----------------- Stage1: 三路加法 A1..A3 -----------------
    assign addA_valid = 1'b1;
    fp_adder U_add_A1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A1),
        .a      (Theta_4_4),
        .b      (Q_4_4),
        .result (A1)
    );
    fp_adder U_add_A2 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A2),
        .a      (Theta_4_10),
        .b      (Theta_7_7),
        .result (A2)
    );
    fp_adder U_add_A3 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A3),
        .a      (Theta_7_10),
        .b      (Theta_4_7),
        .result (A3)
    );

    // ----------------- Stage2: 四路乘法 X1..X4 -----------------
    assign multX_valid = finish_A1 & finish_A2 & finish_A3;
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (two_dt),
        .b      (Theta_4_7),
        .result (X1)
    );
    fp_multiplier U_mul_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (dt2),
        .b      (A2),
        .result (X2)
    );
    fp_multiplier U_mul_X3 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X3),
        .a      (half_dt3),
        .b      (A3),
        .result (X3)
    );
    fp_multiplier U_mul_X4 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X4),
        .a      (quarter_dt4),
        .b      (Theta_10_10),
        .result (X4)
    );

    // ----------------- Stage3: 三路加法 T1..T3 -----------------
    // T1 = A1 + X1
    assign addT_valid = finish_X1 & finish_X2 & finish_X3 & finish_X4;
    fp_adder U_add_T1 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T1),
        .a      (A1),
        .b      (X1),
        .result (T1)
    );
    // T2 = X2 + X3
    fp_adder U_add_T2 (
        .clk    (clk),
        .valid  (addT_valid & finish_T1),
        .finish (finish_T2),
        .a      (X2),
        .b      (X3),
        .result (T2)
    );
    // T3 = T1 + T2
    fp_adder U_add_T3 (
        .clk    (clk),
        .valid  (addT_valid & finish_T1 & finish_T2),
        .finish (finish_T3),
        .a      (T1),
        .b      (T2),
        .result (T3)
    );

    // ----------------- Stage4: 最终加法 a = T3 + X4 -----------------
    assign final_valid = finish_T3 & finish_X4;
    fp_adder U_add_final (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_final),
        .a      (T3),
        .b      (X4),
        .result (a)
    );

    // ----------------- valid_out 管线 -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid_pipe <= 3'b000;
        else
            valid_pipe <= { valid_pipe[1:0], finish_final };
    end


    // ----------------- 流水线寄存与控制 -----------------
    // Stage registers
    logic [DBL_WIDTH-1:0] stage1_A1;
    logic [DBL_WIDTH-1:0] stage2_X1, stage2_X2, stage2_X3;
    logic [DBL_WIDTH-1:0] stage3_T1, stage3_T2, stage3_T3;


    assign valid_out = valid_pipe[2] & finish_final;

endmodule
