`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi33
// Description: PHi33 通道的 CMU 计算，双级流水计算  
//              a = (Θ7,7 + Q7,7) + (2Δt·Θ7,10 + Δt²·Θ10,10)
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi33 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_7_7,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   two_dt,    // 2·Δt
    input  logic [DBL_WIDTH-1:0]   dt2,       // Δt²
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] A1;
    logic [DBL_WIDTH-1:0] X1, X2;
    logic [DBL_WIDTH-1:0] T1, T2;

    // valid/finish 信号
    logic multX_valid,   finish_X1, finish_X2;
    logic addA_valid,    finish_A1;
    logic final_valid,   finish_final;

    // 流水段寄存器（保留原样）
    logic [DBL_WIDTH-1:0] stage1_T1, stage1_X1, stage1_X2;
    logic [DBL_WIDTH-1:0] stage2_T2;
    // valid 管线，2 级流水
    logic [1:0]           valid_pipe;

    // ----------------- Stage1: 加法 A1 -----------------
    assign addA_valid = 1'b1;
    fp_adder U_add_A1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A1),
        .a      (Theta_7_7),
        .b      (Q_7_7),
        .result (A1)
    );

    // ----------------- Stage2: 乘法 X1, X2 -----------------
    assign multX_valid = finish_A1;
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (two_dt),
        .b      (Theta_7_10),
        .result (X1)
    );
    fp_multiplier U_mul_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (dt2),
        .b      (Theta_10_10),
        .result (X2)
    );

    // ----------------- Stage3: 累加 T1, T2 -----------------
    assign final_valid = finish_X1 & finish_X2;
    logic finish_T1;
    // T1 = A1 + X1
    fp_adder U_add_T1 (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_T1 ),
        .a      (A1),
        .b      (X1),
        .result (T1)
    );
    // T2 = X2 (bypassed)
    assign T2 = X2;

    // ----------------- Stage4: final a = T1 + T2 -----------------
    fp_adder U_add_final (
        .clk    (clk),
        .valid  (finish_T1),
        .finish (finish_final),
        .a      (T1),
        .b      (T2),
        .result (a)
    );

    // ----------------- 流水线寄存与控制 -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe  <= 2'b00;
        end else begin
            valid_pipe  <= { valid_pipe[0], finish_final };
        end
    end

    assign valid_out = valid_pipe[1]&final_valid;

endmodule
