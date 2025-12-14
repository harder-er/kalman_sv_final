`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// Create Date: 2025/04/25 17:46:27
// Module Name: CEU_alpha
// Description: 计算 α = in1*in2 - in3*in3，采用两级流水线，带 valid/finish 信号
// Dependencies: fp_arithmetic.svh (声明 fp_multiplier, fp_suber)
//////////////////////////////////////////////////////////////////////////////////
module CEU_alpha #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入数据有效
    input  logic [DBL_WIDTH-1:0]   in1,    // 对应 a
    input  logic [DBL_WIDTH-1:0]   in2,    // 对应 d
    input  logic [DBL_WIDTH-1:0]   in3,    // 对应 x
    // 输出
    output logic [DBL_WIDTH-1:0]   out,    // α = in1*in2 - in3*in3
    output logic                   valid_out
);

    // --------------------------------------------------
    // Stage1: 两个乘法
    // --------------------------------------------------
    wire [DBL_WIDTH-1:0] m1, m2;
    logic mult1_valid, finish_mul1, finish_mul2;

    // in1 * in2
    assign mult1_valid = 1'b1;
    fp_multiplier U_mul1 (
        .clk    (clk),
        .valid  (mult1_valid),
        .finish (finish_mul1),
        .a      (in1),
        .b      (in2),
        .result (m1)
    );
    // in3 * in3
    fp_multiplier U_mul2 (
        .clk    (clk),
        .valid  (mult1_valid),
        .finish (finish_mul2),
        .a      (in3),
        .b      (in3),
        .result (m2)
    );

    // --------------------------------------------------
    // Stage2: 相减
    // --------------------------------------------------
    wire [DBL_WIDTH-1:0] diff_m;
    logic sub_valid, finish_sub;

    // 当两个乘法都完成后启动减法
    assign sub_valid = finish_mul1 & finish_mul2;
    fp_suber U_sub (
        .clk    (clk),
        .valid  (sub_valid),
        .finish (finish_sub),
        .a      (m1),
        .b      (m2),
        .result (diff_m)
    );

    // --------------------------------------------------
    // 流水线寄存与 valid 管线
    // --------------------------------------------------
    assign out = diff_m;
    assign valid_out = finish_sub;

endmodule
