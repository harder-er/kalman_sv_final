`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CEU_x
// Description: “x” 通道 CEU 计算，两级流水 def = T4 + T5
// Dependencies: fp_adder, fp_multiplier
//////////////////////////////////////////////////////////////////////////////////
module CEU_x #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // Theta 输入
    input  logic [DBL_WIDTH-1:0]   Theta_1_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_1,
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Theta_1_4,

    // Q/R 噪声
    input  logic [DBL_WIDTH-1:0]   Q_1_4,
    input  logic [DBL_WIDTH-1:0]   R_1_4,

    // 时间系数
    input  logic [DBL_WIDTH-1:0]   delta_t,
    input  logic [DBL_WIDTH-1:0]   half_dt2,
    input  logic [DBL_WIDTH-1:0]   sixth_dt3,
    input  logic [DBL_WIDTH-1:0]   five12_dt4,
    input  logic [DBL_WIDTH-1:0]   one12_dt5,

    // 输出
    output logic [DBL_WIDTH-1:0]   x,
    output logic                   valid_out
);

    // 流水寄存
    logic [DBL_WIDTH-1:0] stage1_A1, stage1_A4, stage2_M1, stage2_M2, stage2_M3;
    logic [DBL_WIDTH-1:0] stage3_A2, stage3_A3, stage4_X1, stage4_X2, stage4_X3, stage4_X4, stage4_X5;
    logic [DBL_WIDTH-1:0] T1, T2, T3, T4, T5;
    logic [1:0]           pipe_valid;

    // valid/finish 信号
    logic add1_valid, finish_A1, finish_A4;
    logic mul1_valid, finish_M1, finish_M2, finish_M3;
    logic add2_valid, finish_A2, finish_A3;
    logic mul2_valid, finish_X1, finish_X2, finish_X3, finish_X4, finish_X5;
    logic add3_valid, finish_T1, finish_T2, finish_T3, finish_T4, finish_T5;

    // ========== Stage1 ========== 
    // A1 = Θ4_4 + Θ1_7
    assign add1_valid = 1'b1;
    fp_adder U1_add_A1 (
        .clk    (clk), .valid(add1_valid), .finish(finish_A1),
        .a(Theta_4_4), .b(Theta_1_7), .result(stage1_A1)
    );
    // A4 = Q1_4 + R1_4
    fp_adder U1_add_A4 (
        .clk    (clk), .valid(add1_valid), .finish(finish_A4),
        .a(Q_1_4), .b(R_1_4), .result(stage1_A4)
    );
    // M1 = 3*Θ7_4, M2 = 3*Θ7_7, M3 = 4*Θ10_4
    assign mul1_valid = finish_A1 & finish_A4;
    fp_multiplier U1_mul_M1 (
        .clk    (clk), .valid(mul1_valid), .finish(finish_M1),
        .a(64'h4008_0000_0000_0000), .b(Theta_7_4), .result(stage2_M1)
    );
    fp_multiplier U1_mul_M2 (
        .clk    (clk), .valid(mul1_valid), .finish(finish_M2),
        .a(64'h4008_0000_0000_0000), .b(Theta_7_7), .result(stage2_M2)
    );
    fp_multiplier U1_mul_M3 (
        .clk    (clk), .valid(mul1_valid), .finish(finish_M3),
        .a(64'h4010_0000_0000_0000), .b(Theta_10_4), .result(stage2_M3)
    );

    // ========== Stage2 ==========
    assign add2_valid = finish_M1 & finish_M2 & finish_M3;
    // A2 = Θ10_1 + M1
    fp_adder U2_add_A2 (
        .clk    (clk), .valid(add2_valid), .finish(finish_A2),
        .a(Theta_10_1), .b(stage2_M1), .result(stage3_A2)
    );
    // A3 = M2 + M3
    fp_adder U2_add_A3 (
        .clk    (clk), .valid(add2_valid), .finish(finish_A3),
        .a(stage2_M2), .b(stage2_M3), .result(stage3_A3)
    );
    // X1 = Δt * A1
    fp_multiplier U2_mul_X1 (
        .clk    (clk), .valid(add2_valid), .finish(finish_X1),
        .a(delta_t), .b(stage1_A1), .result(stage4_X1)
    );
    // X2 = ½Δt² * A2
    fp_multiplier U2_mul_X2 (
        .clk    (clk), .valid(add2_valid & finish_A2), .finish(finish_X2),
        .a(half_dt2), .b(stage3_A2), .result(stage4_X2)
    );
    // X3 = ⅙Δt³ * A3
    fp_multiplier U2_mul_X3 (
        .clk    (clk), .valid(add2_valid & finish_A2 & finish_X2), .finish(finish_X3),
        .a(sixth_dt3), .b(stage3_A3), .result(stage4_X3)
    );
    // X4 = 5/12Δt⁴ * Θ10_7
    fp_multiplier U2_mul_X4 (
        .clk    (clk), .valid(add2_valid & finish_A2 & finish_X2 & finish_X3), .finish(finish_X4),
        .a(five12_dt4), .b(Theta_10_7), .result(stage4_X4)
    );
    // X5 = 1/12Δt⁵ * Θ10_10
    fp_multiplier U2_mul_X5 (
        .clk    (clk), .valid(add2_valid & finish_A2 & finish_X2 & finish_X3 & finish_X4), .finish(finish_X5),
        .a(one12_dt5), .b(Theta_10_10), .result(stage4_X5)
    );

    // ========== Stage3 (final) ==========
    assign add3_valid = finish_X5;
    // T1 = Θ1_4 + X1
    fp_adder U3_add_T1 (
        .clk    (clk), .valid(add3_valid), .finish(finish_T1),
        .a(Theta_1_4), .b(stage4_X1), .result(T1)
    );
    // T2 = X2 + X3
    fp_adder U3_add_T2 (
        .clk    (clk), .valid(add3_valid & finish_T1), .finish(finish_T2),
        .a(stage4_X2), .b(stage4_X3), .result(T2)
    );
    // T3 = X4 + X5
    fp_adder U3_add_T3 (
        .clk    (clk), .valid(add3_valid & finish_T1 & finish_T2), .finish(finish_T3),
        .a(stage4_X4), .b(stage4_X5), .result(T3)
    );
    // T4 = T1 + T2
    fp_adder U3_add_T4 (
        .clk    (clk), .valid(add3_valid & finish_T1 & finish_T2 & finish_T3), .finish(finish_T4),
        .a(T1), .b(T2), .result(T4)
    );
    // T5 = T3 + A4
    fp_adder U3_add_T5 (
        .clk    (clk), .valid(add3_valid & finish_T1 & finish_T2 & finish_T3 & finish_T4), .finish(finish_T5),
        .a(T3), .b(stage1_A4), .result(T5)
    );
    // x = T4 + T5
    fp_adder U3_add_x (
        .clk    (clk), .valid(finish_T5), .finish(),
        .a(T4), .b(T5), .result(x)
    );

    // ========== 保留原有 pipe_valid ==========
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pipe_valid <= 2'b00;
        else
            pipe_valid <= { pipe_valid[0], finish_T5 };
    end
    assign valid_out = pipe_valid[1];

endmodule
