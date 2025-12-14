`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi31
// Description: PHi31 通道的 CMU 计算，四级流水计算 a = X4 + T3
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi31 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_7_1,
    input  logic [DBL_WIDTH-1:0]   Theta_1_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_7_1,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,      // Δt
    input  logic [DBL_WIDTH-1:0]   half_dt2,     // ½·Δt²
    input  logic [DBL_WIDTH-1:0]   two3_dt3,     // ⅔·Δt³
    input  logic [DBL_WIDTH-1:0]   sixth_dt4,    // ⅙·Δt⁴
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

    // 流水段寄存器（保持原样）
    logic [DBL_WIDTH-1:0] stage1_A1;
    logic [DBL_WIDTH-1:0] stage2_X1, stage2_X2, stage2_X3, stage2_X4;
    logic [DBL_WIDTH-1:0] stage3_T1, stage3_T2, stage3_T3;
    logic [3:0]           valid_pipe;

    // ----------------- Stage1: 加法 A1..A3 -----------------
    assign addA_valid = 1'b1;
    fp_adder U_add_A1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A1),
        .a      (Theta_7_1),
        .b      (Q_7_1),
        .result (A1)
    );
    fp_adder U_add_A2 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A2),
        .a      (Theta_1_10),
        .b      (Theta_4_7),
        .result (A2)
    );
    wire [DBL_WIDTH-1:0] tmp_sum;
    fp_adder U_add_tmp (
        .clk    (clk),
        .valid  (addA_valid & finish_A1 & finish_A2),
        .finish (),
        .a      (Theta_7_7),
        .b      (Theta_4_10),
        .result (tmp_sum)
    );
    fp_adder U_add_A3 (
        .clk    (clk),
        .valid  (addA_valid & finish_A1 & finish_A2),
        .finish (finish_A3),
        .a      (tmp_sum),
        .b      (Theta_4_10),
        .result (A3)
    );

    // ----------------- Stage2: 乘法 X1..X4 -----------------
    assign multX_valid = finish_A1 & finish_A2 & finish_A3;
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (delta_t),
        .b      (A2),
        .result (X1)
    );
    fp_multiplier U_mul_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (half_dt2),
        .b      (A3),
        .result (X2)
    );
    fp_multiplier U_mul_X3 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X3),
        .a      (two3_dt3),
        .b      (Theta_7_10),
        .result (X3)
    );
    fp_multiplier U_mul_X4 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X4),
        .a      (sixth_dt4),
        .b      (Theta_10_10),
        .result (X4)
    );

    // ----------------- Stage3: 加法 T1..T3 -----------------
    assign addT_valid = finish_X1 & finish_X2 & finish_X3 & finish_X4;
    fp_adder U_add_T1 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T1),
        .a      (A1),
        .b      (X1),
        .result (T1)
    );
    fp_adder U_add_T2 (
        .clk    (clk),
        .valid  (addT_valid & finish_T1),
        .finish (finish_T2),
        .a      (X2),
        .b      (X3),
        .result (T2)
    );
    fp_adder U_add_T3 (
        .clk    (clk),
        .valid  (addT_valid & finish_T1 & finish_T2),
        .finish (finish_T3),
        .a      (T1),
        .b      (T2),
        .result (T3)
    );

    // ----------------- Stage4: final a = X4 + T3 -----------------
    assign final_valid = finish_T3 & finish_X4;
    fp_adder U_add_final (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_final),
        .a      (X4),
        .b      (T3),
        .result (a)
    );

    // ----------------- 流水线寄存与控制 -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe   <= 4'b0000;
        end else begin
            valid_pipe   <= { valid_pipe[2:0], finish_final };
        end
    end

    assign valid_out = valid_pipe[3]&finish_final;

endmodule
