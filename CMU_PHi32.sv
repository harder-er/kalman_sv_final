`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi32
// Description: PHi32 通道的 CMU 计算，四级流水计算  
//              a = (Θ7,4 + Q7,4)·? + (Δt·A2 + 3/2·Δt²·Θ7,10 + ½·Δt³·Θ10,10)
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi32 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,    // (unused)
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_7_4,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,         // Δt
    input  logic [DBL_WIDTH-1:0]   three2_dt2,      // 3/2·Δt²
    input  logic [DBL_WIDTH-1:0]   half_dt3,        // ½·Δt³
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] A1, A2;
    logic [DBL_WIDTH-1:0] X1, X2, X3;
    logic [DBL_WIDTH-1:0] T1, T2;

    // valid/finish 信号
    logic addA_valid,    finish_A1, finish_A2;
    logic multX_valid,   finish_X1, finish_X2, finish_X3;
    logic addT_valid,    finish_T1, finish_T2;
    logic final_valid,   finish_final;

    // 流水段寄存器（保留原样）
    logic [DBL_WIDTH-1:0] stage1_A1, stage1_X1;
    logic [DBL_WIDTH-1:0] stage2_X2, stage2_X3;
    logic [DBL_WIDTH-1:0] stage3_T1, stage3_T2;
    // valid 管线，三级流水
    logic [2:0]           valid_pipe;

    // ----------------- Stage1: 加法 A1, A2 -----------------
    assign addA_valid = 1'b1;
    // A1 = Θ7,4 + Q7,4
    fp_adder U_add_A1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A1),
        .a      (Theta_7_4),
        .b      (Q_7_4),
        .result (A1)
    );
    // A2 = Θ7,10 + Θ7,7
    fp_adder U_add_A2 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A2),
        .a      (Theta_7_10),
        .b      (Theta_7_7),
        .result (A2)
    );

    // ----------------- Stage2: 乘法 X1..X3 -----------------
    assign multX_valid = finish_A1 & finish_A2;
    // X1 = Δt * A2
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (delta_t),
        .b      (A2),
        .result (X1)
    );
    // X2 = 3/2·Δt² * Θ7,10
    fp_multiplier U_mul_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (three2_dt2),
        .b      (Theta_7_10),
        .result (X2)
    );
    // X3 = ½·Δt³ * Θ10,10
    fp_multiplier U_mul_X3 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X3),
        .a      (half_dt3),
        .b      (Theta_10_10),
        .result (X3)
    );

    // ----------------- Stage3: 累加 T1, T2 -----------------
    assign addT_valid = finish_X1 & finish_X2 & finish_X3;
    // T1 = A1 + X1
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

    // ----------------- Stage4: 最终累加 a = T1 + T2 -----------------
    assign final_valid = finish_T2;
    fp_adder U_add_final (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_final),
        .a      (T1),
        .b      (T2),
        .result (a)
    );

    // ----------------- 流水线寄存与控制 -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe  <= 3'b000;
        end else begin
            valid_pipe  <= { valid_pipe[1:0], finish_final };
        end
    end

    assign valid_out = valid_pipe[2]&final_valid;

endmodule
