`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi42
// Description: PHi42 通道的 CMU 计算，四级流水计算  
//              a = (Θ10,4 + Q10,4) + (Δt·Θ10,7 + ½·Δt²·Θ10,10)
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi42 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_10_4,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,   // Δt
    input  logic [DBL_WIDTH-1:0]   half_dt2,  // ½·Δt²
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] X1, X2, T1, T2;

    // valid/finish 信号
    logic multX_valid,   finish_X1, finish_X2;
    logic addA_valid,    finish_T1;
    logic final_valid,   finish_T2;

    // 流水段寄存器（保持原样）
    logic [DBL_WIDTH-1:0] stage1_T1, stage1_X1, stage1_X2;
    logic [DBL_WIDTH-1:0] stage2_a;
    // 有效信号管线，2 级流水
    logic [1:0]           valid_pipe;

    // ----------------- Stage1: 乘法 X1, X2 -----------------
    assign multX_valid = 1'b1;
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (delta_t),
        .b      (Theta_10_7),
        .result (X1)
    );
    fp_multiplier U_mul_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (half_dt2),
        .b      (Theta_10_10),
        .result (X2)
    );

    // ----------------- Stage2: 加法 T1 -----------------
    assign addA_valid = finish_X1 & finish_X2;
    fp_adder U_add_T1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_T1),
        .a      (Theta_10_4),
        .b      (Q_10_4),
        .result (T1)
    );

    // ----------------- Stage3: 加法 T2 = X1 + X2 -----------------
    assign final_valid = finish_T1;
    fp_adder U_add_T2 (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_T2),
        .a      (X1),
        .b      (X2),
        .result (T2)
    );

    // ----------------- Stage4: final a = T1 + T2 -----------------
    // reuse finish_T2 as finish_final
    // result is captured by pipeline reg stage2_a
    wire finish_final = finish_T2;
    fp_adder U_add_final (
        .clk    (clk),
        .valid  (finish_T2),
        .finish (),
        .a      (T1),
        .b      (T2),
        .result (stage2_a)
    );

    // ----------------- 流水线寄存与控制 -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe <= 2'b00;
        end else begin
            // Stage1 寄存 T1, X1, X2
            // valid 管线移位并注入 finish_final
            valid_pipe <= { valid_pipe[0], finish_final };
        end
    end

    assign a         = stage2_a;
    assign valid_out = valid_pipe[1] & finish_final;

endmodule
