`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi43
// Description: PHi43 通道的 CMU 计算，双级流水计算  
//              a = (Θ10,7 + Q10,7) + (Δt·Θ10,10)
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi43 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_10_7,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,    // Δt
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] T1, X1;

    // valid/finish 信号
    logic addA_valid,    finish_A1;
    logic multX_valid,   finish_X1;
    logic final_valid,   finish_final;

    // 流水段寄存器（保持原样）
    logic [DBL_WIDTH-1:0] stage1_X1, stage1_T1;
    logic [DBL_WIDTH-1:0] stage2_a;
    logic [1:0]           valid_pipe;

    // ----------------- Stage1: 加法 T1 -----------------
    assign addA_valid = 1'b1;
    fp_adder U_add_T1 (
        .clk    (clk),
        .valid  (addA_valid),
        .finish (finish_A1),
        .a      (Theta_10_7),
        .b      (Q_10_7),
        .result (T1)
    );

    // ----------------- Stage2: 乘法 X1 -----------------
    assign multX_valid = finish_A1;
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (delta_t),
        .b      (Theta_10_10),
        .result (X1)
    );

    // ----------------- Stage3: final a = T1 + X1 -----------------
    assign final_valid = finish_X1;
    fp_adder U_add_final (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_final),
        .a      (T1),
        .b      (X1),
        .result (stage2_a)
    );

    // ----------------- 流水线寄存与控制 -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe <= 2'b00;
        end else begin
            // valid 管线移位并注入 finish_final
            valid_pipe <= { valid_pipe[0], finish_final };
        end
    end

    assign a         = stage2_a;
    assign valid_out = valid_pipe[1] & finish_final;

endmodule
