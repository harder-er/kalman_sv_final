`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi24
// Description: PHi24 通道的 CMU 计算，三级流水计算  
//              a = (Θ4,10 + Q4,10) + (Δt·Θ7,4 + ½Δt²·Θ10,10)
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi24 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_4_10,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,   // Δt
    input  logic [DBL_WIDTH-1:0]   half_dt2,  // ½·Δt²
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] X1, X2;
    logic [DBL_WIDTH-1:0] T1, T2;

    // valid/finish 信号
    logic multX_valid,   finish_X1, finish_X2;
    logic addT1_valid,   finish_T1;
    logic addT2_valid,   finish_T2;
    logic final_valid,   finish_final;

    // 流水段寄存器
    logic [DBL_WIDTH-1:0] stage1_T1, stage1_X1, stage1_X2;
    logic [DBL_WIDTH-1:0] stage2_T2;
    // 有效信号管线，三级流水
    logic [2:0]           valid_pipe;

    // ----------------- Stage1: 乘法 X1,X2 + 加法 T1 -----------------
    assign multX_valid  = 1'b1;
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (delta_t),
        .b      (Theta_7_4),
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
    assign addT1_valid = finish_X1 & finish_X2;
    fp_adder U_add_T1 (
        .clk    (clk),
        .valid  (addT1_valid),
        .finish (finish_T1),
        .a      (Theta_4_10),
        .b      (Q_4_10),
        .result (T1)
    );

    // ----------------- Stage2: 加法 T2 = X1 + X2 -----------------
    assign addT2_valid = finish_T1;
    fp_adder U_add_T2 (
        .clk    (clk),
        .valid  (addT2_valid),
        .finish (finish_T2),
        .a      (X1),
        .b      (X2),
        .result (T2)
    );

    // ----------------- Stage3: final a = T1 + T2 -----------------
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
            // valid 管线移位并注入 finish_final
            valid_pipe  <= { valid_pipe[1:0], finish_final };
        end
    end

    assign valid_out = valid_pipe[2]&finish_final;

endmodule
