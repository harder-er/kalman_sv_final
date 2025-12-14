`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi44
// Description: PHi44 通道的 CMU 计算，一级流水计算  
//              a = Θ10,10 + Q10,10
// Dependencies: fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi44 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_10_10,
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] sum;
    // valid/finish 信号
    logic add_valid, finish_sum;
    // 流水有效信号管线（1 级流水）
    logic valid_pipe;

    // ----------------- Stage1: 加法 -----------------
    assign add_valid = 1'b1;
    fp_adder U_add (
        .clk    (clk),
        .valid  (add_valid),
        .finish (finish_sum),
        .a      (Theta_10_10),
        .b      (Q_10_10),
        .result (sum)
    );

    // ----------------- 流水寄存与控制 -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe <= 1'b0;
        end else begin
            valid_pipe <= finish_sum;
        end
    end

    assign valid_out = valid_pipe;
    assign a = sum;
endmodule
