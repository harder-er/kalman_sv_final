`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CMU_PHi41
// Description: PHi41 通道的 CMU 计算，四级流水计算  
//              a = (Θ10,1 + Q10,1) + (Δt·Θ10,4 + ½·Δt²·Θ10,7 + ⅙·Δt³·Θ10,10)
// Dependencies: fp_multiplier, fp_adder
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi41 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // —— 动态输入 —— 
    input  logic [DBL_WIDTH-1:0]   Theta_10_1,
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_10_1,
    // —— 时间参数 —— 
    input  logic [DBL_WIDTH-1:0]   delta_t,      // Δt
    input  logic [DBL_WIDTH-1:0]   half_dt2,     // ½·Δt²
    input  logic [DBL_WIDTH-1:0]   sixth_dt3,    // ⅙·Δt³
    // —— 输出 —— 
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 中间信号
    logic [DBL_WIDTH-1:0] X1, X2, X3;
    logic [DBL_WIDTH-1:0] T1, T2, T3;

    // valid/finish signals
    logic multX_valid;
    logic finish_X1, finish_X2, finish_X3;
    logic addT_valid;
    logic finish_T1, finish_T2, finish_T3;
    logic final_valid, finish_final;

    // pipeline regs (unchanged)
    logic [DBL_WIDTH-1:0] stage1_X1, stage1_X2, stage1_X3, stage1_T1;
    logic [DBL_WIDTH-1:0] stage2_T2;
    logic [DBL_WIDTH-1:0] stage3_T3;
    logic [3:0]           valid_pipe;

    // ----------------- Stage1: multipliers X1, X2, X3 -----------------
    assign multX_valid = 1'b1;
    fp_multiplier U_mul_X1 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X1),
        .a      (delta_t),
        .b      (Theta_10_4),
        .result (X1)
    );
    fp_multiplier U_mul_X2 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X2),
        .a      (half_dt2),
        .b      (Theta_10_7),
        .result (X2)
    );
    fp_multiplier U_mul_X3 (
        .clk    (clk),
        .valid  (multX_valid),
        .finish (finish_X3),
        .a      (sixth_dt3),
        .b      (Theta_10_10),
        .result (X3)
    );

    // ----------------- Stage2: adder T1, T2 -----------------
    assign addT_valid = finish_X1 & finish_X2 & finish_X3;
    fp_adder U_add_T1 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T1),
        .a      (Theta_10_1),
        .b      (Q_10_1),
        .result (T1)
    );
    fp_adder U_add_T2 (
        .clk    (clk),
        .valid  (addT_valid & finish_T1),
        .finish (finish_T2),
        .a      (X1),
        .b      (X2),
        .result (T2)
    );

    // ----------------- Stage3: adder T3 -----------------
    fp_adder U_add_T3 (
        .clk    (clk),
        .valid  (addT_valid),
        .finish (finish_T3),
        .a      (T1),
        .b      (T2),
        .result (T3)
    );

    // ----------------- Stage4: final a = T3 + X3 -----------------
    assign final_valid = finish_T3 & finish_X3;
    fp_adder U_add_final (
        .clk    (clk),
        .valid  (final_valid),
        .finish (finish_final),
        .a      (T3),
        .b      (X3),
        .result (a)
    );

    // ----------------- pipeline registers & valid pipe -----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            
            valid_pipe  <= 4'b0000;
        end else begin
            
            // valid pipeline shift in finish_final
            valid_pipe  <= { valid_pipe[2:0], finish_final };
        end
    end

    assign valid_out = valid_pipe[3]&final_valid;

endmodule
