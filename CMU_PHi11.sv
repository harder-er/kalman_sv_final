`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/05/08 22:01:37
// Design Name: 
// Module Name: CMU_PHi11
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module CMU_PHi11 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入参数
    input  logic [DBL_WIDTH-1:0]   Theta_1_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_1,
    input  logic [DBL_WIDTH-1:0]   Theta_7_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_1_1,
    // 时间参数（外部已算好）
    input  logic [DBL_WIDTH-1:0]   delta_t1,
    input  logic [DBL_WIDTH-1:0]   delta_t2,
    input  logic [DBL_WIDTH-1:0]   delta_t3,
    input  logic [DBL_WIDTH-1:0]   delta_t4,
    input  logic [DBL_WIDTH-1:0]   delta_t5,
    input  logic [DBL_WIDTH-1:0]   delta_t6,

    // 输出
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 有效标志流水线
    logic [5:0] valid_pipe;

    // 中间寄存器
    reg [63:0] stage1_M1, stage1_M2, stage1_M3;
    reg [63:0] stage2_A1, stage2_A2, stage2_A3, stage2_A4;
    reg [63:0] stage3_X1, stage3_X2, stage3_X3, stage3_X4, stage3_X5, stage3_X6;
    reg [63:0] stage4_T1, stage4_T2, stage4_T3, stage4_T4, stage4_T5;

    // 有效 / 完成标志
    logic multN_valid, finish_347, finish_377, finish_410;
    assign multN_valid = 1'b1;

    logic addA_valid, addA1_finish, addA2_finish, addA3_finish, addA4_finish;
    assign addA_valid = finish_347 & finish_377 & finish_410;

    logic multX_valid, multX1_finish, multX2_finish, multX3_finish, multX4_finish, multX5_finish, multX6_finish;
    // 目前暂时只用 X2~X6，X1 可以后续需要时再打开相应乘法器
    assign multX_valid = multX2_finish & multX3_finish & multX4_finish & multX5_finish & multX6_finish;

    logic addT_valid, addT1_finish, addT2_finish, addT3_finish, addT4_finish, addT5_finish;
    assign addT_valid = multX_valid;

    logic finaladd_valid, final_add_finish;
    assign finaladd_valid = addT1_finish & addT2_finish & addT3_finish & addT4_finish & addT5_finish;    

    //==========================================================================
    //  常数乘法：3Θ4,7 ；3Θ7,7 ；4Θ10,4 (用 Theta_4_10 代替对称项)
    //==========================================================================
    fp_multiplier mult_3_4_7 (
        .clk   (clk),
        .valid (multN_valid),
        .finish(finish_347),
        .a     (64'h4008_0000_0000_0000), // 3.0
        .b     (Theta_4_7),
        .result(stage1_M1)
    );

    fp_multiplier mult_3_7_7 (
        .clk   (clk),
        .valid (multN_valid),
        .finish(finish_377),
        .a     (64'h4008_0000_0000_0000), // 3.0
        .b     (Theta_7_7),
        .result(stage1_M2)
    );

    // 原代码使用 Theta_10_4_10（不存在），这里按注释“4*Θ10,4”使用 Theta_4_10
    fp_multiplier mult_4_10_4 (
        .clk   (clk),
        .valid (multN_valid),
        .finish(finish_410),
        .a     (64'h4010_0000_0000_0000), // 4.0
        .b     (Theta_4_10),
        .result(stage1_M3)
    );

    //==========================================================================
    //  A1~A4 加法
    //==========================================================================
    fp_adder add_A1 (
        .clk   (clk),
        .valid (addA_valid),
        .finish(addA1_finish),
        .a     (Theta_1_1),
        .b     (Q_1_1),
        .result(stage2_A1)
    );

    fp_adder add_A2 (
        .clk   (clk),
        .valid (addA_valid),
        .finish(addA2_finish),
        .a     (Theta_7_1),
        .b     (Theta_4_4),
        .result(stage2_A2)
    );

    fp_adder add_A3 (
        .clk   (clk),
        .valid (addA_valid),
        .finish(addA3_finish),
        .a     (Theta_10_1),
        .b     (stage1_M1),
        .result(stage2_A3)
    );

    fp_adder add_A4 (
        .clk   (clk),
        .valid (addA_valid),
        .finish(addA4_finish),
        .a     (stage1_M2),
        .b     (stage1_M3),
        .result(stage2_A4)
    );

    //==========================================================================
    //  时间参数乘法：X2~X6
    //==========================================================================
    // 若后续需要 X1，可按注释补回一个 fp_multiplier
    // fp_multiplier mult_X1 (
    //     .clk   (clk),
    //     .valid (multX_valid),
    //     .finish(multX1_finish),
    //     .a     (64'h4000_0000_0000_0000), // 2.0
    //     .b     (Theta_4_1),
    //     .result(stage3_X1)
    // );

    assign multX1_finish = 1'b1;   // 暂不使用 X1，避免 multX_valid 里出现未驱动信号
    assign stage3_X1     = 64'd0;  // 不参与计算时给个确定值

    fp_multiplier mult_X2 (
        .clk   (clk),
        .valid (multX_valid),
        .finish(multX2_finish),
        .a     (delta_t2),
        .b     (stage2_A2),
        .result(stage3_X2)
    );

    fp_multiplier mult_X3 (
        .clk   (clk),
        .valid (multX_valid),
        .finish(multX3_finish),
        .a     (delta_t3),
        .b     (stage2_A3),
        .result(stage3_X3)
    );

    fp_multiplier mult_X4 (
        .clk   (clk),
        .valid (multX_valid),
        .finish(multX4_finish),
        .a     (delta_t4),
        .b     (stage2_A4),
        .result(stage3_X4)
    );

    fp_multiplier mult_X5 (
        .clk   (clk),
        .valid (multX_valid),
        .finish(multX5_finish),
        .a     (delta_t5),
        .b     (Theta_7_10),
        .result(stage3_X5)
    );

    fp_multiplier mult_X6 (
        .clk   (clk),
        .valid (multX_valid),
        .finish(multX6_finish),
        .a     (delta_t6),
        .b     (Theta_10_10),
        .result(stage3_X6)
    );
    
    //==========================================================================
    //  累加 T1~T5
    //==========================================================================

    // T1 = A1 + X1  （目前 X1=0，相当于 T1=A1）
    fp_adder add_T1 (
        .clk   (clk),
        .valid (addT_valid),
        .finish(addT1_finish),
        .a     (stage2_A1),
        .b     (stage3_X1),
        .result(stage4_T1)
    );

    // T2 = X2 + X3
    fp_adder add_T2 (
        .clk   (clk),
        .valid (addT_valid),
        .finish(addT2_finish),
        .a     (stage3_X2),
        .b     (stage3_X3),
        .result(stage4_T2)
    );

    // T3 = X4 + X5
    fp_adder add_T3 (
        .clk   (clk),
        .valid (addT_valid),
        .finish(addT3_finish),
        .a     (stage3_X4),
        .b     (stage3_X5),
        .result(stage4_T3)
    );

    // *** 关键修改 ***
    // 原代码 add_T4 和 add_T3 共用 result(stage4_T3)，导致 stage4_T3 有两个驱动源 -> 多驱动错误
    // 这里修正为：T4 = T1 + X6，输出写入 stage4_T4
    fp_adder add_T4 (
        .clk   (clk),
        .valid (addT_valid),
        .finish(addT4_finish),
        .a     (stage4_T1),
        .b     (stage3_X6),
        .result(stage4_T4)
    );

    // T5 = T2 + T3
    fp_adder add_T5 (
        .clk   (clk),
        .valid (addT_valid),
        .finish(addT5_finish),
        .a     (stage4_T2),
        .b     (stage4_T3),
        .result(stage4_T5)
    );

    // 最终输出：a = T5 + T4
    fp_adder final_add (
        .clk   (clk),
        .valid (finaladd_valid),
        .finish(final_add_finish),
        .a     (stage4_T5),
        .b     (stage4_T4),
        .result(a)
    );

    //==========================================================================
    //  valid_out 流水线
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid_pipe <= 6'b0;
        else
            valid_pipe <= {valid_pipe[4:0], 1'b1};
    end

    assign valid_out = valid_pipe[5] & final_add_finish;

endmodule
