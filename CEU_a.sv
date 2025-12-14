`timescale 1ns / 1ps
module CEU_a #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 动态输入
    // input  logic [DBL_WIDTH-1:0]   Theta_1_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_1,
    input  logic [DBL_WIDTH-1:0]   Theta_7_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_1,
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_1_1,
    input  logic [DBL_WIDTH-1:0]   R_1_1,
    // 固定参数
    input  logic [DBL_WIDTH-1:0]   dt_1,   // 2*Δt
    input  logic [DBL_WIDTH-1:0]   dt_2,   // Δt^2
    input  logic [DBL_WIDTH-1:0]   dt_3,   // (1/3)Δt^3
    input  logic [DBL_WIDTH-1:0]   dt_4,   // (1/12)Δt^4
    input  logic [DBL_WIDTH-1:0]   dt_5,   // (1/6)Δt^5
    input  logic [DBL_WIDTH-1:0]   dt_6,   // (1/36)Δt^6
    // 输出
    output logic [DBL_WIDTH-1:0]   a_out,
    output logic                   valid_out
);

    // 已定义的流水寄存信号
    logic [DBL_WIDTH-1:0] stage1_A1, stage1_A4;
    logic [DBL_WIDTH-1:0] stage2_M1, stage2_M2, stage2_M3;
    logic [DBL_WIDTH-1:0] stage3_A2, stage3_A3;
    logic [DBL_WIDTH-1:0] stage4_X1, stage4_X2, stage4_X3, stage4_X4, stage4_X5, stage4_X6;
    logic [DBL_WIDTH-1:0] stage5_T1, stage5_T2, stage5_T3, stage5_T4;
    logic [DBL_WIDTH-1:0] stage6_T5, stage6_T6;
    logic [6:0]           pipe_valid;

    // valid/finish 信号
    logic add1_valid, finish_A1, finish_A4;
    logic mul1_valid, finish_M1, finish_M2, finish_M3;
    logic add2_valid, finish_A2, finish_A3;
    logic mul2_valid, finish_X1, finish_X2, finish_X3, finish_X4, finish_X5, finish_X6;
    logic add3_valid, finish_T1, finish_T2, finish_T3, finish_T4;
    logic add4_valid, finish_T5, finish_T6;
    logic final_valid, finish_out;

    // ---------------- Stage1: A1 = Θ7_1+Θ4_4, A4=Q_1_1+R_1_1 ----------------
    assign add1_valid = 1'b1;
    fp_adder U1_add_A1   (.clk(clk), .valid(add1_valid), .finish(finish_A1),
                          .a(Theta_7_1), .b(Theta_4_4), .result(stage1_A1));
    fp_adder U1_add_A4   (.clk(clk), .valid(add1_valid), .finish(finish_A4),
                          .a(Q_1_1),     .b(R_1_1),     .result(stage1_A4));

    // ---------------- Stage2: M1=3*Θ7_4, M2=4*Θ10_4, M3=3*Θ7_7 ----------------
    assign mul1_valid = finish_A1 & finish_A4;
    fp_multiplier U2_mul_M1 (.clk(clk), .valid(mul1_valid), .finish(finish_M1),
                             .a(64'h4008_0000_0000_0000), .b(Theta_7_4), .result(stage2_M1));
    fp_multiplier U2_mul_M2 (.clk(clk), .valid(mul1_valid), .finish(finish_M2),
                             .a(64'h4010_0000_0000_0000), .b(Theta_10_4),.result(stage2_M2));
    fp_multiplier U2_mul_M3 (.clk(clk), .valid(mul1_valid), .finish(finish_M3),
                             .a(64'h4008_0000_0000_0000), .b(Theta_7_7), .result(stage2_M3));

    // ---------------- Stage3: A2=Θ10_1+M1, A3=M2+M3 ----------------
    assign add2_valid = finish_M1 & finish_M2 & finish_M3;
    fp_adder U3_add_A2 (.clk(clk), .valid(add2_valid), .finish(finish_A2),
                       .a(Theta_10_1), .b(stage2_M1), .result(stage3_A2));
    fp_adder U3_add_A3 (.clk(clk), .valid(add2_valid), .finish(finish_A3),
                       .a(stage2_M2),   .b(stage2_M3), .result(stage3_A3));

    // ---------------- Stage4: X1..X6 ----------------
    assign mul2_valid = finish_A2 & finish_A3;
    fp_multiplier U4_mul_X1 (.clk(clk), .valid(mul2_valid), .finish(finish_X1),
                             .a(dt_1), .b(Theta_4_1),   .result(stage4_X1));
    fp_multiplier U4_mul_X2 (.clk(clk), .valid(mul2_valid), .finish(finish_X2),
                             .a(dt_2), .b(stage1_A1),   .result(stage4_X2));
    fp_multiplier U4_mul_X3 (.clk(clk), .valid(mul2_valid), .finish(finish_X3),
                             .a(dt_3), .b(stage3_A2),   .result(stage4_X3));
    fp_multiplier U4_mul_X4 (.clk(clk), .valid(mul2_valid), .finish(finish_X4),
                             .a(dt_4), .b(stage3_A3),   .result(stage4_X4));
    fp_multiplier U4_mul_X5 (.clk(clk), .valid(mul2_valid), .finish(finish_X5),
                             .a(dt_5), .b(Theta_10_7),  .result(stage4_X5));
    fp_multiplier U4_mul_X6 (.clk(clk), .valid(mul2_valid), .finish(finish_X6),
                             .a(dt_6), .b(Theta_10_10), .result(stage4_X6));

    // ---------------- Stage5: T1=Θ10_7+X1, T2=X2+X3 ----------------
    assign add3_valid = finish_X1 & finish_X2 & finish_X3 & finish_X4 & finish_X5 & finish_X6;
    fp_adder U5_add_T1 (.clk(clk), .valid(add3_valid), .finish(finish_T1),
                       .a(Theta_10_7), .b(stage4_X1), .result(stage5_T1));
    fp_adder U5_add_T2 (.clk(clk), .valid(add3_valid), .finish(finish_T2),
                       .a(stage4_X2),   .b(stage4_X3), .result(stage5_T2));

    // ---------------- Stage6: T3=X4+X5, T4=X6+A4 ----------------
    assign add4_valid = finish_T2;
    fp_adder U6_add_T3 (.clk(clk), .valid(add4_valid), .finish(finish_T3),
                       .a(stage4_X4),  .b(stage4_X5), .result(stage5_T3));
    fp_adder U6_add_T4 (.clk(clk), .valid(add4_valid), .finish(finish_T4),
                       .a(stage4_X6),  .b(stage1_A4), .result(stage5_T4));

    // ---------------- Stage7: T5=T1+T2, T6=T3+T4 ----------------
    assign final_valid = finish_T4;
    fp_adder U7_add_T5  (.clk(clk), .valid(final_valid), .finish(finish_T5),
                        .a(stage5_T1), .b(stage5_T2), .result(stage6_T5));
    fp_adder U7_add_T6  (.clk(clk), .valid(final_valid ), .finish(finish_T6),
                        .a(stage5_T3), .b(stage5_T4), .result(stage6_T6));
    // 最终输出
    fp_adder U7_add_out (.clk(clk), .valid(final_valid & finish_T5 & finish_T6), .finish(finish_out),
                        .a(stage6_T5), .b(stage6_T6), .result(a_out));

    // ---------------- Stage0: valid 管线 ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pipe_valid <= 7'b0;
        else
            pipe_valid <= { pipe_valid[5:0], finish_out };
    end



    // 输出有效信号：当最高位为 1 时，a_out 为有效数据
    assign valid_out = pipe_valid[6]&finish_out;

endmodule
