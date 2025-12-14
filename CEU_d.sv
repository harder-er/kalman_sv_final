`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CEU_d
// Description: CEU_d 模块，四级流水计算 d = T5 = stage4_T5 + stage3_T3
// Dependencies: fp_adder, fp_multiplier
//////////////////////////////////////////////////////////////////////////////////
module CEU_d #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 静态输入：对应 "def" 的相关输入
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Q_4_4,
    input  logic [DBL_WIDTH-1:0]   R_4_4,
    // 动态输入参数
    input  logic [DBL_WIDTH-1:0]   delta_t2,    // 2·t
    input  logic [DBL_WIDTH-1:0]   delta_t_sq,  // t^2
    input  logic [DBL_WIDTH-1:0]   delta_t_hcu, // ½·t^3
    input  logic [DBL_WIDTH-1:0]   delta_t_qr,  // ¼·t^4
    // 输出
    output logic [DBL_WIDTH-1:0]   d,
    output logic                   valid_out
);

    // 流水寄存信号（保持不变）
    logic [DBL_WIDTH-1:0] stage1_A1, stage1_A2;
    logic [DBL_WIDTH-1:0] stage2_X1, stage2_X2, stage2_X3, stage2_X4;
    logic [DBL_WIDTH-1:0] stage3_T1, stage3_T2, stage3_T3;
    logic [DBL_WIDTH-1:0] stage4_T4, stage4_T5;
    logic [4:0]           pipe_valid;

    // valid/finish 信号
    logic add1_valid,    finish_A1, finish_A2;
    logic mul_valid,     finish_X1, finish_X2, finish_X3, finish_X4;
    logic add2_valid,    finish_T1, finish_T2, finish_T3;
    logic add3_valid,    finish_T4, finish_T5;

    // ---------------- Stage1: 加法 A1 = Θ10_7+Θ7_4, A2 = Q4_4+R4_4 -------------
    assign add1_valid = 1'b1;
    fp_adder U1_add_A1 (
        .clk    (clk        ),
        .valid  (add1_valid ),
        .finish (finish_A1  ),
        .a      (Theta_10_7 ),
        .b      (Theta_7_4  ),
        .result (stage1_A1  )
    );

    fp_adder U1_add_A2 (
        .clk    (clk        ),
        .valid  (add1_valid ),
        .finish (finish_A2  ),
        .a      (Q_4_4      ),
        .b      (R_4_4      ),
        .result (stage1_A2  )
    );

    // ---------------- Stage2: 乘法 X1..X4 ----------------
    assign mul_valid = finish_A1 & finish_A2;
    fp_multiplier U2_mul_X1 (
        .clk    (clk        ),
        .valid  (mul_valid  ),
        .finish (finish_X1  ),
        .a      (delta_t2   ),
        .b      (Theta_7_4  ),
        .result (stage2_X1  )
    );
    fp_multiplier U2_mul_X2 (
        .clk    (clk        ),
        .valid  (mul_valid  ),
        .finish (finish_X2  ),
        .a      (delta_t_sq ),
        .b      (Theta_10_4 ),
        .result (stage2_X2  )
    );
    fp_multiplier U2_mul_X3 (
        .clk    (clk            ),
        .valid  (mul_valid      ),
        .finish (finish_X3      ),
        .a      (delta_t_hcu    ),
        .b      (stage1_A1      ),
        .result (stage2_X3      )
    );
    fp_multiplier U2_mul_X4 (
        .clk    (clk            ),
        .valid  (mul_valid      ),
        .finish (finish_X4      ),
        .a      (delta_t_qr     ),
        .b      (Theta_10_10    ),
        .result (stage2_X4      )
    );

    // ---------------- Stage3: 加法 T1..T3 ----------------
    assign add2_valid = finish_X1 & finish_X2 & finish_X3 & finish_X4;
    fp_adder U3_add_T1 (
        .clk    (clk),
        .valid  (add2_valid),
        .finish (finish_T1),
        .a      (Theta_4_4),
        .b      (stage2_X1),
        .result (stage3_T1)
    );
    fp_adder U3_add_T2 (
        .clk    (clk),
        .valid  (add2_valid),
        .finish (finish_T2),
        .a      (stage2_X2),
        .b      (stage2_X3),
        .result (stage3_T2)
    );
    fp_adder U3_add_T3 (
        .clk    (clk),
        .valid  (add2_valid),
        .finish (finish_T3),
        .a      (stage1_A2),
        .b      (stage2_X4),
        .result (stage3_T3)
    );

    // ---------------- Stage4: T4=T1+T2, T5=T3+T4 ----------------
    assign add3_valid = finish_T3;
    fp_adder U4_add_T4 (
        .clk    (clk),
        .valid  (add3_valid),
        .finish (finish_T4),
        .a      (stage3_T1),
        .b      (stage3_T2),
        .result (stage4_T4)
    );
    
    logic finish_out;
    // 输出赋值
    fp_adder U5_add_T6 (
        .clk    (clk),
        .valid  (add3_valid),
        .finish (finish_out ),
        .a      (stage3_T3  ),
        .b      (stage4_T4  ),
        .result (d          )
    );

    // ---------------- valid 管线 ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_valid <= 2'b00;
        end else begin
            pipe_valid <= { pipe_valid[3:0], finish_T5 };
        end
    end



    // 最终输出赋值
    assign valid_out = pipe_valid[4]&finish_T5;

endmodule