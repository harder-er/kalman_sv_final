`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Shared 2-mul + 2-add version (sequential FSM) to reduce FP IP usage
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

    // 2 路乘 + 2 路加的共享单元
    logic mul_go [0:1], mul_finish [0:1];
    logic [DBL_WIDTH-1:0] mul_a [0:1], mul_b [0:1], mul_r [0:1];
    logic add_go [0:1], add_finish [0:1];
    logic [DBL_WIDTH-1:0] add_a [0:1], add_b [0:1], add_r [0:1];

    fp_multiplier u_mul0 (.clk(clk), .valid(mul_go[0]), .finish(mul_finish[0]), .a(mul_a[0]), .b(mul_b[0]), .result(mul_r[0]));
    fp_multiplier u_mul1 (.clk(clk), .valid(mul_go[1]), .finish(mul_finish[1]), .a(mul_a[1]), .b(mul_b[1]), .result(mul_r[1]));
    fp_adder      u_add0 (.clk(clk), .valid(add_go[0]), .finish(add_finish[0]), .a(add_a[0]), .b(add_b[0]), .result(add_r[0]));
    fp_adder      u_add1 (.clk(clk), .valid(add_go[1]), .finish(add_finish[1]), .a(add_a[1]), .b(add_b[1]), .result(add_r[1]));

    // 中间寄存器
    logic [DBL_WIDTH-1:0] m1, m2, m3;
    logic [DBL_WIDTH-1:0] a1, a2, a3, a4;
    logic [DBL_WIDTH-1:0] x2, x3, x4, x5, x6;
    logic [DBL_WIDTH-1:0] t1, t2, t3, t4, t5;

    typedef enum logic [3:0] {
        S_IDLE,
        S_M1M2, S_M3,
        S_A12, S_A34,
        S_X23, S_X45, S_X6,
        S_T2T3, S_T4T5, S_FINAL
    } state_e;

    state_e state;
    logic done_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            {m1,m2,m3,a1,a2,a3,a4,x2,x3,x4,x5,x6,t1,t2,t3,t4,t5,a} <= '{default:'0};
            done_pipe <= 1'b0;
        end else begin
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            done_pipe <= 1'b0;

            case (state)
                S_IDLE: begin
                    // 启动常数乘法 M1/M2
                    mul_a[0] <= 64'h4008_0000_0000_0000; // 3.0
                    mul_b[0] <= Theta_4_7;
                    mul_a[1] <= 64'h4008_0000_0000_0000; // 3.0
                    mul_b[1] <= Theta_7_7;
                    mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                    state <= S_M1M2;
                end

                S_M1M2: begin
                    if (mul_finish[0]) m1 <= mul_r[0];
                    if (mul_finish[1]) m2 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        mul_a[0] <= 64'h4010_0000_0000_0000; // 4.0
                        mul_b[0] <= Theta_4_10;
                        mul_go[0] <= 1'b1;
                        state <= S_M3;
                    end
                end

                S_M3: begin
                    if (mul_finish[0]) begin
                        m3 <= mul_r[0];
                        // A1/A2
                        add_a[0] <= Theta_1_1; add_b[0] <= Q_1_1;
                        add_a[1] <= Theta_7_1; add_b[1] <= Theta_4_4;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_A12;
                    end
                end

                S_A12: begin
                    if (add_finish[0]) a1 <= add_r[0];
                    if (add_finish[1]) a2 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        add_a[0] <= Theta_10_1; add_b[0] <= m1;
                        add_a[1] <= m2;         add_b[1] <= m3;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_A34;
                    end
                end

                S_A34: begin
                    if (add_finish[0]) a3 <= add_r[0];
                    if (add_finish[1]) a4 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // X2/X3
                        mul_a[0] <= delta_t2; mul_b[0] <= a2;
                        mul_a[1] <= delta_t3; mul_b[1] <= a3;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_X23;
                    end
                end

                S_X23: begin
                    if (mul_finish[0]) x2 <= mul_r[0];
                    if (mul_finish[1]) x3 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        mul_a[0] <= delta_t4; mul_b[0] <= a4;
                        mul_a[1] <= delta_t5; mul_b[1] <= Theta_7_10;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_X45;
                    end
                end

                S_X45: begin
                    if (mul_finish[0]) x4 <= mul_r[0];
                    if (mul_finish[1]) x5 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        mul_a[0] <= delta_t6; mul_b[0] <= Theta_10_10;
                        mul_go[0] <= 1'b1;
                        state <= S_X6;
                    end
                end

                S_X6: begin
                    if (mul_finish[0]) begin
                        x6 <= mul_r[0];
                        // T2 = X2+X3, T3 = X4+X5
                        add_a[0] <= x2; add_b[0] <= x3;
                        add_a[1] <= x4; add_b[1] <= x5;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_T2T3;
                    end
                end

                S_T2T3: begin
                    if (add_finish[0]) t2 <= add_r[0];
                    if (add_finish[1]) t3 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // T4 = A1 + X6, T5 = T2 + T3
                        add_a[0] <= a1; add_b[0] <= x6;
                        add_a[1] <= t2; add_b[1] <= t3;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_T4T5;
                    end
                end

                S_T4T5: begin
                    if (add_finish[0]) t4 <= add_r[0];
                    if (add_finish[1]) t5 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        add_a[0] <= t5; add_b[0] <= t4;
                        add_go[0] <= 1'b1;
                        state <= S_FINAL;
                    end
                end

                S_FINAL: begin
                    if (add_finish[0]) begin
                        a <= add_r[0];
                        done_pipe <= 1'b1;
                        state <= S_IDLE; // 可重复使用
                    end
                end
            endcase
        end
    end

    // valid_out：在最终加法完成后拉高一个周期
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_out <= 1'b0;
        else        valid_out <= done_pipe;
    end

endmodule
