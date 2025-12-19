`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Shared 2-mul + 2-add FSM version (PHi21)
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi21 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 动态输入
    input  logic [DBL_WIDTH-1:0]   Theta_4_1,
    input  logic [DBL_WIDTH-1:0]   Theta_7_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_1_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_4_1,
    // 时间参数
    input  logic [DBL_WIDTH-1:0]   delta_t,
    input  logic [DBL_WIDTH-1:0]   dt2_half,
    input  logic [DBL_WIDTH-1:0]   dt3_sixth,
    input  logic [DBL_WIDTH-1:0]   dt4_twelth,
    input  logic [DBL_WIDTH-1:0]   dt5_twelth,
    input  logic [DBL_WIDTH-1:0]   dt6_thirtysix, // unused but kept for interface
    // 输出
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);

    // 2 路乘 + 2 路加共享单元
    logic mul_go [0:1], mul_finish [0:1];
    logic [DBL_WIDTH-1:0] mul_a [0:1], mul_b [0:1], mul_r [0:1];
    logic add_go [0:1], add_finish [0:1];
    logic [DBL_WIDTH-1:0] add_a [0:1], add_b [0:1], add_r [0:1];

    fp_multiplier u_mul0 (.clk(clk), .valid(mul_go[0]), .finish(mul_finish[0]), .a(mul_a[0]), .b(mul_b[0]), .result(mul_r[0]));
    fp_multiplier u_mul1 (.clk(clk), .valid(mul_go[1]), .finish(mul_finish[1]), .a(mul_a[1]), .b(mul_b[1]), .result(mul_r[1]));
    fp_adder      u_add0 (.clk(clk), .valid(add_go[0]), .finish(add_finish[0]), .a(add_a[0]), .b(add_b[0]), .result(add_r[0]));
    fp_adder      u_add1 (.clk(clk), .valid(add_go[1]), .finish(add_finish[1]), .a(add_a[1]), .b(add_b[1]), .result(add_r[1]));

    // 中间寄存器
    logic [DBL_WIDTH-1:0] m1, m2, m3, m4, m5;
    logic [DBL_WIDTH-1:0] a1, a2, a3, a4, a5;
    logic [DBL_WIDTH-1:0] x1, x2, x3, x4, x5;
    logic [DBL_WIDTH-1:0] t1, t2, t3, t4;

    typedef enum logic [3:0] {
        S_IDLE,
        S_M12, S_M34, S_M5,
        S_A12, S_A345,
        S_X12, S_X34, S_X5,
        S_T12, S_T34, S_T5, S_FINAL
    } state_e;

    state_e state;
    logic done_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            {m1,m2,m3,m4,m5,a1,a2,a3,a4,a5,x1,x2,x3,x4,x5,t1,t2,t3,t4,a} <= '{default:'0};
            done_pipe <= 1'b0;
        end else begin
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            done_pipe <= 1'b0;

            case (state)
                S_IDLE: begin
                    // M1 = 3*Θ4_7, M2 = 3*Θ7_7
                    mul_a[0] <= 64'h4008_0000_0000_0000; mul_b[0] <= Theta_4_7;
                    mul_a[1] <= 64'h4008_0000_0000_0000; mul_b[1] <= Theta_7_7;
                    mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                    state <= S_M12;
                end

                S_M12: begin
                    if (mul_finish[0]) m1 <= mul_r[0];
                    if (mul_finish[1]) m2 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // M3 = 4*Θ4_10, M4 = 3*Θ7_10
                        mul_a[0] <= 64'h4010_0000_0000_0000; mul_b[0] <= Theta_4_10;
                        mul_a[1] <= 64'h4008_0000_0000_0000; mul_b[1] <= Theta_7_10;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_M34;
                    end
                end

                S_M34: begin
                    if (mul_finish[0]) m3 <= mul_r[0];
                    if (mul_finish[1]) m4 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // M5 = 2*Θ4_7
                        mul_a[0] <= 64'h4000_0000_0000_0000; mul_b[0] <= Theta_4_7;
                        mul_go[0] <= 1'b1;
                        state <= S_M5;
                    end
                end

                S_M5: begin
                    if (mul_finish[0]) begin
                        m5 <= mul_r[0];
                        // A1 = Θ4_1 + Q_4_1, A2 = Θ7_1 + Θ4_4
                        add_a[0] <= Theta_4_1; add_b[0] <= Q_4_1;
                        add_a[1] <= Theta_7_1; add_b[1] <= Theta_4_4;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_A12;
                    end
                end

                S_A12: begin
                    if (add_finish[0]) a1 <= add_r[0];
                    if (add_finish[1]) a2 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // A3 = Theta_1_10 + M1; A4 = M2 + M3
                        add_a[0] <= Theta_1_10; add_b[0] <= m1;
                        add_a[1] <= m2;         add_b[1] <= m3;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_A345;
                    end
                end

                S_A345: begin
                    if (add_finish[0]) a3 <= add_r[0];
                    if (add_finish[1]) a4 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // A5 = M4 + M5
                        add_a[0] <= m4; add_b[0] <= m5;
                        add_go[0] <= 1'b1;
                        state <= S_X12; // prepare X after A5 ready
                    end
                end

                S_X12: begin
                    if (add_finish[0]) begin
                        a5 <= add_r[0];
                        // X1 = delta_t * a2, X2 = dt2_half * a3
                        mul_a[0] <= delta_t;  mul_b[0] <= a2;
                        mul_a[1] <= dt2_half; mul_b[1] <= a3;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_X34;
                    end
                end

                S_X34: begin
                    if (mul_finish[0]) x1 <= mul_r[0];
                    if (mul_finish[1]) x2 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // X3 = dt3_sixth * a4, X4 = dt4_twelth * a5
                        mul_a[0] <= dt3_sixth;  mul_b[0] <= a4;
                        mul_a[1] <= dt4_twelth; mul_b[1] <= a5;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_X5;
                    end
                end

                S_X5: begin
                    if (mul_finish[0]) x3 <= mul_r[0];
                    if (mul_finish[1]) x4 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // X5 = dt5_twelth * Theta_1_10
                        mul_a[0] <= dt5_twelth; mul_b[0] <= Theta_1_10;
                        mul_go[0] <= 1'b1;
                        state <= S_T12;
                    end
                end

                S_T12: begin
                    if (mul_finish[0]) begin
                        x5 <= mul_r[0];
                        // T1 = a1 + x1, T2 = x2 + x3
                        add_a[0] <= a1; add_b[0] <= x1;
                        add_a[1] <= x2; add_b[1] <= x3;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_T34;
                    end
                end

                S_T34: begin
                    if (add_finish[0]) t1 <= add_r[0];
                    if (add_finish[1]) t2 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // T3 = x4 + x5, T4 = t1 + Theta_4_10 (复用加法器，占位)
                        add_a[0] <= x4; add_b[0] <= x5;
                        add_a[1] <= t1; add_b[1] <= Theta_4_10;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_T5;
                    end
                end

                S_T5: begin
                    if (add_finish[0]) t3 <= add_r[0];
                    if (add_finish[1]) t4 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // a = t2 + t3
                        add_a[0] <= t2; add_b[0] <= t3;
                        add_go[0] <= 1'b1;
                        state <= S_FINAL;
                    end
                end

                S_FINAL: begin
                    if (add_finish[0]) begin
                        a <= add_r[0];
                        done_pipe <= 1'b1;
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_out <= 1'b0;
        else        valid_out <= done_pipe;
    end

endmodule
