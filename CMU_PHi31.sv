`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Shared 2-mul + 2-add FSM version (PHi31)
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi31 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入
    input  logic [DBL_WIDTH-1:0]   Theta_7_1,
    input  logic [DBL_WIDTH-1:0]   Theta_1_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_7_1,
    // 时间参数
    input  logic [DBL_WIDTH-1:0]   delta_t,
    input  logic [DBL_WIDTH-1:0]   half_dt2,
    input  logic [DBL_WIDTH-1:0]   two3_dt3,
    input  logic [DBL_WIDTH-1:0]   sixth_dt4,
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
    logic [DBL_WIDTH-1:0] a1, a2, a3;
    logic [DBL_WIDTH-1:0] x1, x2, x3, x4;
    logic [DBL_WIDTH-1:0] t1, t2, t3;

    typedef enum logic [3:0] {
        S_IDLE,
        S_A12, S_A3,
        S_X12, S_X34,
        S_T12, S_T3, S_FINAL
    } state_e;

    state_e state;
    logic done_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            {a1,a2,a3,x1,x2,x3,x4,t1,t2,t3,a} <= '{default:'0};
            done_pipe <= 1'b0;
        end else begin
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            done_pipe <= 1'b0;

            case (state)
                S_IDLE: begin
                    // A1 = Theta_7_1 + Q_7_1, A2 = Theta_1_10 + Theta_4_7
                    add_a[0] <= Theta_7_1; add_b[0] <= Q_7_1;
                    add_a[1] <= Theta_1_10; add_b[1] <= Theta_4_7;
                    add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                    state <= S_A12;
                end

                S_A12: begin
                    if (add_finish[0]) a1 <= add_r[0];
                    if (add_finish[1]) a2 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // A3 = Theta_7_10 + Theta_4_10
                        add_a[0] <= Theta_7_10; add_b[0] <= Theta_4_10;
                        add_go[0] <= 1'b1;
                        state <= S_A3;
                    end
                end

                S_A3: begin
                    if (add_finish[0]) begin
                        a3 <= add_r[0];
                        // X1 = delta_t * a2, X2 = half_dt2 * Theta_7_7
                        mul_a[0] <= delta_t;  mul_b[0] <= a2;
                        mul_a[1] <= half_dt2; mul_b[1] <= Theta_7_7;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_X12;
                    end
                end

                S_X12: begin
                    if (mul_finish[0]) x1 <= mul_r[0];
                    if (mul_finish[1]) x2 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // X3 = two3_dt3 * a3, X4 = sixth_dt4 * Theta_10_10
                        mul_a[0] <= two3_dt3;  mul_b[0] <= a3;
                        mul_a[1] <= sixth_dt4; mul_b[1] <= Theta_10_10;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_X34;
                    end
                end

                S_X34: begin
                    if (mul_finish[0]) x3 <= mul_r[0];
                    if (mul_finish[1]) x4 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // T1 = a1 + x1, T2 = x2 + x3
                        add_a[0] <= a1; add_b[0] <= x1;
                        add_a[1] <= x2; add_b[1] <= x3;
                        add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                        state <= S_T12;
                    end
                end

                S_T12: begin
                    if (add_finish[0]) t1 <= add_r[0];
                    if (add_finish[1]) t2 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // T3 = x4 + Theta_1_10 (占位复用加法器)
                        add_a[0] <= x4; add_b[0] <= Theta_1_10;
                        add_go[0] <= 1'b1;
                        state <= S_T3;
                    end
                end

                S_T3: begin
                    if (add_finish[0]) begin
                        t3 <= add_r[0];
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
