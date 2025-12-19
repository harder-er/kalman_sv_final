`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Shared 2-mul + 2-add FSM version (PHi42)
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi42 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_10_4,
    // 时间参数
    input  logic [DBL_WIDTH-1:0]   delta_t,
    input  logic [DBL_WIDTH-1:0]   half_dt2,
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
    logic [DBL_WIDTH-1:0] a1;
    logic [DBL_WIDTH-1:0] x1, x2;

    typedef enum logic [1:0] {
        S_IDLE,
        S_A1,
        S_X12,
        S_FINAL
    } state_e;

    state_e state;
    logic done_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            {a1,x1,x2,a} <= '{default:'0};
            done_pipe <= 1'b0;
        end else begin
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            done_pipe <= 1'b0;

            case (state)
                S_IDLE: begin
                    // A1 = Theta_10_4 + Q_10_4
                    add_a[0] <= Theta_10_4; add_b[0] <= Q_10_4;
                    add_go[0] <= 1'b1;
                    state <= S_A1;
                end

                S_A1: begin
                    if (add_finish[0]) begin
                        a1 <= add_r[0];
                        // X1 = delta_t * Theta_10_7, X2 = half_dt2 * Theta_10_10
                        mul_a[0] <= delta_t;  mul_b[0] <= Theta_10_7;
                        mul_a[1] <= half_dt2; mul_b[1] <= Theta_10_10;
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_X12;
                    end
                end

                S_X12: begin
                    if (mul_finish[0]) x1 <= mul_r[0];
                    if (mul_finish[1]) x2 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // a = a1 + x1 + x2 -> 分两次加
                        add_a[0] <= a1; add_b[0] <= x1;
                        add_go[0] <= 1'b1;
                        state <= S_FINAL;
                    end
                end

                S_FINAL: begin
                    if (add_finish[0]) begin
                        // 第二次加：add0 继续复用
                        add_a[0] <= add_r[0]; add_b[0] <= x2;
                        add_go[0] <= 1'b1;
                    end
                    if (add_finish[0] && add_r[0] != add_r[0]); // placeholder to keep structure
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
