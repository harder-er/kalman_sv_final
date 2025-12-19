`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Shared 2-mul + 2-add FSM version (PHi34)
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi34 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_7_10,
    // 时间参数
    input  logic [DBL_WIDTH-1:0]   delta_t,
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
    logic [DBL_WIDTH-1:0] t1, x1;

    typedef enum logic [1:0] {
        S_IDLE,
        S_A1,
        S_X1,
        S_FINAL
    } state_e;

    state_e state;
    logic done_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            {t1,x1,a} <= '{default:'0};
            done_pipe <= 1'b0;
        end else begin
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            done_pipe <= 1'b0;

            case (state)
                S_IDLE: begin
                    // T1 = Theta_7_10 + Q_7_10
                    add_a[0] <= Theta_7_10; add_b[0] <= Q_7_10;
                    add_go[0] <= 1'b1;
                    state <= S_A1;
                end

                S_A1: begin
                    if (add_finish[0]) begin
                        t1 <= add_r[0];
                        // X1 = delta_t * Theta_10_10
                        mul_a[0] <= delta_t; mul_b[0] <= Theta_10_10;
                        mul_go[0] <= 1'b1;
                        state <= S_X1;
                    end
                end

                S_X1: begin
                    if (mul_finish[0]) begin
                        x1 <= mul_r[0];
                        // a = t1 + x1
                        add_a[0] <= t1; add_b[0] <= x1;
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
