`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// CMU_PHi43 - 单乘单加 FSM 版本
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi43 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_10_7,
    // 时间参数
    input  logic [DBL_WIDTH-1:0]   delta_t,
    // 输出
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);
    // fp_* ready wires
    logic u_mul_ready;
    logic u_add_ready;


    // 单乘 + 单加
    logic mul_go, mul_finish;
    logic [DBL_WIDTH-1:0] mul_a, mul_b, mul_r;
    logic add_go, add_finish;
    logic [DBL_WIDTH-1:0] add_a, add_b, add_r;

    fp_multiplier u_mul (.clk(clk), .rst_n(rst_n), .valid(mul_go), .ready  (u_mul_ready), .finish(mul_finish), .a(mul_a), .b(mul_b), .result(mul_r));
    fp_adder u_add (.clk(clk), .rst_n(rst_n), .valid(add_go), .ready  (u_add_ready), .finish(add_finish), .a(add_a), .b(add_b), .result(add_r));

    typedef enum logic [1:0] {S_IDLE, S_A1, S_X1, S_SUM} st_e;
    st_e st;
    logic [DBL_WIDTH-1:0] t1, x1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;
            mul_go <= 1'b0; add_go <= 1'b0;
            t1 <= '0; x1 <= '0; a <= '0;
            valid_out <= 1'b0;
        end else begin
            mul_go <= 1'b0; add_go <= 1'b0;
            valid_out <= 1'b0;

            case (st)
                S_IDLE: begin
                    add_a <= Theta_10_7; add_b <= Q_10_7;
                    if (u_add_ready) begin
                        add_go <= 1'b1;
                        st <= S_A1;
                    end
                end
                S_A1: if (add_finish) begin
                    t1 <= add_r;
                    mul_a <= delta_t; mul_b <= Theta_10_10;
                    if (u_mul_ready) begin
                        mul_go <= 1'b1;
                        st <= S_X1;
                    end
                end
                S_X1: if (mul_finish) begin
                    x1 <= mul_r;
                    add_a <= t1; add_b <= x1;
                    if (u_add_ready) begin
                        add_go <= 1'b1;
                        st <= S_SUM;
                    end
                end
                S_SUM: if (add_finish) begin
                    a <= add_r;
                    valid_out <= 1'b1;
                    st <= S_IDLE;
                end
            endcase
        end
    end

endmodule




