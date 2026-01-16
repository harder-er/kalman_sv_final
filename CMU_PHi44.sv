`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// CMU_PHi44 - 单加 FSM 版本（无乘法�?//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi44 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_10_10,
    // 输出
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);
    // fp_* ready wires
    logic u_add0_ready;


    // 单加
    logic add_go, add_finish;
    logic [DBL_WIDTH-1:0] add_a, add_b, add_r;

    fp_adder u_add0 (.clk(clk), .rst_n(rst_n), .valid(add_go), .ready  (u_add0_ready), .finish(add_finish), .a(add_a), .b(add_b), .result(add_r));

    typedef enum logic [1:0] {S_IDLE, S_A1} st_e;
    st_e st;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;
            add_go <= 1'b0;
            a <= '0;
            valid_out <= 1'b0;
        end else begin
            add_go <= 1'b0;
            valid_out <= 1'b0;

            case (st)
                S_IDLE: begin
                    add_a <= Theta_10_10; add_b <= Q_10_10;
                    if (u_add0_ready) begin
                        add_go <= 1'b1;
                        st <= S_A1;
                    end
                end
                S_A1: if (add_finish) begin
                    a <= add_r;
                    valid_out <= 1'b1;
                    st <= S_IDLE;
                end
            endcase
        end
    end

endmodule



