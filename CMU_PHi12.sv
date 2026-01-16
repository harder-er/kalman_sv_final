`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Shared 2-mul + 2-add FSM version to reduce FP IP usage
//////////////////////////////////////////////////////////////////////////////////
module CMU_PHi12 #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    // 输入
    input  logic [DBL_WIDTH-1:0]   Theta_1_4,
    input  logic [DBL_WIDTH-1:0]   Theta_1_1,
    input  logic [DBL_WIDTH-1:0]   Theta_1_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_1_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_10,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_10,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_1_4,
    // 时间参数
    input  logic [DBL_WIDTH-1:0]   delta_t,
    input  logic [DBL_WIDTH-1:0]   half_dt2,
    input  logic [DBL_WIDTH-1:0]   sixth_dt3,
    input  logic [DBL_WIDTH-1:0]   five12_dt4,
    input  logic [DBL_WIDTH-1:0]   twleve_dt5,
    // 输出
    output logic [DBL_WIDTH-1:0]   a,
    output logic                   valid_out
);
    // fp_* ready wires
    logic u_mul0_ready;
    logic u_mul1_ready;
    logic u_add0_ready;
    logic u_add1_ready;


    // 2 路乘 + 2 路加共享单元
    logic mul_go [0:1], mul_finish [0:1];
    logic [DBL_WIDTH-1:0] mul_a [0:1], mul_b [0:1], mul_r [0:1];
    logic add_go [0:1], add_finish [0:1];
    logic [DBL_WIDTH-1:0] add_a [0:1], add_b [0:1], add_r [0:1];

    fp_multiplier u_mul0 (.clk(clk), .rst_n(rst_n), .valid(mul_go[0]), .ready  (u_mul0_ready), .finish(mul_finish[0]), .a(mul_a[0]), .b(mul_b[0]), .result(mul_r[0]));
    fp_multiplier u_mul1 (.clk(clk), .rst_n(rst_n), .valid(mul_go[1]), .ready  (u_mul1_ready), .finish(mul_finish[1]), .a(mul_a[1]), .b(mul_b[1]), .result(mul_r[1]));
    fp_adder u_add0 (.clk(clk), .rst_n(rst_n), .valid(add_go[0]), .ready  (u_add0_ready), .finish(add_finish[0]), .a(add_a[0]), .b(add_b[0]), .result(add_r[0]));
    fp_adder u_add1 (.clk(clk), .rst_n(rst_n), .valid(add_go[1]), .ready  (u_add1_ready), .finish(add_finish[1]), .a(add_a[1]), .b(add_b[1]), .result(add_r[1]));

    // 中间寄存�?    
    logic [DBL_WIDTH-1:0] m1, m2, m3;
    logic [DBL_WIDTH-1:0] a1, a2, a3, a4;
    logic [DBL_WIDTH-1:0] x1, x2, x3, x4, x5;
    logic [DBL_WIDTH-1:0] t1, t2, t3, t4, t5;

    typedef enum logic [3:0] {
        S_IDLE,
        S_M1M2, S_M3,
        S_A12, S_A34,
        S_X12, S_X34, S_X5,
        S_T2T3, S_T4T5, S_FINAL
    } state_e;

    state_e state;
    logic done_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            {m1,m2,m3,a1,a2,a3,a4,x1,x2,x3,x4,x5,t1,t2,t3,t4,t5,a} <= '{default:'0};
            done_pipe <= 1'b0;
        end else begin
            {mul_go[0], mul_go[1], add_go[0], add_go[1]} <= 4'b0;
            done_pipe <= 1'b0;

            case (state)
                S_IDLE: begin
                    // 常数乘法 M1=3*Θ4_7, M2=3*Θ7_7
                    mul_a[0] <= 64'h4008_0000_0000_0000; mul_b[0] <= Theta_4_7;
                    mul_a[1] <= 64'h4008_0000_0000_0000; mul_b[1] <= Theta_7_7;
                    if (u_mul0_ready && u_mul1_ready) begin
                        mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                        state <= S_M1M2;
                    end
                end

                S_M1M2: begin
                    if (mul_finish[0]) m1 <= mul_r[0];
                    if (mul_finish[1]) m2 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        mul_a[0] <= 64'h4010_0000_0000_0000; // 4.0
                        mul_b[0] <= Theta_4_10;
                        if (u_mul0_ready) begin
                            mul_go[0] <= 1'b1;
                            state <= S_M3;
                        end
                    end
                end

                S_M3: begin
                    if (mul_finish[0]) begin
                        m3 <= mul_r[0];
                        // A1 = Θ1_1 + Q1_4, A2 = Θ1_7 + Θ1_4
                        add_a[0] <= Theta_1_1; add_b[0] <= Q_1_4;
                        add_a[1] <= Theta_1_7; add_b[1] <= Theta_1_4;
                        if (u_add0_ready && u_add1_ready) begin
                            add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                            state <= S_A12;
                        end
                    end
                end

                S_A12: begin
                    if (add_finish[0]) a1 <= add_r[0];
                    if (add_finish[1]) a2 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        add_a[0] <= Theta_1_10; add_b[0] <= m1; // A3
                        add_a[1] <= m2;         add_b[1] <= m3; // A4
                        if (u_add0_ready && u_add1_ready) begin
                            add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                            state <= S_A34;
                        end
                    end
                end

                S_A34: begin
                    if (add_finish[0]) a3 <= add_r[0];
                    if (add_finish[1]) a4 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // X1 = delta_t * Θ1_10
                        mul_a[0] <= delta_t; mul_b[0] <= Theta_1_10;
                        // X2 = half_dt2 * a2
                        mul_a[1] <= half_dt2; mul_b[1] <= a2;
                        if (u_mul0_ready && u_mul1_ready) begin
                            mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                            state <= S_X12;
                        end
                    end
                end

                S_X12: begin
                    if (mul_finish[0]) x1 <= mul_r[0];
                    if (mul_finish[1]) x2 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // X3 = sixth_dt3 * a3, X4 = five12_dt4 * a4
                        mul_a[0] <= sixth_dt3; mul_b[0] <= a3;
                        mul_a[1] <= five12_dt4; mul_b[1] <= a4;
                        if (u_mul0_ready && u_mul1_ready) begin
                            mul_go[0] <= 1'b1; mul_go[1] <= 1'b1;
                            state <= S_X34;
                        end
                    end
                end

                S_X34: begin
                    if (mul_finish[0]) x3 <= mul_r[0];
                    if (mul_finish[1]) x4 <= mul_r[1];
                    if (mul_finish[0] && mul_finish[1]) begin
                        // X5 = twleve_dt5 * Θ7_10
                        mul_a[0] <= twleve_dt5; mul_b[0] <= Theta_7_10;
                        if (u_mul0_ready) begin
                            mul_go[0] <= 1'b1;
                            state <= S_X5;
                        end
                    end
                end

                S_X5: begin
                    if (mul_finish[0]) begin
                        x5 <= mul_r[0];
                        // T1 = a1 + x1, T2 = x2 + x3
                        add_a[0] <= a1; add_b[0] <= x1;
                        add_a[1] <= x2; add_b[1] <= x3;
                        if (u_add0_ready && u_add1_ready) begin
                            add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                            state <= S_T2T3;
                        end
                    end
                end

                S_T2T3: begin
                    if (add_finish[0]) t1 <= add_r[0];
                    if (add_finish[1]) t2 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // T3 = x4 + x5, T4 = t1 + Theta_4_10 (延用模式保持占位)
                        add_a[0] <= x4; add_b[0] <= x5;
                        add_a[1] <= t1; add_b[1] <= Theta_4_10; // 复用第二加法�?                        
                        if (u_add0_ready && u_add1_ready) begin
                            add_go[0] <= 1'b1; add_go[1] <= 1'b1;
                            state <= S_T4T5;
                        end
                    end
                end

                S_T4T5: begin
                    if (add_finish[0]) t3 <= add_r[0];
                    if (add_finish[1]) t4 <= add_r[1];
                    if (add_finish[0] && add_finish[1]) begin
                        // 最�?a = t2 + t3 (这里 t4 未用，可视作占位)
                        add_a[0] <= t2; add_b[0] <= t3;
                        if (u_add0_ready) begin
                            add_go[0] <= 1'b1;
                            state <= S_FINAL;
                        end
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




