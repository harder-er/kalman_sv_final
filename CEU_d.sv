`timescale 1ns / 1ps
module CEU_d #(
    parameter int DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,

    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_4_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Q_4_4,
    input  logic [DBL_WIDTH-1:0]   R_4_4,

    input  logic [DBL_WIDTH-1:0]   delta_t2,
    input  logic [DBL_WIDTH-1:0]   delta_t_sq,
    input  logic [DBL_WIDTH-1:0]   delta_t_hcu,
    input  logic [DBL_WIDTH-1:0]   delta_t_qr,

    output logic [DBL_WIDTH-1:0]   d,
    output logic                   valid_out
);

    // ----------------------------
    // stage registers
    // ----------------------------
    logic [DBL_WIDTH-1:0] A1, A2;
    logic [DBL_WIDTH-1:0] X1, X2, X3, X4;
    logic [DBL_WIDTH-1:0] T1, T2, T3;
    logic [DBL_WIDTH-1:0] T4;

    // ----------------------------
    // operator wires
    // ----------------------------
    logic [DBL_WIDTH-1:0] A1_w, A2_w;
    logic fin_A1, fin_A2;

    logic [DBL_WIDTH-1:0] X1_w, X2_w, X3_w, X4_w;
    logic fin_X1, fin_X2, fin_X3, fin_X4;

    logic [DBL_WIDTH-1:0] T1_w, T2_w, T3_w;
    logic fin_T1, fin_T2, fin_T3;

    logic [DBL_WIDTH-1:0] T4_w;
    logic fin_T4;

    logic [DBL_WIDTH-1:0] d_w;
    logic fin_D;

    // ----------------------------
    // valid pulses (1-cycle)
    // ----------------------------
    logic v_add1, v_mul, v_add2, v_add3, v_add_out;

    // ----------------------------
    // done flags (capture finish pulses)
    // ----------------------------
    logic done_A1, done_A2;
    logic done_X1, done_X2, done_X3, done_X4;
    logic done_T1, done_T2, done_T3;
    logic done_T4;

    // ----------------------------
    // FP operators
    // ----------------------------
    fp_adder u_add_A1 (.clk(clk), .valid(v_add1), .finish(fin_A1),
        .a(Theta_10_7), .b(Theta_7_4), .result(A1_w)
    );

    fp_adder u_add_A2 (.clk(clk), .valid(v_add1), .finish(fin_A2),
        .a(Q_4_4), .b(R_4_4), .result(A2_w)
    );

    fp_multiplier u_mul_X1 (.clk(clk), .valid(v_mul), .finish(fin_X1),
        .a(delta_t2), .b(Theta_7_4), .result(X1_w)
    );

    fp_multiplier u_mul_X2 (.clk(clk), .valid(v_mul), .finish(fin_X2),
        .a(delta_t_sq), .b(Theta_10_4), .result(X2_w)
    );

    fp_multiplier u_mul_X3 (.clk(clk), .valid(v_mul), .finish(fin_X3),
        .a(delta_t_hcu), .b(A1), .result(X3_w)
    );

    fp_multiplier u_mul_X4 (.clk(clk), .valid(v_mul), .finish(fin_X4),
        .a(delta_t_qr), .b(Theta_10_10), .result(X4_w)
    );

    fp_adder u_add_T1 (.clk(clk), .valid(v_add2), .finish(fin_T1),
        .a(Theta_4_4), .b(X1), .result(T1_w)
    );

    fp_adder u_add_T2 (.clk(clk), .valid(v_add2), .finish(fin_T2),
        .a(X2), .b(X3), .result(T2_w)
    );

    fp_adder u_add_T3 (.clk(clk), .valid(v_add2), .finish(fin_T3),
        .a(A2), .b(X4), .result(T3_w)
    );

    fp_adder u_add_T4 (.clk(clk), .valid(v_add3), .finish(fin_T4),
        .a(T1), .b(T2), .result(T4_w)
    );

    fp_adder u_add_out (.clk(clk), .valid(v_add_out), .finish(fin_D),
        .a(T3), .b(T4), .result(d_w)
    );

    // ----------------------------
    // FSM
    // ----------------------------
    typedef enum logic [3:0] {
        S_IDLE,
        S1_FIRE, S1_WAIT,
        S2_FIRE, S2_WAIT,
        S3_FIRE, S3_WAIT,
        S4_FIRE, S4_WAIT,
        S5_FIRE, S5_WAIT,
        S_DONE
    } state_t;

    state_t st;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            valid_out <= 1'b0;
            d         <= '0;

            A1 <= '0; A2 <= '0;
            X1 <= '0; X2 <= '0; X3 <= '0; X4 <= '0;
            T1 <= '0; T2 <= '0; T3 <= '0; T4 <= '0;

            done_A1 <= 1'b0; done_A2 <= 1'b0;
            done_X1 <= 1'b0; done_X2 <= 1'b0; done_X3 <= 1'b0; done_X4 <= 1'b0;
            done_T1 <= 1'b0; done_T2 <= 1'b0; done_T3 <= 1'b0;
            done_T4 <= 1'b0;

            v_add1    <= 1'b0;
            v_mul     <= 1'b0;
            v_add2    <= 1'b0;
            v_add3    <= 1'b0;
            v_add_out <= 1'b0;
        end else begin
            valid_out <= 1'b0;

            // default: pulse valids low (we raise them only in *_FIRE states)
            v_add1    <= 1'b0;
            v_mul     <= 1'b0;
            v_add2    <= 1'b0;
            v_add3    <= 1'b0;
            v_add_out <= 1'b0;

            unique case (st)
                S_IDLE: begin
                    // auto-start once after reset deassert
                    done_A1 <= 1'b0; done_A2 <= 1'b0;
                    done_X1 <= 1'b0; done_X2 <= 1'b0; done_X3 <= 1'b0; done_X4 <= 1'b0;
                    done_T1 <= 1'b0; done_T2 <= 1'b0; done_T3 <= 1'b0;
                    done_T4 <= 1'b0;
                    st <= S1_FIRE;
                end

                S1_FIRE: begin
                    v_add1 <= 1'b1;      // 1-cycle pulse
                    st     <= S1_WAIT;
                end

                S1_WAIT: begin
                    if (fin_A1) begin A1 <= A1_w; done_A1 <= 1'b1; end
                    if (fin_A2) begin A2 <= A2_w; done_A2 <= 1'b1; end
                    if (done_A1 && done_A2) st <= S2_FIRE;
                end

                S2_FIRE: begin
                    v_mul <= 1'b1;
                    st    <= S2_WAIT;
                end

                S2_WAIT: begin
                    if (fin_X1) begin X1 <= X1_w; done_X1 <= 1'b1; end
                    if (fin_X2) begin X2 <= X2_w; done_X2 <= 1'b1; end
                    if (fin_X3) begin X3 <= X3_w; done_X3 <= 1'b1; end
                    if (fin_X4) begin X4 <= X4_w; done_X4 <= 1'b1; end
                    if (done_X1 && done_X2 && done_X3 && done_X4) st <= S3_FIRE;
                end

                S3_FIRE: begin
                    v_add2 <= 1'b1;
                    st     <= S3_WAIT;
                end

                S3_WAIT: begin
                    if (fin_T1) begin T1 <= T1_w; done_T1 <= 1'b1; end
                    if (fin_T2) begin T2 <= T2_w; done_T2 <= 1'b1; end
                    if (fin_T3) begin T3 <= T3_w; done_T3 <= 1'b1; end
                    if (done_T1 && done_T2 && done_T3) st <= S4_FIRE;
                end

                S4_FIRE: begin
                    v_add3 <= 1'b1;
                    st     <= S4_WAIT;
                end

                S4_WAIT: begin
                    if (fin_T4) begin T4 <= T4_w; done_T4 <= 1'b1; end
                    if (done_T4) st <= S5_FIRE;
                end

                S5_FIRE: begin
                    v_add_out <= 1'b1;
                    st        <= S5_WAIT;
                end

                S5_WAIT: begin
                    if (fin_D) begin
                        d         <= d_w;
                        valid_out <= 1'b1; // 1-cycle pulse
                        st        <= S_DONE;
                    end
                end

                S_DONE: begin
                    // hold results; re-run by toggling rst_n
                    st <= S_DONE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

