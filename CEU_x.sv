`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CEU_x
// Description: R-channel CEU computation
// Dependencies: fp_adder, fp_multiplier
//////////////////////////////////////////////////////////////////////////////////
module CEU_x #(
    parameter DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // Theta inputs
    input  logic [DBL_WIDTH-1:0]   Theta_1_7,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_1,
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Theta_1_4,

    // Q/R noise
    input  logic [DBL_WIDTH-1:0]   Q_1_4,
    input  logic [DBL_WIDTH-1:0]   R_1_4,

    // time factors
    input  logic [DBL_WIDTH-1:0]   delta_t,
    input  logic [DBL_WIDTH-1:0]   half_dt2,
    input  logic [DBL_WIDTH-1:0]   sixth_dt3,
    input  logic [DBL_WIDTH-1:0]   five12_dt4,
    input  logic [DBL_WIDTH-1:0]   one12_dt5,

    // outputs
    output logic [DBL_WIDTH-1:0]   x,
    output logic                   valid_out
);
    // fp_* ready wires
    logic U1_add_A1_ready;
    logic U1_add_A4_ready;
    logic U1_mul_M1_ready;
    logic U1_mul_M2_ready;
    logic U1_mul_M3_ready;
    logic U2_add_A2_ready;
    logic U2_add_A3_ready;
    logic U2_mul_X1_ready;
    logic U2_mul_X2_ready;
    logic U2_mul_X3_ready;
    logic U2_mul_X4_ready;
    logic U2_mul_X5_ready;
    logic U3_add_T1_ready;
    logic U3_add_T2_ready;
    logic U3_add_T3_ready;
    logic U3_add_T4_ready;
    logic U3_add_T5_ready;
    logic U3_add_x_ready;

    // stage valids
    logic v_s1, v_s2, v_s3, v_s4, v_s5, v_s6, v_s7;

    // finish signals
    logic fin_A1, fin_A4;
    logic fin_M1, fin_M2, fin_M3;
    logic fin_A2, fin_A3;
    logic fin_X1, fin_X2, fin_X3, fin_X4, fin_X5;
    logic fin_T1, fin_T2, fin_T3, fin_T4, fin_T5;
    logic fin_X;

    // unit outputs
    logic [DBL_WIDTH-1:0] A1_w, A4_w;
    logic [DBL_WIDTH-1:0] M1_w, M2_w, M3_w;
    logic [DBL_WIDTH-1:0] A2_w, A3_w;
    logic [DBL_WIDTH-1:0] X1_w, X2_w, X3_w, X4_w, X5_w;
    logic [DBL_WIDTH-1:0] T1_w, T2_w, T3_w, T4_w, T5_w;
    logic [DBL_WIDTH-1:0] x_w;

    // registered intermediates
    logic [DBL_WIDTH-1:0] A1, A4;
    logic [DBL_WIDTH-1:0] M1, M2, M3;
    logic [DBL_WIDTH-1:0] A2, A3;
    logic [DBL_WIDTH-1:0] X1, X2, X3, X4, X5;
    logic [DBL_WIDTH-1:0] T1, T2, T3, T4, T5;

    // done flags
    logic dA1, dA4;
    logic dM1, dM2, dM3;
    logic dA2, dA3;
    logic dX1, dX2, dX3, dX4, dX5;
    logic dT1, dT2, dT3, dT4, dT5;

    // ready groups
    wire s1_ready = U1_add_A1_ready & U1_add_A4_ready;
    wire s2_ready = U1_mul_M1_ready & U1_mul_M2_ready & U1_mul_M3_ready;
    wire s3_ready = U2_add_A2_ready & U2_add_A3_ready & U2_mul_X1_ready & U2_mul_X4_ready & U2_mul_X5_ready;
    wire s4_ready = U2_mul_X2_ready & U2_mul_X3_ready;
    wire s5_ready = U3_add_T1_ready & U3_add_T2_ready & U3_add_T3_ready;
    wire s6_ready = U3_add_T4_ready & U3_add_T5_ready;
    wire s7_ready = U3_add_x_ready;

    // Stage 1: A1/A4
    fp_adder U1_add_A1 (.clk(clk), .rst_n(rst_n), .valid(v_s1), .ready(U1_add_A1_ready), .finish(fin_A1),
        .a(Theta_4_4), .b(Theta_1_7), .result(A1_w)
    );
    fp_adder U1_add_A4 (.clk(clk), .rst_n(rst_n), .valid(v_s1), .ready(U1_add_A4_ready), .finish(fin_A4),
        .a(Q_1_4), .b(R_1_4), .result(A4_w)
    );

    // Stage 2: M1/M2/M3
    fp_multiplier U1_mul_M1 (.clk(clk), .rst_n(rst_n), .valid(v_s2), .ready(U1_mul_M1_ready), .finish(fin_M1),
        .a(64'h4008_0000_0000_0000), .b(Theta_7_4), .result(M1_w)
    );
    fp_multiplier U1_mul_M2 (.clk(clk), .rst_n(rst_n), .valid(v_s2), .ready(U1_mul_M2_ready), .finish(fin_M2),
        .a(64'h4008_0000_0000_0000), .b(Theta_7_7), .result(M2_w)
    );
    fp_multiplier U1_mul_M3 (.clk(clk), .rst_n(rst_n), .valid(v_s2), .ready(U1_mul_M3_ready), .finish(fin_M3),
        .a(64'h4010_0000_0000_0000), .b(Theta_10_4), .result(M3_w)
    );

    // Stage 3: A2/A3/X1/X4/X5
    fp_adder U2_add_A2 (.clk(clk), .rst_n(rst_n), .valid(v_s3), .ready(U2_add_A2_ready), .finish(fin_A2),
        .a(Theta_10_1), .b(M1), .result(A2_w)
    );
    fp_adder U2_add_A3 (.clk(clk), .rst_n(rst_n), .valid(v_s3), .ready(U2_add_A3_ready), .finish(fin_A3),
        .a(M2), .b(M3), .result(A3_w)
    );
    fp_multiplier U2_mul_X1 (.clk(clk), .rst_n(rst_n), .valid(v_s3), .ready(U2_mul_X1_ready), .finish(fin_X1),
        .a(delta_t), .b(A1), .result(X1_w)
    );
    fp_multiplier U2_mul_X4 (.clk(clk), .rst_n(rst_n), .valid(v_s3), .ready(U2_mul_X4_ready), .finish(fin_X4),
        .a(five12_dt4), .b(Theta_10_7), .result(X4_w)
    );
    fp_multiplier U2_mul_X5 (.clk(clk), .rst_n(rst_n), .valid(v_s3), .ready(U2_mul_X5_ready), .finish(fin_X5),
        .a(one12_dt5), .b(Theta_10_10), .result(X5_w)
    );

    // Stage 4: X2/X3
    fp_multiplier U2_mul_X2 (.clk(clk), .rst_n(rst_n), .valid(v_s4), .ready(U2_mul_X2_ready), .finish(fin_X2),
        .a(half_dt2), .b(A2), .result(X2_w)
    );
    fp_multiplier U2_mul_X3 (.clk(clk), .rst_n(rst_n), .valid(v_s4), .ready(U2_mul_X3_ready), .finish(fin_X3),
        .a(sixth_dt3), .b(A3), .result(X3_w)
    );

    // Stage 5: T1/T2/T3
    fp_adder U3_add_T1 (.clk(clk), .rst_n(rst_n), .valid(v_s5), .ready(U3_add_T1_ready), .finish(fin_T1),
        .a(Theta_1_4), .b(X1), .result(T1_w)
    );
    fp_adder U3_add_T2 (.clk(clk), .rst_n(rst_n), .valid(v_s5), .ready(U3_add_T2_ready), .finish(fin_T2),
        .a(X2), .b(X3), .result(T2_w)
    );
    fp_adder U3_add_T3 (.clk(clk), .rst_n(rst_n), .valid(v_s5), .ready(U3_add_T3_ready), .finish(fin_T3),
        .a(X4), .b(X5), .result(T3_w)
    );

    // Stage 6: T4/T5
    fp_adder U3_add_T4 (.clk(clk), .rst_n(rst_n), .valid(v_s6), .ready(U3_add_T4_ready), .finish(fin_T4),
        .a(T1), .b(T2), .result(T4_w)
    );
    fp_adder U3_add_T5 (.clk(clk), .rst_n(rst_n), .valid(v_s6), .ready(U3_add_T5_ready), .finish(fin_T5),
        .a(T3), .b(A4), .result(T5_w)
    );

    // Stage 7: final add
    fp_adder U3_add_x (.clk(clk), .rst_n(rst_n), .valid(v_s7), .ready(U3_add_x_ready), .finish(fin_X),
        .a(T4), .b(T5), .result(x_w)
    );

    // FSM
    typedef enum logic [4:0] {
        S_IDLE,
        S1_FIRE, S1_WAIT,
        S2_FIRE, S2_WAIT,
        S3_FIRE, S3_WAIT,
        S4_FIRE, S4_WAIT,
        S5_FIRE, S5_WAIT,
        S6_FIRE, S6_WAIT,
        S7_FIRE, S7_WAIT
    } state_t;

    state_t st;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            valid_out <= 1'b0;
            x         <= '0;

            A1 <= '0; A4 <= '0;
            M1 <= '0; M2 <= '0; M3 <= '0;
            A2 <= '0; A3 <= '0;
            X1 <= '0; X2 <= '0; X3 <= '0; X4 <= '0; X5 <= '0;
            T1 <= '0; T2 <= '0; T3 <= '0; T4 <= '0; T5 <= '0;

            dA1 <= 1'b0; dA4 <= 1'b0;
            dM1 <= 1'b0; dM2 <= 1'b0; dM3 <= 1'b0;
            dA2 <= 1'b0; dA3 <= 1'b0;
            dX1 <= 1'b0; dX2 <= 1'b0; dX3 <= 1'b0; dX4 <= 1'b0; dX5 <= 1'b0;
            dT1 <= 1'b0; dT2 <= 1'b0; dT3 <= 1'b0; dT4 <= 1'b0; dT5 <= 1'b0;

            v_s1 <= 1'b0; v_s2 <= 1'b0; v_s3 <= 1'b0; v_s4 <= 1'b0;
            v_s5 <= 1'b0; v_s6 <= 1'b0; v_s7 <= 1'b0;
        end else begin
            valid_out <= 1'b0;
            v_s1 <= 1'b0; v_s2 <= 1'b0; v_s3 <= 1'b0; v_s4 <= 1'b0;
            v_s5 <= 1'b0; v_s6 <= 1'b0; v_s7 <= 1'b0;

            case (st)
                S_IDLE: begin
                    dA1 <= 1'b0; dA4 <= 1'b0;
                    dM1 <= 1'b0; dM2 <= 1'b0; dM3 <= 1'b0;
                    dA2 <= 1'b0; dA3 <= 1'b0;
                    dX1 <= 1'b0; dX2 <= 1'b0; dX3 <= 1'b0; dX4 <= 1'b0; dX5 <= 1'b0;
                    dT1 <= 1'b0; dT2 <= 1'b0; dT3 <= 1'b0; dT4 <= 1'b0; dT5 <= 1'b0;
                    st  <= S1_FIRE;
                end

                S1_FIRE: begin
                    if (s1_ready) begin
                        v_s1 <= 1'b1;
                        st <= S1_WAIT;
                    end
                end
                S1_WAIT: begin
                    if (fin_A1) begin A1 <= A1_w; dA1 <= 1'b1; end
                    if (fin_A4) begin A4 <= A4_w; dA4 <= 1'b1; end
                    if (dA1 && dA4) st <= S2_FIRE;
                end

                S2_FIRE: begin
                    if (s2_ready) begin
                        v_s2 <= 1'b1;
                        st <= S2_WAIT;
                    end
                end
                S2_WAIT: begin
                    if (fin_M1) begin M1 <= M1_w; dM1 <= 1'b1; end
                    if (fin_M2) begin M2 <= M2_w; dM2 <= 1'b1; end
                    if (fin_M3) begin M3 <= M3_w; dM3 <= 1'b1; end
                    if (dM1 && dM2 && dM3) st <= S3_FIRE;
                end

                S3_FIRE: begin
                    if (s3_ready) begin
                        v_s3 <= 1'b1;
                        st <= S3_WAIT;
                    end
                end
                S3_WAIT: begin
                    if (fin_A2) begin A2 <= A2_w; dA2 <= 1'b1; end
                    if (fin_A3) begin A3 <= A3_w; dA3 <= 1'b1; end
                    if (fin_X1) begin X1 <= X1_w; dX1 <= 1'b1; end
                    if (fin_X4) begin X4 <= X4_w; dX4 <= 1'b1; end
                    if (fin_X5) begin X5 <= X5_w; dX5 <= 1'b1; end
                    if (dA2 && dA3 && dX1 && dX4 && dX5) st <= S4_FIRE;
                end

                S4_FIRE: begin
                    if (s4_ready) begin
                        v_s4 <= 1'b1;
                        st <= S4_WAIT;
                    end
                end
                S4_WAIT: begin
                    if (fin_X2) begin X2 <= X2_w; dX2 <= 1'b1; end
                    if (fin_X3) begin X3 <= X3_w; dX3 <= 1'b1; end
                    if (dX2 && dX3) st <= S5_FIRE;
                end

                S5_FIRE: begin
                    if (s5_ready) begin
                        v_s5 <= 1'b1;
                        st <= S5_WAIT;
                    end
                end
                S5_WAIT: begin
                    if (fin_T1) begin T1 <= T1_w; dT1 <= 1'b1; end
                    if (fin_T2) begin T2 <= T2_w; dT2 <= 1'b1; end
                    if (fin_T3) begin T3 <= T3_w; dT3 <= 1'b1; end
                    if (dT1 && dT2 && dT3) st <= S6_FIRE;
                end

                S6_FIRE: begin
                    if (s6_ready) begin
                        v_s6 <= 1'b1;
                        st <= S6_WAIT;
                    end
                end
                S6_WAIT: begin
                    if (fin_T4) begin T4 <= T4_w; dT4 <= 1'b1; end
                    if (fin_T5) begin T5 <= T5_w; dT5 <= 1'b1; end
                    if (dT4 && dT5) st <= S7_FIRE;
                end

                S7_FIRE: begin
                    if (s7_ready) begin
                        v_s7 <= 1'b1;
                        st <= S7_WAIT;
                    end
                end
                S7_WAIT: begin
                    if (fin_X) begin
                        x <= x_w;
                        valid_out <= 1'b1;
                        st <= S_IDLE;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
