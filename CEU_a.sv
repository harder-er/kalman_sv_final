`timescale 1ns / 1ps
module CEU_a #(
    parameter int DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,

    input  logic [DBL_WIDTH-1:0]   Theta_4_1,
    input  logic [DBL_WIDTH-1:0]   Theta_7_1,
    input  logic [DBL_WIDTH-1:0]   Theta_4_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_1,
    input  logic [DBL_WIDTH-1:0]   Theta_7_4,
    input  logic [DBL_WIDTH-1:0]   Theta_10_4,
    input  logic [DBL_WIDTH-1:0]   Theta_7_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_7,
    input  logic [DBL_WIDTH-1:0]   Theta_10_10,
    input  logic [DBL_WIDTH-1:0]   Q_1_1,
    input  logic [DBL_WIDTH-1:0]   R_1_1,

    input  logic [DBL_WIDTH-1:0]   dt_1,
    input  logic [DBL_WIDTH-1:0]   dt_2,
    input  logic [DBL_WIDTH-1:0]   dt_3,
    input  logic [DBL_WIDTH-1:0]   dt_4,
    input  logic [DBL_WIDTH-1:0]   dt_5,
    input  logic [DBL_WIDTH-1:0]   dt_6,

    output logic [DBL_WIDTH-1:0]   a_out,
    output logic                   valid_out
);

    localparam logic [DBL_WIDTH-1:0] C_3 = 64'h4008_0000_0000_0000; // 3.0
    localparam logic [DBL_WIDTH-1:0] C_4 = 64'h4010_0000_0000_0000; // 4.0

    // ----------------------------
    // stage regs
    // ----------------------------
    logic [DBL_WIDTH-1:0] A1, A4;
    logic [DBL_WIDTH-1:0] M1, M2, M3;
    logic [DBL_WIDTH-1:0] A2, A3;
    logic [DBL_WIDTH-1:0] X1, X2, X3, X4, X5, X6;
    logic [DBL_WIDTH-1:0] T1, T2, T3, T4;
    logic [DBL_WIDTH-1:0] T5, T6;

    // wires + finish
    logic [DBL_WIDTH-1:0] A1_w, A4_w; logic fin_A1, fin_A4;
    logic [DBL_WIDTH-1:0] M1_w, M2_w, M3_w; logic fin_M1, fin_M2, fin_M3;
    logic [DBL_WIDTH-1:0] A2_w, A3_w; logic fin_A2, fin_A3;
    logic [DBL_WIDTH-1:0] X1_w, X2_w, X3_w, X4_w, X5_w, X6_w;
    logic fin_X1, fin_X2, fin_X3, fin_X4, fin_X5, fin_X6;
    logic [DBL_WIDTH-1:0] T1_w, T2_w; logic fin_T1, fin_T2;
    logic [DBL_WIDTH-1:0] T3_w, T4_w; logic fin_T3, fin_T4;
    logic [DBL_WIDTH-1:0] T5_w, T6_w; logic fin_T5, fin_T6;
    logic [DBL_WIDTH-1:0] OUT_w;      logic fin_OUT;

    // valid pulses
    logic v_s1, v_s2, v_s3, v_s4, v_s5, v_s6, v_s7, v_out;

    // done flags
    logic dA1, dA4;
    logic dM1, dM2, dM3;
    logic dA2, dA3;
    logic dX1, dX2, dX3, dX4, dX5, dX6;
    logic dT1, dT2, dT3, dT4;
    logic dT5, dT6;

    // ----------------------------
    // operators
    // ----------------------------
    fp_adder u_add_A1 (.clk(clk), .valid(v_s1), .finish(fin_A1), .a(Theta_7_1), .b(Theta_4_4), .result(A1_w));
    fp_adder u_add_A4 (.clk(clk), .valid(v_s1), .finish(fin_A4), .a(Q_1_1),     .b(R_1_1),     .result(A4_w));

    fp_multiplier u_mul_M1 (.clk(clk), .valid(v_s2), .finish(fin_M1), .a(C_3), .b(Theta_7_4),  .result(M1_w));
    fp_multiplier u_mul_M2 (.clk(clk), .valid(v_s2), .finish(fin_M2), .a(C_4), .b(Theta_10_4), .result(M2_w));
    fp_multiplier u_mul_M3 (.clk(clk), .valid(v_s2), .finish(fin_M3), .a(C_3), .b(Theta_7_7),  .result(M3_w));

    fp_adder u_add_A2 (.clk(clk), .valid(v_s3), .finish(fin_A2), .a(Theta_10_1), .b(M1), .result(A2_w));
    fp_adder u_add_A3 (.clk(clk), .valid(v_s3), .finish(fin_A3), .a(M2),         .b(M3), .result(A3_w));

    fp_multiplier u_mul_X1 (.clk(clk), .valid(v_s4), .finish(fin_X1), .a(dt_1), .b(Theta_4_1),  .result(X1_w));
    fp_multiplier u_mul_X2 (.clk(clk), .valid(v_s4), .finish(fin_X2), .a(dt_2), .b(A1),         .result(X2_w));
    fp_multiplier u_mul_X3 (.clk(clk), .valid(v_s4), .finish(fin_X3), .a(dt_3), .b(A2),         .result(X3_w));
    fp_multiplier u_mul_X4 (.clk(clk), .valid(v_s4), .finish(fin_X4), .a(dt_4), .b(A3),         .result(X4_w));
    fp_multiplier u_mul_X5 (.clk(clk), .valid(v_s4), .finish(fin_X5), .a(dt_5), .b(Theta_10_7), .result(X5_w));
    fp_multiplier u_mul_X6 (.clk(clk), .valid(v_s4), .finish(fin_X6), .a(dt_6), .b(Theta_10_10),.result(X6_w));

    fp_adder u_add_T1 (.clk(clk), .valid(v_s5), .finish(fin_T1), .a(Theta_10_7), .b(X1), .result(T1_w));
    fp_adder u_add_T2 (.clk(clk), .valid(v_s5), .finish(fin_T2), .a(X2),         .b(X3), .result(T2_w));

    fp_adder u_add_T3 (.clk(clk), .valid(v_s6), .finish(fin_T3), .a(X4), .b(X5), .result(T3_w));
    fp_adder u_add_T4 (.clk(clk), .valid(v_s6), .finish(fin_T4), .a(X6), .b(A4), .result(T4_w));

    fp_adder u_add_T5 (.clk(clk), .valid(v_s7), .finish(fin_T5), .a(T1), .b(T2), .result(T5_w));
    fp_adder u_add_T6 (.clk(clk), .valid(v_s7), .finish(fin_T6), .a(T3), .b(T4), .result(T6_w));

    fp_adder u_add_out (.clk(clk), .valid(v_out), .finish(fin_OUT), .a(T5), .b(T6), .result(OUT_w));

    // ----------------------------
    // FSM
    // ----------------------------
    typedef enum logic [4:0] {
        S_IDLE,
        S1_FIRE, S1_WAIT,
        S2_FIRE, S2_WAIT,
        S3_FIRE, S3_WAIT,
        S4_FIRE, S4_WAIT,
        S5_FIRE, S5_WAIT,
        S6_FIRE, S6_WAIT,
        S7_FIRE, S7_WAIT,
        SOUT_FIRE, SOUT_WAIT,
        S_DONE
    } state_t;

    state_t st;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            valid_out <= 1'b0;
            a_out     <= '0;

            A1 <= '0; A4 <= '0;
            M1 <= '0; M2 <= '0; M3 <= '0;
            A2 <= '0; A3 <= '0;
            X1 <= '0; X2 <= '0; X3 <= '0; X4 <= '0; X5 <= '0; X6 <= '0;
            T1 <= '0; T2 <= '0; T3 <= '0; T4 <= '0;
            T5 <= '0; T6 <= '0;

            dA1 <= 1'b0; dA4 <= 1'b0;
            dM1 <= 1'b0; dM2 <= 1'b0; dM3 <= 1'b0;
            dA2 <= 1'b0; dA3 <= 1'b0;
            dX1 <= 1'b0; dX2 <= 1'b0; dX3 <= 1'b0; dX4 <= 1'b0; dX5 <= 1'b0; dX6 <= 1'b0;
            dT1 <= 1'b0; dT2 <= 1'b0; dT3 <= 1'b0; dT4 <= 1'b0;
            dT5 <= 1'b0; dT6 <= 1'b0;

            v_s1  <= 1'b0; v_s2  <= 1'b0; v_s3  <= 1'b0; v_s4  <= 1'b0;
            v_s5  <= 1'b0; v_s6  <= 1'b0; v_s7  <= 1'b0; v_out <= 1'b0;
        end else begin
            valid_out <= 1'b0;

            // default: clear pulses
            v_s1  <= 1'b0; v_s2  <= 1'b0; v_s3  <= 1'b0; v_s4  <= 1'b0;
            v_s5  <= 1'b0; v_s6  <= 1'b0; v_s7  <= 1'b0; v_out <= 1'b0;

            unique case (st)
                S_IDLE: begin
                    // auto-start once after reset deassert
                    dA1 <= 1'b0; dA4 <= 1'b0;
                    dM1 <= 1'b0; dM2 <= 1'b0; dM3 <= 1'b0;
                    dA2 <= 1'b0; dA3 <= 1'b0;
                    dX1 <= 1'b0; dX2 <= 1'b0; dX3 <= 1'b0; dX4 <= 1'b0; dX5 <= 1'b0; dX6 <= 1'b0;
                    dT1 <= 1'b0; dT2 <= 1'b0; dT3 <= 1'b0; dT4 <= 1'b0;
                    dT5 <= 1'b0; dT6 <= 1'b0;
                    st  <= S1_FIRE;
                end

                S1_FIRE: begin v_s1 <= 1'b1; st <= S1_WAIT; end
                S1_WAIT: begin
                    if (fin_A1) begin A1 <= A1_w; dA1 <= 1'b1; end
                    if (fin_A4) begin A4 <= A4_w; dA4 <= 1'b1; end
                    if (dA1 && dA4) st <= S2_FIRE;
                end

                S2_FIRE: begin v_s2 <= 1'b1; st <= S2_WAIT; end
                S2_WAIT: begin
                    if (fin_M1) begin M1 <= M1_w; dM1 <= 1'b1; end
                    if (fin_M2) begin M2 <= M2_w; dM2 <= 1'b1; end
                    if (fin_M3) begin M3 <= M3_w; dM3 <= 1'b1; end
                    if (dM1 && dM2 && dM3) st <= S3_FIRE;
                end

                S3_FIRE: begin v_s3 <= 1'b1; st <= S3_WAIT; end
                S3_WAIT: begin
                    if (fin_A2) begin A2 <= A2_w; dA2 <= 1'b1; end
                    if (fin_A3) begin A3 <= A3_w; dA3 <= 1'b1; end
                    if (dA2 && dA3) st <= S4_FIRE;
                end

                S4_FIRE: begin v_s4 <= 1'b1; st <= S4_WAIT; end
                S4_WAIT: begin
                    if (fin_X1) begin X1 <= X1_w; dX1 <= 1'b1; end
                    if (fin_X2) begin X2 <= X2_w; dX2 <= 1'b1; end
                    if (fin_X3) begin X3 <= X3_w; dX3 <= 1'b1; end
                    if (fin_X4) begin X4 <= X4_w; dX4 <= 1'b1; end
                    if (fin_X5) begin X5 <= X5_w; dX5 <= 1'b1; end
                    if (fin_X6) begin X6 <= X6_w; dX6 <= 1'b1; end
                    if (dX1 && dX2 && dX3 && dX4 && dX5 && dX6) st <= S5_FIRE;
                end

                S5_FIRE: begin v_s5 <= 1'b1; st <= S5_WAIT; end
                S5_WAIT: begin
                    if (fin_T1) begin T1 <= T1_w; dT1 <= 1'b1; end
                    if (fin_T2) begin T2 <= T2_w; dT2 <= 1'b1; end
                    if (dT1 && dT2) st <= S6_FIRE;
                end

                S6_FIRE: begin v_s6 <= 1'b1; st <= S6_WAIT; end
                S6_WAIT: begin
                    if (fin_T3) begin T3 <= T3_w; dT3 <= 1'b1; end
                    if (fin_T4) begin T4 <= T4_w; dT4 <= 1'b1; end
                    if (dT3 && dT4) st <= S7_FIRE;
                end

                S7_FIRE: begin v_s7 <= 1'b1; st <= S7_WAIT; end
                S7_WAIT: begin
                    if (fin_T5) begin T5 <= T5_w; dT5 <= 1'b1; end
                    if (fin_T6) begin T6 <= T6_w; dT6 <= 1'b1; end
                    if (dT5 && dT6) st <= SOUT_FIRE;
                end

                SOUT_FIRE: begin v_out <= 1'b1; st <= SOUT_WAIT; end
                SOUT_WAIT: begin
                    if (fin_OUT) begin
                        a_out     <= OUT_w;
                        valid_out <= 1'b1;   // 1-cycle pulse
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

