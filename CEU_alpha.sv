`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: CEU_alpha (fixed)
// Description: alpha = in1*in2 - in3*in3
// Notes:
//   - no input valid port, so this module auto-triggers when inputs change and are known.
//   - latches multiplier results and aligns finish pulses (finish may be 1-cycle).
//////////////////////////////////////////////////////////////////////////////////
module CEU_alpha #(
    parameter int DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic [DBL_WIDTH-1:0]   in1,
    input  logic [DBL_WIDTH-1:0]   in2,
    input  logic [DBL_WIDTH-1:0]   in3,
    output logic [DBL_WIDTH-1:0]   out,
    output logic                   valid_out
);

    // -----------------------------
    // helpers: detect X/Z on inputs
    // -----------------------------
    function automatic logic is_known(input logic [DBL_WIDTH-1:0] x);
        // reduction XOR returns X if any bit is X/Z
        is_known = (^x !== 1'bx);
    endfunction

    logic inputs_known;
    assign inputs_known = is_known(in1) && is_known(in2) && is_known(in3);

    // -----------------------------
    // input change detect (4-state)
    // -----------------------------
    logic [DBL_WIDTH-1:0] last_in1, last_in2, last_in3;
    logic                have_last;

    wire new_sample = (!have_last) ||
                      (in1 !== last_in1) ||
                      (in2 !== last_in2) ||
                      (in3 !== last_in3);

    // -----------------------------
    // internal FSM
    // -----------------------------
    typedef enum logic [1:0] {S_IDLE, S_WAIT_MUL, S_WAIT_SUB} state_t;
    state_t st;

    // latched operands (freeze for one transaction)
    logic [DBL_WIDTH-1:0] in1_lat, in2_lat, in3_lat;

    // multiplier raw outputs
    wire  [DBL_WIDTH-1:0] m1_w, m2_w;
    logic                 mul_valid_r;

    // latch multiplier results when finish arrives
    logic [DBL_WIDTH-1:0] m1_lat, m2_lat;
    logic                 mul1_done, mul2_done;

    // sub
    wire  [DBL_WIDTH-1:0] diff_w;
    logic                 sub_valid_r;

    // finish pulses from your wrappers
    wire finish_mul1, finish_mul2, finish_sub;

    // -----------------------------
    // datapath instances
    // -----------------------------
    fp_multiplier U_mul1 (.clk(clk),
        .a      (in1_lat),
        .b      (in2_lat),
        .valid  (mul_valid_r),
        .finish (finish_mul1),
        .result (m1_w)
    );

    fp_multiplier U_mul2 (.clk(clk),
        .a      (in3_lat),
        .b      (in3_lat),
        .valid  (mul_valid_r),
        .finish (finish_mul2),
        .result (m2_w)
    );

    fp_suber U_sub (.clk(clk),
        .a      (m1_lat),
        .b      (m2_lat),
        .valid  (sub_valid_r),
        .finish (finish_sub),
        .result (diff_w)
    );

    // -----------------------------
    // control
    // -----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;

            mul_valid_r <= 1'b0;
            sub_valid_r <= 1'b0;

            mul1_done  <= 1'b0;
            mul2_done  <= 1'b0;

            in1_lat    <= '0;
            in2_lat    <= '0;
            in3_lat    <= '0;

            m1_lat     <= '0;
            m2_lat     <= '0;

            out        <= '0;
            valid_out  <= 1'b0;

            last_in1   <= '0;
            last_in2   <= '0;
            last_in3   <= '0;
            have_last  <= 1'b0;
        end else begin
            // default pulses
            mul_valid_r <= 1'b0;
            sub_valid_r <= 1'b0;
            valid_out   <= 1'b0;

            unique case (st)
                S_IDLE: begin
                    // only start when inputs are known and changed
                    if (inputs_known && new_sample) begin
                        // latch "sample"
                        in1_lat   <= in1;
                        in2_lat   <= in2;
                        in3_lat   <= in3;

                        // remember last inputs
                        last_in1  <= in1;
                        last_in2  <= in2;
                        last_in3  <= in3;
                        have_last <= 1'b1;

                        // reset done flags
                        mul1_done <= 1'b0;
                        mul2_done <= 1'b0;

                        // fire multipliers (1-cycle pulse)
                        mul_valid_r <= 1'b1;
                        st          <= S_WAIT_MUL;
                    end
                end

                S_WAIT_MUL: begin
                    // latch each result when its finish pulse arrives
                    if (finish_mul1) begin
                        m1_lat    <= m1_w;
                        mul1_done <= 1'b1;
                    end
                    if (finish_mul2) begin
                        m2_lat    <= m2_w;
                        mul2_done <= 1'b1;
                    end

                    // when both done (allow same-cycle finish)
                    if ( (mul1_done || finish_mul1) && (mul2_done || finish_mul2) ) begin
                        sub_valid_r <= 1'b1; // fire subtract
                        st          <= S_WAIT_SUB;
                    end
                end

                S_WAIT_SUB: begin
                    if (finish_sub) begin
                        out       <= diff_w;
                        valid_out <= 1'b1;   // 1-cycle pulse
                        st        <= S_IDLE;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

