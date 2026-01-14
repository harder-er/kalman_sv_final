`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: NoiseGenerator
// Description:
//   One-shot (enable_gen) generation of constant Q_k and R_k covariance matrices.
//   - Parameters are REAL, computed as CONSTANTS, converted to IEEE754-64 bits.
//   - On enable_gen high: write matrices once, matrices_ready goes high and holds
//     until enable_gen deasserts.
// Notes:
//   - This version is synthesizer-friendly: no runtime real arithmetic, no FP IP.
//   - Q_k example fills 3 axes of {pos, vel} 2x2 blocks: indices [0..2] and [3..5].
//     Other state dims (6..STATE_DIM-1) are set to 0 by default.
//////////////////////////////////////////////////////////////////////////////////

module NoiseGenerator #(
    parameter int  STATE_DIM   = 12,
    parameter int  MEASURE_DIM = 6,

    // IMPORTANT: use REAL typed parameters
    parameter real deltat              = 0.01,

    // Process noise variances (example)
    parameter real NOISE_VAR_POS_REAL      = 0.001, // position variance scale
    parameter real NOISE_VAR_VEL_REAL      = 0.01,  // velocity variance scale
    parameter real NOISE_VAR_POS_VEL_REAL  = 0.001, // pos-vel cross variance scale (example)

    // Measurement noise variance (diagonal)
    parameter real MEASURE_VAR_REAL        = 0.1
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             enable_gen,

    output logic [63:0]      Q_k [0:STATE_DIM-1][0:STATE_DIM-1],
    output logic [63:0]      R_k [0:MEASURE_DIM-1][0:MEASURE_DIM-1],
    output logic             matrices_ready
);

    // -------------------------
    // Constant helper: real -> IEEE754(64) bits
    // -------------------------
    function automatic logic [63:0] r2b(input real r);
        r2b = $realtobits(r);
    endfunction

    localparam logic [63:0] ZERO_FP = 64'h0;

    // -------------------------
    // Pre-compute dt powers as REAL constants
    // -------------------------
    localparam real DT_R   = deltat;
    localparam real DT2_R  = DT_R * DT_R;
    localparam real DT3_R  = DT2_R * DT_R;

    // Example continuous white-noise model constants (pos/vel only)
    localparam real Q_PP_R = (DT3_R / 3.0) * NOISE_VAR_POS_REAL;      // dt^3/3 * var_pos
    localparam real Q_PV_R = (DT2_R / 2.0) * NOISE_VAR_POS_VEL_REAL;  // dt^2/2 * var_cross
    localparam real Q_VV_R = (DT_R)        * NOISE_VAR_VEL_REAL;      // dt * var_vel

    localparam logic [63:0] Q_PP_FP = r2b(Q_PP_R);
    localparam logic [63:0] Q_PV_FP = r2b(Q_PV_R);
    localparam logic [63:0] Q_VV_FP = r2b(Q_VV_R);

    localparam logic [63:0] R_DIAG_FP = r2b(MEASURE_VAR_REAL);

    // -------------------------
    // FSM (one-shot)
    // -------------------------
    typedef enum logic [1:0] {ST_IDLE, ST_BUILD, ST_DONE} state_e;
    state_e st;

    integer i, j;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st             <= ST_IDLE;
            matrices_ready <= 1'b0;

            // clear matrices
            for (i = 0; i < STATE_DIM; i = i + 1) begin
                for (j = 0; j < STATE_DIM; j = j + 1) begin
                    Q_k[i][j] <= ZERO_FP;
                end
            end
            for (i = 0; i < MEASURE_DIM; i = i + 1) begin
                for (j = 0; j < MEASURE_DIM; j = j + 1) begin
                    R_k[i][j] <= ZERO_FP;
                end
            end
        end else begin
            unique case (st)
                ST_IDLE: begin
                    matrices_ready <= 1'b0;
                    if (enable_gen) begin
                        st <= ST_BUILD;
                    end
                end

                ST_BUILD: begin
                    // 1) clear all
                    for (i = 0; i < STATE_DIM; i = i + 1) begin
                        for (j = 0; j < STATE_DIM; j = j + 1) begin
                            Q_k[i][j] <= ZERO_FP;
                        end
                    end
                    for (i = 0; i < MEASURE_DIM; i = i + 1) begin
                        for (j = 0; j < MEASURE_DIM; j = j + 1) begin
                            R_k[i][j] <= ZERO_FP;
                        end
                    end

                    // 2) fill Q: 3 axes of (pos, vel) blocks
                    // pos indices: 0,1,2 ; vel indices: 3,4,5
                    // Q[pos][pos] = Q_PP
                    // Q[pos][vel] = Q_PV
                    // Q[vel][pos] = Q_PV
                    // Q[vel][vel] = Q_VV
                    for (int axis = 0; axis < 3; axis++) begin
                        int p, v;
                        p = axis;
                        v = axis + 3;
                        if ((p < STATE_DIM) && (v < STATE_DIM)) begin
                            Q_k[p][p] <= Q_PP_FP;
                            Q_k[p][v] <= Q_PV_FP;
                            Q_k[v][p] <= Q_PV_FP;
                            Q_k[v][v] <= Q_VV_FP;
                        end
                    end

                    // 3) fill R: diagonal
                    for (int m = 0; m < MEASURE_DIM; m++) begin
                        R_k[m][m] <= R_DIAG_FP;
                    end

                    matrices_ready <= 1'b1;
                    st             <= ST_DONE;
                end

                ST_DONE: begin
                    matrices_ready <= 1'b1;
                    // allow re-generate next time
                    if (!enable_gen) begin
                        st <= ST_IDLE;
                    end
                end

                default: begin
                    st <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
