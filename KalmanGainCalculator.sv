`timescale 1ns/1ps

// =========================================================
// TimeParamsSeq (FIXED)
//  - start only accepted in IDLE (avoid level-sensitive restart)
//  - mul_resp_ready asserted ONLY in WAIT states (avoid consuming stray responses)
// =========================================================
module TimeParamsSeq #(
    parameter int DWIDTH = 64
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  start,
    input  logic [DWIDTH-1:0]     delta_t,

    // ---------- shared MUL request ----------
    output logic                  mul_req_valid,
    input  logic                  mul_req_ready,
    output logic [DWIDTH-1:0]     mul_req_a,
    output logic [DWIDTH-1:0]     mul_req_b,

    // ---------- shared MUL response ----------
    input  logic                  mul_resp_valid,
    output logic                  mul_resp_ready,
    input  logic [DWIDTH-1:0]     mul_resp_y,

    // ---------- outputs ----------
    output logic [DWIDTH-1:0]     delta_t2,       // 2*dt
    output logic [DWIDTH-1:0]     dt2,            // dt^2

    output logic [DWIDTH-1:0]     half_dt2,       // 1/2*dt^2
    output logic [DWIDTH-1:0]     three2_dt2,     // 3/2*dt^2

    output logic [DWIDTH-1:0]     half_dt3,       // 1/2*dt^3
    output logic [DWIDTH-1:0]     three_dt3,      // 1/3*dt^3
    output logic [DWIDTH-1:0]     sixth_dt3,      // 1/6*dt^3
    output logic [DWIDTH-1:0]     two3_dt3,       // 2/3*dt^3

    output logic [DWIDTH-1:0]     quarter_dt4,    // 1/4*dt^4
    output logic [DWIDTH-1:0]     sixth_dt4,      // 1/6*dt^4
    output logic [DWIDTH-1:0]     twive_dt4,      // 1/12*dt^4
    output logic [DWIDTH-1:0]     five12_dt4,     // 5/12*dt^4

    output logic [DWIDTH-1:0]     six_dt5,        // 1/6*dt^5
    output logic [DWIDTH-1:0]     twleve_dt5,     // 1/12*dt^5

    output logic [DWIDTH-1:0]     thirtysix_dt6,  // 1/36*dt^6

    output logic                  done,           // 1-cycle pulse
    output logic                  valid           // level: coefficients ready
);

    // Constants (IEEE 754 double)
    localparam logic [DWIDTH-1:0] C_2_0    = 64'h4000_0000_0000_0000; // 2.0
    localparam logic [DWIDTH-1:0] C_0_5    = 64'h3fe0_0000_0000_0000; // 0.5
    localparam logic [DWIDTH-1:0] C_1_5    = 64'h3ff8_0000_0000_0000; // 1.5
    localparam logic [DWIDTH-1:0] C_0_333  = 64'h3fd5_5555_5555_5555; // 1/3
    localparam logic [DWIDTH-1:0] C_0_166  = 64'h3fc5_5555_5555_5555; // 1/6
    localparam logic [DWIDTH-1:0] C_0_666  = 64'h3fe5_5555_5555_5555; // 2/3
    localparam logic [DWIDTH-1:0] C_0_25   = 64'h3fd0_0000_0000_0000; // 1/4
    localparam logic [DWIDTH-1:0] C_0_0833 = 64'h3fb5_5555_5555_5555; // 1/12
    localparam logic [DWIDTH-1:0] C_0_416  = 64'h3fdb_6db6_db6d_b6db; // 5/12
    localparam logic [DWIDTH-1:0] C_0_0277 = 64'h3f72_2222_2222_2222; // 1/36

    logic [DWIDTH-1:0] dt_reg;
    logic [DWIDTH-1:0] dt_sq, dt_cu, dt_qu, dt_qi, dt_sx;

    typedef enum logic [5:0] {
        ST_IDLE,

        ST_REQ_DT_SQ,          ST_WAIT_DT_SQ,
        ST_REQ_2DT,            ST_WAIT_2DT,

        ST_REQ_HALF_DT2,       ST_WAIT_HALF_DT2,
        ST_REQ_THREE2_DT2,     ST_WAIT_THREE2_DT2,

        ST_REQ_DT_CU,          ST_WAIT_DT_CU,

        ST_REQ_HALF_DT3,       ST_WAIT_HALF_DT3,
        ST_REQ_THREE_DT3,      ST_WAIT_THREE_DT3,
        ST_REQ_SIXTH_DT3,      ST_WAIT_SIXTH_DT3,
        ST_REQ_TWO3_DT3,       ST_WAIT_TWO3_DT3,

        ST_REQ_DT_QU,          ST_WAIT_DT_QU,

        ST_REQ_QUARTER_DT4,    ST_WAIT_QUARTER_DT4,
        ST_REQ_SIXTH_DT4,      ST_WAIT_SIXTH_DT4,
        ST_REQ_TWIVE_DT4,      ST_WAIT_TWIVE_DT4,
        ST_REQ_FIVE12_DT4,     ST_WAIT_FIVE12_DT4,

        ST_REQ_DT_QI,          ST_WAIT_DT_QI,

        ST_REQ_SIX_DT5,        ST_WAIT_SIX_DT5,
        ST_REQ_TWLEVE_DT5,     ST_WAIT_TWLEVE_DT5,

        ST_REQ_DT_SX,          ST_WAIT_DT_SX,

        ST_REQ_THIRTYSIX_DT6,  ST_WAIT_THIRTYSIX_DT6
    } state_t;

    state_t st;

    function automatic logic is_wait_state(input state_t s);
        unique case (s)
            ST_WAIT_DT_SQ,
            ST_WAIT_2DT,
            ST_WAIT_HALF_DT2,
            ST_WAIT_THREE2_DT2,
            ST_WAIT_DT_CU,
            ST_WAIT_HALF_DT3,
            ST_WAIT_THREE_DT3,
            ST_WAIT_SIXTH_DT3,
            ST_WAIT_TWO3_DT3,
            ST_WAIT_DT_QU,
            ST_WAIT_QUARTER_DT4,
            ST_WAIT_SIXTH_DT4,
            ST_WAIT_TWIVE_DT4,
            ST_WAIT_FIVE12_DT4,
            ST_WAIT_DT_QI,
            ST_WAIT_SIX_DT5,
            ST_WAIT_TWLEVE_DT5,
            ST_WAIT_DT_SX,
            ST_WAIT_THIRTYSIX_DT6: is_wait_state = 1'b1;
            default:               is_wait_state = 1'b0;
        endcase
    endfunction

    always_comb begin
        mul_req_valid  = 1'b0;
        mul_req_a      = '0;
        mul_req_b      = '0;

        // FIX: only ready in WAIT states
        mul_resp_ready = is_wait_state(st);

        unique case (st)
            ST_REQ_DT_SQ: begin mul_req_valid = 1'b1; mul_req_a = dt_reg; mul_req_b = dt_reg; end
            ST_REQ_2DT:   begin mul_req_valid = 1'b1; mul_req_a = C_2_0;  mul_req_b = dt_reg; end

            ST_REQ_HALF_DT2:   begin mul_req_valid = 1'b1; mul_req_a = dt_sq; mul_req_b = C_0_5; end
            ST_REQ_THREE2_DT2: begin mul_req_valid = 1'b1; mul_req_a = dt_sq; mul_req_b = C_1_5; end

            ST_REQ_DT_CU: begin mul_req_valid = 1'b1; mul_req_a = dt_sq; mul_req_b = dt_reg; end

            ST_REQ_HALF_DT3:  begin mul_req_valid = 1'b1; mul_req_a = dt_cu; mul_req_b = C_0_5; end
            ST_REQ_THREE_DT3: begin mul_req_valid = 1'b1; mul_req_a = dt_cu; mul_req_b = C_0_333; end
            ST_REQ_SIXTH_DT3: begin mul_req_valid = 1'b1; mul_req_a = dt_cu; mul_req_b = C_0_166; end
            ST_REQ_TWO3_DT3:  begin mul_req_valid = 1'b1; mul_req_a = dt_cu; mul_req_b = C_0_666; end

            ST_REQ_DT_QU: begin mul_req_valid = 1'b1; mul_req_a = dt_cu; mul_req_b = dt_reg; end

            ST_REQ_QUARTER_DT4: begin mul_req_valid = 1'b1; mul_req_a = dt_qu; mul_req_b = C_0_25; end
            ST_REQ_SIXTH_DT4:   begin mul_req_valid = 1'b1; mul_req_a = dt_qu; mul_req_b = C_0_166; end
            ST_REQ_TWIVE_DT4:   begin mul_req_valid = 1'b1; mul_req_a = dt_qu; mul_req_b = C_0_0833; end
            ST_REQ_FIVE12_DT4:  begin mul_req_valid = 1'b1; mul_req_a = dt_qu; mul_req_b = C_0_416; end

            ST_REQ_DT_QI: begin mul_req_valid = 1'b1; mul_req_a = dt_qu; mul_req_b = dt_reg; end

            ST_REQ_SIX_DT5:    begin mul_req_valid = 1'b1; mul_req_a = dt_qi; mul_req_b = C_0_166; end
            ST_REQ_TWLEVE_DT5: begin mul_req_valid = 1'b1; mul_req_a = dt_qi; mul_req_b = C_0_0833; end

            ST_REQ_DT_SX: begin mul_req_valid = 1'b1; mul_req_a = dt_qi; mul_req_b = dt_reg; end

            ST_REQ_THIRTYSIX_DT6: begin mul_req_valid = 1'b1; mul_req_a = dt_sx; mul_req_b = C_0_0277; end
            default: ;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st            <= ST_IDLE;
            done          <= 1'b0;
            valid         <= 1'b0;

            dt_reg        <= '0;
            dt_sq         <= '0;
            dt_cu         <= '0;
            dt_qu         <= '0;
            dt_qi         <= '0;
            dt_sx         <= '0;

            delta_t2      <= '0;
            dt2           <= '0;
            half_dt2      <= '0;
            three2_dt2    <= '0;

            half_dt3      <= '0;
            three_dt3     <= '0;
            sixth_dt3     <= '0;
            two3_dt3      <= '0;

            quarter_dt4   <= '0;
            sixth_dt4     <= '0;
            twive_dt4     <= '0;
            five12_dt4    <= '0;

            six_dt5       <= '0;
            twleve_dt5    <= '0;

            thirtysix_dt6 <= '0;
        end else begin
            done <= 1'b0;

            // FIX: only accept start in IDLE
            if (start && (st == ST_IDLE)) begin
                dt_reg <= delta_t;
                st     <= ST_REQ_DT_SQ;
                valid  <= 1'b0;
            end else begin
                unique case (st)
                    ST_IDLE: ;

                    // request -> wait
                    ST_REQ_DT_SQ:      if (mul_req_valid && mul_req_ready) st <= ST_WAIT_DT_SQ;
                    ST_REQ_2DT:        if (mul_req_valid && mul_req_ready) st <= ST_WAIT_2DT;

                    ST_REQ_HALF_DT2:   if (mul_req_valid && mul_req_ready) st <= ST_WAIT_HALF_DT2;
                    ST_REQ_THREE2_DT2: if (mul_req_valid && mul_req_ready) st <= ST_WAIT_THREE2_DT2;

                    ST_REQ_DT_CU:      if (mul_req_valid && mul_req_ready) st <= ST_WAIT_DT_CU;

                    ST_REQ_HALF_DT3:   if (mul_req_valid && mul_req_ready) st <= ST_WAIT_HALF_DT3;
                    ST_REQ_THREE_DT3:  if (mul_req_valid && mul_req_ready) st <= ST_WAIT_THREE_DT3;
                    ST_REQ_SIXTH_DT3:  if (mul_req_valid && mul_req_ready) st <= ST_WAIT_SIXTH_DT3;
                    ST_REQ_TWO3_DT3:   if (mul_req_valid && mul_req_ready) st <= ST_WAIT_TWO3_DT3;

                    ST_REQ_DT_QU:      if (mul_req_valid && mul_req_ready) st <= ST_WAIT_DT_QU;

                    ST_REQ_QUARTER_DT4: if (mul_req_valid && mul_req_ready) st <= ST_WAIT_QUARTER_DT4;
                    ST_REQ_SIXTH_DT4:   if (mul_req_valid && mul_req_ready) st <= ST_WAIT_SIXTH_DT4;
                    ST_REQ_TWIVE_DT4:   if (mul_req_valid && mul_req_ready) st <= ST_WAIT_TWIVE_DT4;
                    ST_REQ_FIVE12_DT4:  if (mul_req_valid && mul_req_ready) st <= ST_WAIT_FIVE12_DT4;

                    ST_REQ_DT_QI:      if (mul_req_valid && mul_req_ready) st <= ST_WAIT_DT_QI;

                    ST_REQ_SIX_DT5:    if (mul_req_valid && mul_req_ready) st <= ST_WAIT_SIX_DT5;
                    ST_REQ_TWLEVE_DT5: if (mul_req_valid && mul_req_ready) st <= ST_WAIT_TWLEVE_DT5;

                    ST_REQ_DT_SX:      if (mul_req_valid && mul_req_ready) st <= ST_WAIT_DT_SX;

                    ST_REQ_THIRTYSIX_DT6: if (mul_req_valid && mul_req_ready) st <= ST_WAIT_THIRTYSIX_DT6;

                    // wait -> latch -> next
                    ST_WAIT_DT_SQ: if (mul_resp_valid && mul_resp_ready) begin
                        dt_sq <= mul_resp_y; dt2 <= mul_resp_y; st <= ST_REQ_2DT;
                    end
                    ST_WAIT_2DT: if (mul_resp_valid && mul_resp_ready) begin
                        delta_t2 <= mul_resp_y; st <= ST_REQ_HALF_DT2;
                    end
                    ST_WAIT_HALF_DT2: if (mul_resp_valid && mul_resp_ready) begin
                        half_dt2 <= mul_resp_y; st <= ST_REQ_THREE2_DT2;
                    end
                    ST_WAIT_THREE2_DT2: if (mul_resp_valid && mul_resp_ready) begin
                        three2_dt2 <= mul_resp_y; st <= ST_REQ_DT_CU;
                    end
                    ST_WAIT_DT_CU: if (mul_resp_valid && mul_resp_ready) begin
                        dt_cu <= mul_resp_y; st <= ST_REQ_HALF_DT3;
                    end
                    ST_WAIT_HALF_DT3: if (mul_resp_valid && mul_resp_ready) begin
                        half_dt3 <= mul_resp_y; st <= ST_REQ_THREE_DT3;
                    end
                    ST_WAIT_THREE_DT3: if (mul_resp_valid && mul_resp_ready) begin
                        three_dt3 <= mul_resp_y; st <= ST_REQ_SIXTH_DT3;
                    end
                    ST_WAIT_SIXTH_DT3: if (mul_resp_valid && mul_resp_ready) begin
                        sixth_dt3 <= mul_resp_y; st <= ST_REQ_TWO3_DT3;
                    end
                    ST_WAIT_TWO3_DT3: if (mul_resp_valid && mul_resp_ready) begin
                        two3_dt3 <= mul_resp_y; st <= ST_REQ_DT_QU;
                    end
                    ST_WAIT_DT_QU: if (mul_resp_valid && mul_resp_ready) begin
                        dt_qu <= mul_resp_y; st <= ST_REQ_QUARTER_DT4;
                    end
                    ST_WAIT_QUARTER_DT4: if (mul_resp_valid && mul_resp_ready) begin
                        quarter_dt4 <= mul_resp_y; st <= ST_REQ_SIXTH_DT4;
                    end
                    ST_WAIT_SIXTH_DT4: if (mul_resp_valid && mul_resp_ready) begin
                        sixth_dt4 <= mul_resp_y; st <= ST_REQ_TWIVE_DT4;
                    end
                    ST_WAIT_TWIVE_DT4: if (mul_resp_valid && mul_resp_ready) begin
                        twive_dt4 <= mul_resp_y; st <= ST_REQ_FIVE12_DT4;
                    end
                    ST_WAIT_FIVE12_DT4: if (mul_resp_valid && mul_resp_ready) begin
                        five12_dt4 <= mul_resp_y; st <= ST_REQ_DT_QI;
                    end
                    ST_WAIT_DT_QI: if (mul_resp_valid && mul_resp_ready) begin
                        dt_qi <= mul_resp_y; st <= ST_REQ_SIX_DT5;
                    end
                    ST_WAIT_SIX_DT5: if (mul_resp_valid && mul_resp_ready) begin
                        six_dt5 <= mul_resp_y; st <= ST_REQ_TWLEVE_DT5;
                    end
                    ST_WAIT_TWLEVE_DT5: if (mul_resp_valid && mul_resp_ready) begin
                        twleve_dt5 <= mul_resp_y; st <= ST_REQ_DT_SX;
                    end
                    ST_WAIT_DT_SX: if (mul_resp_valid && mul_resp_ready) begin
                        dt_sx <= mul_resp_y; st <= ST_REQ_THIRTYSIX_DT6;
                    end
                    ST_WAIT_THIRTYSIX_DT6: if (mul_resp_valid && mul_resp_ready) begin
                        thirtysix_dt6 <= mul_resp_y;
                        valid         <= 1'b1;
                        done          <= 1'b1;
                        st            <= ST_IDLE;
                    end
                    default: st <= ST_IDLE;
                endcase
            end
        end
    end

endmodule


// =========================================================
// Shared MUL arb (Vivado-friendly: avoid declaring variables inside procedural blocks)
// =========================================================
module FpMulArb #(
    parameter int DWIDTH      = 64,
    parameter int NUM_CLIENTS = 1
)(
    input  logic                    clk,
    input  logic                    rst_n,

    input  logic [NUM_CLIENTS-1:0]  req_valid,
    output logic [NUM_CLIENTS-1:0]  req_ready,
    input  logic [DWIDTH-1:0]       req_a [0:NUM_CLIENTS-1],
    input  logic [DWIDTH-1:0]       req_b [0:NUM_CLIENTS-1],

    output logic [NUM_CLIENTS-1:0]  resp_valid,
    input  logic [NUM_CLIENTS-1:0]  resp_ready,
    output logic [DWIDTH-1:0]       resp_y [0:NUM_CLIENTS-1]
);

    localparam int PTR_W = (NUM_CLIENTS <= 1) ? 1 : $clog2(NUM_CLIENTS);

    logic [PTR_W-1:0] rr_ptr;
    logic             busy;
    logic             resp_pending;
    logic [PTR_W-1:0] cur_client;

    logic [DWIDTH-1:0] a_reg, b_reg;
    logic [DWIDTH-1:0] y_reg;

    logic mul_valid_pulse;
    logic mul_ready;
    logic mul_finish;
    logic [DWIDTH-1:0] mul_result;

    logic grant;
    logic [PTR_W-1:0] grant_client;

    // --------------------------
    // combinational grant
    // --------------------------
    always_comb begin : GRANT_COMB
        int c_i;
        int off_i;
        int idx_i;

        grant        = 1'b0;
        grant_client = rr_ptr;

        for (c_i = 0; c_i < NUM_CLIENTS; c_i = c_i + 1)
            req_ready[c_i] = 1'b0;

        if (!busy && !resp_pending && mul_ready) begin
            for (off_i = 0; off_i < NUM_CLIENTS; off_i = off_i + 1) begin
                idx_i = int'(rr_ptr) + off_i;
                if (idx_i >= NUM_CLIENTS) idx_i = idx_i - NUM_CLIENTS;

                if (!grant && req_valid[idx_i]) begin
                    grant           = 1'b1;
                    grant_client    = PTR_W'(idx_i);   // FIX: cast instead of idx_i[...]
                    req_ready[idx_i]= 1'b1;
                end
            end
        end
    end

    // --------------------------
    // response mux
    // --------------------------
    always_comb begin : RESP_COMB
        int c_i;

        for (c_i = 0; c_i < NUM_CLIENTS; c_i = c_i + 1) begin
            resp_valid[c_i] = 1'b0;
            resp_y[c_i]     = '0;
        end

        if (resp_pending) begin
            resp_valid[cur_client] = 1'b1;
            resp_y[cur_client]     = y_reg;
        end
    end

    // multiplier instance
    fp_multiplier u_mul (
        .clk    (clk),
        .rst_n  (rst_n),
        .valid  (mul_valid_pulse),
        .ready  (mul_ready),
        .finish (mul_finish),
        .a      (a_reg),
        .b      (b_reg),
        .result (mul_result)
    );

    // sequential control
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr          <= '0;
            busy            <= 1'b0;
            resp_pending    <= 1'b0;
            cur_client      <= '0;
            a_reg           <= '0;
            b_reg           <= '0;
            y_reg           <= '0;
            mul_valid_pulse <= 1'b0;
        end else begin
            mul_valid_pulse <= 1'b0;

            // accept response
            if (resp_pending && resp_valid[cur_client] && resp_ready[cur_client])
                resp_pending <= 1'b0;

            // start new op
            if (grant && req_valid[grant_client] && req_ready[grant_client]) begin
                a_reg           <= req_a[grant_client];
                b_reg           <= req_b[grant_client];
                cur_client      <= grant_client;
                busy            <= 1'b1;
                mul_valid_pulse <= 1'b1;

                if (NUM_CLIENTS <= 1) rr_ptr <= '0;
                else rr_ptr <= (grant_client == (NUM_CLIENTS-1)) ? '0 : (grant_client + 1'b1);
            end

            // finish
            if (busy && mul_finish) begin
                y_reg         <= mul_result;
                busy          <= 1'b0;
                resp_pending  <= 1'b1;
            end
        end
    end

endmodule



// =========================================================
// KalmanGainCalculator (Vivado 2020.1 synthesizable)
//  - FIX: P_predicted index mapping for phi12/phi13
//  - FIX: Serial MAC rewritten as ISSUE/WAIT to avoid stale operand sampling
// =========================================================
module KalmanGainCalculator #(
    parameter int DWIDTH      = 64,
    parameter int STATE_DIM   = 12,
    parameter int MEASURE_DIM = 6
)(
    input  logic                        clk,
    input  logic                        rst_n,
    input  logic [DWIDTH-1:0]           delta_t,

    input  logic                        SP_Done,
    output logic                        CKG_Done,

    input  logic [DWIDTH-1:0]           P_k1k1 [0:STATE_DIM-1][0:STATE_DIM-1],
    input  logic [DWIDTH-1:0]           Q_k    [0:STATE_DIM-1][0:STATE_DIM-1],
    input  logic [DWIDTH-1:0]           R_k    [0:MEASURE_DIM-1][0:MEASURE_DIM-1],

    output logic [DWIDTH-1:0]           K_k    [0:STATE_DIM-1][0:MEASURE_DIM-1]
);

    localparam int N = STATE_DIM;
    localparam int M = MEASURE_DIM;

    // --------------------------
    // Predicted covariance
    // --------------------------
    logic [DWIDTH-1:0] P_predicted    [0:N-1][0:N-1];
    logic [DWIDTH-1:0] P_predicted_HT [0:N-1][0:M-1];

    integer i0, j0;
    always_comb begin
        for (i0 = 0; i0 < N; i0 = i0 + 1) begin
            for (j0 = 0; j0 < M; j0 = j0 + 1) begin
                P_predicted_HT[i0][j0] = P_predicted[i0][j0];
            end
        end
    end

    // --------------------------
    // tp_start = rising edge of SP_Done
    // --------------------------
    logic sp_done_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) sp_done_d <= 1'b0;
        else        sp_done_d <= SP_Done;
    end
    wire tp_start = SP_Done & ~sp_done_d;

    // --------------------------
    // Time params (shared MUL)
    // --------------------------
    logic [DWIDTH-1:0] delta_t2;
    logic [DWIDTH-1:0] dt2;
    logic [DWIDTH-1:0] half_dt2, three2_dt2;
    logic [DWIDTH-1:0] half_dt3, three_dt3, sixth_dt3, two3_dt3;
    logic [DWIDTH-1:0] quarter_dt4, sixth_dt4, twive_dt4, five12_dt4;
    logic [DWIDTH-1:0] six_dt5, twleve_dt5;
    logic [DWIDTH-1:0] thirtysix_dt6;
    logic tp_done, tp_valid;

    // TimeParamsSeq <-> shared mul signals
    logic                  tp_mul_req_valid;
    logic                  tp_mul_req_ready;
    logic [DWIDTH-1:0]     tp_mul_req_a;
    logic [DWIDTH-1:0]     tp_mul_req_b;
    logic                  tp_mul_resp_valid;
    logic                  tp_mul_resp_ready;
    logic [DWIDTH-1:0]     tp_mul_resp_y;

    TimeParamsSeq #(.DWIDTH(DWIDTH)) u_timeparams (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (tp_start),
        .delta_t        (delta_t),

        .mul_req_valid  (tp_mul_req_valid),
        .mul_req_ready  (tp_mul_req_ready),
        .mul_req_a      (tp_mul_req_a),
        .mul_req_b      (tp_mul_req_b),

        .mul_resp_valid (tp_mul_resp_valid),
        .mul_resp_ready (tp_mul_resp_ready),
        .mul_resp_y     (tp_mul_resp_y),

        .delta_t2       (delta_t2),
        .dt2            (dt2),
        .half_dt2       (half_dt2),
        .three2_dt2     (three2_dt2),
        .half_dt3       (half_dt3),
        .three_dt3      (three_dt3),
        .sixth_dt3      (sixth_dt3),
        .two3_dt3       (two3_dt3),
        .quarter_dt4    (quarter_dt4),
        .sixth_dt4      (sixth_dt4),
        .twive_dt4      (twive_dt4),
        .five12_dt4     (five12_dt4),
        .six_dt5        (six_dt5),
        .twleve_dt5     (twleve_dt5),
        .thirtysix_dt6  (thirtysix_dt6),
        .done           (tp_done),
        .valid          (tp_valid)
    );

    // shared mul arb (1 client now)
    localparam int MUL_CLIENTS = 1;
    logic [MUL_CLIENTS-1:0] mul_req_valid_bus;
    logic [MUL_CLIENTS-1:0] mul_req_ready_bus;
    logic [DWIDTH-1:0]      mul_req_a_bus [0:MUL_CLIENTS-1];
    logic [DWIDTH-1:0]      mul_req_b_bus [0:MUL_CLIENTS-1];
    logic [MUL_CLIENTS-1:0] mul_resp_valid_bus;
    logic [MUL_CLIENTS-1:0] mul_resp_ready_bus;
    logic [DWIDTH-1:0]      mul_resp_y_bus [0:MUL_CLIENTS-1];

    assign mul_req_valid_bus[0]  = tp_mul_req_valid;
    assign mul_req_a_bus[0]      = tp_mul_req_a;
    assign mul_req_b_bus[0]      = tp_mul_req_b;
    assign tp_mul_req_ready      = mul_req_ready_bus[0];

    assign tp_mul_resp_valid     = mul_resp_valid_bus[0];
    assign tp_mul_resp_y         = mul_resp_y_bus[0];
    assign mul_resp_ready_bus[0] = tp_mul_resp_ready;

    FpMulArb #(.DWIDTH(DWIDTH), .NUM_CLIENTS(MUL_CLIENTS)) u_shared_mul (
        .clk        (clk),
        .rst_n      (rst_n),
        .req_valid  (mul_req_valid_bus),
        .req_ready  (mul_req_ready_bus),
        .req_a      (mul_req_a_bus),
        .req_b      (mul_req_b_bus),
        .resp_valid (mul_resp_valid_bus),
        .resp_ready (mul_resp_ready_bus),
        .resp_y     (mul_resp_y_bus)
    );

    // --------------------------
    // Matrix inverse
    // --------------------------
    logic [DWIDTH-1:0] inv_matrix [0:5][0:5];
    logic              inv_complete;

    MatrixInverseUnit #(.DWIDTH(DWIDTH)) u_MatrixInverseUnit (
        .clk          (clk),
        .rst_n        (rst_n),
        .valid        (SP_Done),
        .tp_valid     (tp_valid),

        .delta_t       (delta_t),
        .delta_t2      (delta_t2),
        .dt2           (dt2),
        .half_dt2      (half_dt2),
        .half_dt3      (half_dt3),
        .three_dt3     (three_dt3),
        .sixth_dt3     (sixth_dt3),
        .quarter_dt4   (quarter_dt4),
        .twive_dt4     (twive_dt4),
        .five12_dt4    (five12_dt4),
        .six_dt5       (six_dt5),
        .twleve_dt5    (twleve_dt5),
        .thirtysix_dt6 (thirtysix_dt6),

        .P_k1k1       (P_k1k1),
        .Q_k          (Q_k),
        .R_k          (R_k),

        .inv_matrix   (inv_matrix),
        .finish       (inv_complete)
    );

    // --------------------------
    // CMU time-mux (3 rounds for x/y/z)
    // NOTE: keep your CMU modules as-is; we only fix capture indices + MAC
    // --------------------------
    localparam int CMU_ITER = 3;
    logic [1:0] cmu_idx;
    logic       cmu_all_done;

    // Use registered tp_valid to make rst_cmu deassert synchronous (safer for reset tree)
    logic tp_valid_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) tp_valid_q <= 1'b0;
        else        tp_valid_q <= tp_valid;
    end
    wire rst_cmu = rst_n & tp_valid_q;

    // convenient aliases (state layout: [p,v,a,j] interleaved for x/y/z)
    logic [DWIDTH-1:0] t_1_1, t_4_1, t_7_1, t_4_4, t_10_1, t_4_7, t_7_7, t_4_10, t_7_10, t_10_10;
    logic [DWIDTH-1:0] t_1_4, t_1_7, t_1_10, t_10_4, t_10_7, t_7_4;

    logic [DWIDTH-1:0] q_1_1, q_1_4, q_1_7, q_1_10, q_4_1, q_4_4, q_4_7, q_4_10;
    logic [DWIDTH-1:0] q_7_1, q_7_4, q_7_7, q_7_10, q_10_1, q_10_4, q_10_7, q_10_10;

    // continuous assigns
    assign t_1_1   = P_k1k1[0 + cmu_idx][0 + cmu_idx];
    assign t_4_1   = P_k1k1[3 + cmu_idx][0 + cmu_idx];
    assign t_7_1   = P_k1k1[6 + cmu_idx][0 + cmu_idx];
    assign t_4_4   = P_k1k1[3 + cmu_idx][3 + cmu_idx];
    assign t_10_1  = P_k1k1[9 + cmu_idx][0 + cmu_idx];
    assign t_4_7   = P_k1k1[3 + cmu_idx][6 + cmu_idx];
    assign t_7_7   = P_k1k1[6 + cmu_idx][6 + cmu_idx];
    assign t_4_10  = P_k1k1[3 + cmu_idx][9 + cmu_idx];
    assign t_7_10  = P_k1k1[6 + cmu_idx][9 + cmu_idx];
    assign t_10_10 = P_k1k1[9 + cmu_idx][9 + cmu_idx];

    assign t_1_4   = P_k1k1[0 + cmu_idx][3 + cmu_idx];
    assign t_1_7   = P_k1k1[0 + cmu_idx][6 + cmu_idx];
    assign t_1_10  = P_k1k1[0 + cmu_idx][9 + cmu_idx];
    assign t_10_4  = P_k1k1[9 + cmu_idx][3 + cmu_idx];
    assign t_10_7  = P_k1k1[9 + cmu_idx][6 + cmu_idx];
    assign t_7_4   = P_k1k1[6 + cmu_idx][3 + cmu_idx];

    assign q_1_1   = Q_k[0 + cmu_idx][0 + cmu_idx];
    assign q_1_4   = Q_k[0 + cmu_idx][3 + cmu_idx];
    assign q_1_7   = Q_k[0 + cmu_idx][6 + cmu_idx];
    assign q_1_10  = Q_k[0 + cmu_idx][9 + cmu_idx];
    assign q_4_1   = Q_k[3 + cmu_idx][0 + cmu_idx];
    assign q_4_4   = Q_k[3 + cmu_idx][3 + cmu_idx];
    assign q_4_7   = Q_k[3 + cmu_idx][6 + cmu_idx];
    assign q_4_10  = Q_k[3 + cmu_idx][9 + cmu_idx];
    assign q_7_1   = Q_k[6 + cmu_idx][0 + cmu_idx];
    assign q_7_4   = Q_k[6 + cmu_idx][3 + cmu_idx];
    assign q_7_7   = Q_k[6 + cmu_idx][6 + cmu_idx];
    assign q_7_10  = Q_k[6 + cmu_idx][9 + cmu_idx];
    assign q_10_1  = Q_k[9 + cmu_idx][0 + cmu_idx];
    assign q_10_4  = Q_k[9 + cmu_idx][3 + cmu_idx];
    assign q_10_7  = Q_k[9 + cmu_idx][6 + cmu_idx];
    assign q_10_10 = Q_k[9 + cmu_idx][9 + cmu_idx];

    // ---- CMU instances (as you had) ----
    logic [DWIDTH-1:0] phi11_a, phi12_a, phi13_a, phi14_a;
    logic [DWIDTH-1:0] phi21_a, phi22_a, phi23_a, phi24_a;
    logic [DWIDTH-1:0] phi31_a, phi32_a, phi33_a, phi34_a;
    logic [DWIDTH-1:0] phi41_a, phi42_a, phi43_a, phi44_a;

    logic v_phi11, v_phi12, v_phi13, v_phi14;
    logic v_phi21, v_phi22, v_phi23, v_phi24;
    logic v_phi31, v_phi32, v_phi33, v_phi34;
    logic v_phi41, v_phi42, v_phi43, v_phi44;

    CMU_PHi11 #(.DBL_WIDTH(64)) u_CMU_PHi11 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_1   (t_1_1),
        .Theta_4_1   (t_4_1),
        .Theta_7_1   (t_7_1),
        .Theta_4_4   (t_4_4),
        .Theta_10_1  (t_10_1),
        .Theta_4_7   (t_4_7),
        .Theta_7_7   (t_7_7),
        .Theta_4_10  (t_4_10),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_1       (q_1_1),
        .delta_t1    (delta_t2),
        .delta_t2    (dt2),
        .delta_t3    (three_dt3),
        .delta_t4    (twive_dt4),
        .delta_t5    (six_dt5),
        .delta_t6    (thirtysix_dt6),
        .a           (phi11_a),
        .valid_out   (v_phi11)
    );

    CMU_PHi12 #(.DBL_WIDTH(64)) u_CMU_PHi12 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_4   (t_1_4),
        .Theta_1_1   (t_1_1),
        .Theta_1_7   (t_1_7),
        .Theta_4_7   (t_4_7),
        .Theta_1_10  (t_1_10),
        .Theta_4_10  (t_4_10),
        .Theta_7_7   (t_7_7),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_4       (q_1_4),
        .delta_t     (delta_t),
        .half_dt2    (half_dt2),
        .sixth_dt3   (sixth_dt3),
        .five12_dt4  (five12_dt4),
        .twleve_dt5  (twleve_dt5),
        .a           (phi12_a),
        .valid_out   (v_phi12)
    );

    CMU_PHi13 #(.DBL_WIDTH(64)) u_CMU_PHi13 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_7   (t_1_7),
        .Theta_4_7   (t_4_7),
        .Theta_1_10  (t_1_10),
        .Theta_7_7   (t_7_7),
        .Theta_4_10  (t_4_10),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_7       (q_1_7),
        .delta_t     (delta_t),
        .half_dt2    (half_dt2),
        .two3_dt3    (two3_dt3),
        .sixth_dt4   (sixth_dt4),
        .a           (phi13_a),
        .valid_out   (v_phi13)
    );

    CMU_PHi14 #(.DBL_WIDTH(64)) u_CMU_PHi14 (
        .clk         (clk),
        .rst_n       (rst_cmu),
        .Theta_1_10  (t_1_10),
        .Theta_4_10  (t_4_10),
        .Theta_7_10  (t_7_10),
        .Theta_10_10 (t_10_10),
        .Q_1_10      (q_1_10),
        .delta_t     (delta_t),
        .half_dt2    (half_dt2),
        .two3_dt3    (two3_dt3),
        .a           (phi14_a),
        .valid_out   (v_phi14)
    );

    CMU_PHi21 #(.DBL_WIDTH(64)) u_CMU_PHi21 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_1    (t_4_1),
        .Theta_7_1    (t_7_1),
        .Theta_4_4    (t_4_4),
        .Theta_1_10   (t_1_10),
        .Theta_4_7    (t_4_7),
        .Theta_7_7    (t_7_7),
        .Theta_4_10   (t_4_10),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_4_1        (q_4_1),
        .delta_t        (delta_t),
        .dt2_half       (half_dt2),
        .dt3_sixth      (sixth_dt3),
        .dt4_twelth     (twive_dt4),
        .dt5_twelth     (twleve_dt5),
        .dt6_thirtysix  (thirtysix_dt6),
        .a            (phi21_a),
        .valid_out    (v_phi21)
    );

    CMU_PHi22 #(.DBL_WIDTH(64)) u_CMU_PHi22 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_4    (t_4_4),
        .Theta_4_7    (t_4_7),
        .Theta_4_10   (t_4_10),
        .Theta_7_7    (t_7_7),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_4_4        (q_4_4),
        .two_dt       (delta_t2),
        .dt2          (dt2),
        .half_dt3     (half_dt3),
        .quarter_dt4  (quarter_dt4),
        .a            (phi22_a),
        .valid_out    (v_phi22)
    );

    CMU_PHi23 #(.DBL_WIDTH(64)) u_CMU_PHi23 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_7    (t_4_7),
        .Theta_7_7    (t_7_7),
        .Theta_4_10   (t_4_10),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_4_7        (q_4_7),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .half_dt3     (half_dt3),
        .a            (phi23_a),
        .valid_out    (v_phi23)
    );

    CMU_PHi24 #(.DBL_WIDTH(64)) u_CMU_PHi24 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_4_10   (t_4_10),
        .Theta_7_4    (t_7_4),
        .Theta_10_10  (t_10_10),
        .Q_4_10       (q_4_10),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .a            (phi24_a),
        .valid_out    (v_phi24)
    );

    CMU_PHi31 #(.DBL_WIDTH(64)) u_CMU_PHi31 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_1    (t_7_1),
        .Theta_1_10   (t_1_10),
        .Theta_4_7    (t_4_7),
        .Theta_7_7    (t_7_7),
        .Theta_4_10   (t_4_10),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_1        (q_7_1),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .two3_dt3     (two3_dt3),
        .sixth_dt4    (sixth_dt4),
        .a            (phi31_a),
        .valid_out    (v_phi31)
    );

    CMU_PHi32 #(.DBL_WIDTH(64)) u_CMU_PHi32 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_4    (t_7_4),
        .Theta_4_10   (t_4_10),
        .Theta_7_7    (t_7_7),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_4        (q_7_4),
        .delta_t      (delta_t),
        .three2_dt2   (three2_dt2),
        .half_dt3     (half_dt3),
        .a            (phi32_a),
        .valid_out    (v_phi32)
    );

    CMU_PHi33 #(.DBL_WIDTH(64)) u_CMU_PHi33 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_7    (t_7_7),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_7        (q_7_7),
        .two_dt       (delta_t2),
        .dt2          (dt2),
        .a            (phi33_a),
        .valid_out    (v_phi33)
    );

    CMU_PHi34 #(.DBL_WIDTH(64)) u_CMU_PHi34 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_7_10   (t_7_10),
        .Theta_10_10  (t_10_10),
        .Q_7_10       (q_7_10),
        .delta_t      (delta_t),
        .a            (phi34_a),
        .valid_out    (v_phi34)
    );

    CMU_PHi41 #(.DBL_WIDTH(64)) u_CMU_PHi41 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_1   (t_10_1),
        .Theta_10_4   (t_10_4),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_10_1       (q_10_1),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .sixth_dt3    (sixth_dt3),
        .a            (phi41_a),
        .valid_out    (v_phi41)
    );

    CMU_PHi42 #(.DBL_WIDTH(64)) u_CMU_PHi42 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_4   (t_10_4),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_10_4       (q_10_4),
        .delta_t      (delta_t),
        .half_dt2     (half_dt2),
        .a            (phi42_a),
        .valid_out    (v_phi42)
    );

    CMU_PHi43 #(.DBL_WIDTH(64)) u_CMU_PHi43 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_7   (t_10_7),
        .Theta_10_10  (t_10_10),
        .Q_10_7       (q_10_7),
        .delta_t      (delta_t),
        .a            (phi43_a),
        .valid_out    (v_phi43)
    );

    CMU_PHi44 #(.DBL_WIDTH(64)) u_CMU_PHi44 (
        .clk          (clk),
        .rst_n        (rst_cmu),
        .Theta_10_10  (t_10_10),
        .Q_10_10      (q_10_10),
        .a            (phi44_a),
        .valid_out    (v_phi44)
    );

    // ---- collect results & cmu_idx FSM ----
    logic d_phi11, d_phi12, d_phi13, d_phi14;
    logic d_phi21, d_phi22, d_phi23, d_phi24;
    logic d_phi31, d_phi32, d_phi33, d_phi34;
    logic d_phi41, d_phi42, d_phi43, d_phi44;

    wire cmu_round_done = d_phi11 & d_phi12 & d_phi13 & d_phi14 &
                          d_phi21 & d_phi22 & d_phi23 & d_phi24 &
                          d_phi31 & d_phi32 & d_phi33 & d_phi34 &
                          d_phi41 & d_phi42 & d_phi43 & d_phi44;

    integer r, c;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (r = 0; r < N; r = r + 1) begin
                for (c = 0; c < N; c = c + 1) begin
                    P_predicted[r][c] <= '0;
                end
            end
            cmu_idx      <= '0;
            cmu_all_done <= 1'b0;
            {d_phi11, d_phi12, d_phi13, d_phi14,
             d_phi21, d_phi22, d_phi23, d_phi24,
             d_phi31, d_phi32, d_phi33, d_phi34,
             d_phi41, d_phi42, d_phi43, d_phi44} <= '0;
        end else begin
            if (tp_start) begin
                cmu_idx      <= '0;
                cmu_all_done <= 1'b0;
                {d_phi11, d_phi12, d_phi13, d_phi14,
                 d_phi21, d_phi22, d_phi23, d_phi24,
                 d_phi31, d_phi32, d_phi33, d_phi34,
                 d_phi41, d_phi42, d_phi43, d_phi44} <= '0;
            end

            if (rst_cmu && !cmu_all_done) begin
                if (!d_phi11 && v_phi11) begin
                    P_predicted[0 + cmu_idx][0 + cmu_idx] <= phi11_a;
                    d_phi11 <= 1'b1;
                end

                // FIX: phi12 should map to column 3+idx (position-velocity)
                if (!d_phi12 && v_phi12) begin
                    P_predicted[0 + cmu_idx][3 + cmu_idx] <= phi12_a;
                    d_phi12 <= 1'b1;
                end

                // FIX: phi13 should map to column 6+idx (position-acc)
                if (!d_phi13 && v_phi13) begin
                    P_predicted[0 + cmu_idx][6 + cmu_idx] <= phi13_a;
                    d_phi13 <= 1'b1;
                end

                if (!d_phi14 && v_phi14) begin
                    P_predicted[0 + cmu_idx][9 + cmu_idx] <= phi14_a;
                    d_phi14 <= 1'b1;
                end

                if (!d_phi21 && v_phi21) begin
                    P_predicted[3 + cmu_idx][0 + cmu_idx] <= phi21_a;
                    d_phi21 <= 1'b1;
                end
                if (!d_phi22 && v_phi22) begin
                    P_predicted[3 + cmu_idx][3 + cmu_idx] <= phi22_a;
                    d_phi22 <= 1'b1;
                end
                if (!d_phi23 && v_phi23) begin
                    P_predicted[3 + cmu_idx][6 + cmu_idx] <= phi23_a;
                    d_phi23 <= 1'b1;
                end
                if (!d_phi24 && v_phi24) begin
                    P_predicted[3 + cmu_idx][9 + cmu_idx] <= phi24_a;
                    d_phi24 <= 1'b1;
                end

                if (!d_phi31 && v_phi31) begin
                    P_predicted[6 + cmu_idx][0 + cmu_idx] <= phi31_a;
                    d_phi31 <= 1'b1;
                end
                if (!d_phi32 && v_phi32) begin
                    P_predicted[6 + cmu_idx][3 + cmu_idx] <= phi32_a;
                    d_phi32 <= 1'b1;
                end
                if (!d_phi33 && v_phi33) begin
                    P_predicted[6 + cmu_idx][6 + cmu_idx] <= phi33_a;
                    d_phi33 <= 1'b1;
                end
                if (!d_phi34 && v_phi34) begin
                    P_predicted[6 + cmu_idx][9 + cmu_idx] <= phi34_a;
                    d_phi34 <= 1'b1;
                end

                if (!d_phi41 && v_phi41) begin
                    P_predicted[9 + cmu_idx][0 + cmu_idx] <= phi41_a;
                    d_phi41 <= 1'b1;
                end
                if (!d_phi42 && v_phi42) begin
                    P_predicted[9 + cmu_idx][3 + cmu_idx] <= phi42_a;
                    d_phi42 <= 1'b1;
                end
                if (!d_phi43 && v_phi43) begin
                    P_predicted[9 + cmu_idx][6 + cmu_idx] <= phi43_a;
                    d_phi43 <= 1'b1;
                end
                if (!d_phi44 && v_phi44) begin
                    P_predicted[9 + cmu_idx][9 + cmu_idx] <= phi44_a;
                    d_phi44 <= 1'b1;
                end

                if (cmu_round_done) begin
                    if (cmu_idx == CMU_ITER-1) begin
                        cmu_all_done <= 1'b1;
                    end else begin
                        cmu_idx <= cmu_idx + 1'b1;
                    end
                    {d_phi11, d_phi12, d_phi13, d_phi14,
                     d_phi21, d_phi22, d_phi23, d_phi24,
                     d_phi31, d_phi32, d_phi33, d_phi34,
                     d_phi41, d_phi42, d_phi43, d_phi44} <= '0;
                end
            end
        end
    end

    // --------------------------
    // Expand matrices to 12x12
    // --------------------------
    logic [DWIDTH-1:0] inv_matrix12 [0:11][0:11];
    genvar gr, gc;
    generate
        for (gr = 0; gr < 12; gr = gr + 1) begin : GEN_INV12_R
            for (gc = 0; gc < 12; gc = gc + 1) begin : GEN_INV12_C
                assign inv_matrix12[gr][gc] = (gr < 6 && gc < 6) ? inv_matrix[gr][gc] : {DWIDTH{1'b0}};
            end
        end
    endgenerate

    logic [DWIDTH-1:0] P_predicted_HT12 [0:11][0:11];
    generate
        for (gr = 0; gr < 12; gr = gr + 1) begin : GEN_PHT12_R
            for (gc = 0; gc < 12; gc = gc + 1) begin : GEN_PHT12_C
                assign P_predicted_HT12[gr][gc] = (gc < 6) ? P_predicted_HT[gr][gc] : {DWIDTH{1'b0}};
            end
        end
    endgenerate

    // --------------------------
    // Start MAC after inv_complete & tp_valid & cmu_all_done (one-shot)
    // --------------------------
    logic inv_lat;
    logic mac_fired;

    wire mac_start_pulse = inv_lat & tp_valid & cmu_all_done & ~mac_fired;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inv_lat   <= 1'b0;
            mac_fired <= 1'b0;
        end else begin
            if (tp_start) begin
                inv_lat   <= 1'b0;
                mac_fired <= 1'b0;
            end
            if (inv_complete) inv_lat <= 1'b1;
            if (mac_start_pulse) mac_fired <= 1'b1;
        end
    end

    // --------------------------
    // Serial MAC: K = P_predicted_HT12(12x6) * inv_matrix12(6x6)
    // FIX: ISSUE/WAIT FSM (avoid stale operand sampling)
    // --------------------------
    logic [DWIDTH-1:0] K_k_matrix [0:11][0:11];
    logic              K_done;

    typedef enum logic [2:0] {
        K_IDLE,
        K_ISSUE_MUL,
        K_WAIT_MUL,
        K_ISSUE_ADD,
        K_WAIT_ADD,
        K_STORE
    } k_state_e;

    k_state_e k_state;

    logic [3:0] row_idx;
    logic [2:0] col_idx;
    logic [2:0] k_idx;

    logic [DWIDTH-1:0] acc_reg;
    logic [DWIDTH-1:0] mul_res_reg;

    logic mul_go, add_go;
    logic mul_ready_k, add_ready_k;
    logic mul_finish_k, add_finish_k;
    logic [DWIDTH-1:0] mul_result_k, add_result_k;

    // operands (combinational)
    logic [DWIDTH-1:0] mul_a_k, mul_b_k;
    logic [DWIDTH-1:0] add_a_k, add_b_k;

    assign mul_a_k = P_predicted_HT12[row_idx][k_idx];
    assign mul_b_k = inv_matrix12[k_idx][col_idx];

    assign add_a_k = acc_reg;
    assign add_b_k = mul_res_reg;

    fp_multiplier u_kmul (
        .clk    (clk),
        .rst_n  (rst_n),
        .valid  (mul_go),
        .ready  (mul_ready_k),
        .finish (mul_finish_k),
        .a      (mul_a_k),
        .b      (mul_b_k),
        .result (mul_result_k)
    );

    fp_adder u_kadd (
        .clk    (clk),
        .rst_n  (rst_n),
        .valid  (add_go),
        .ready  (add_ready_k),
        .finish (add_finish_k),
        .a      (add_a_k),
        .b      (add_b_k),
        .result (add_result_k)
    );

    integer rr, cc2;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (rr = 0; rr < 12; rr = rr + 1) begin
                for (cc2 = 0; cc2 < 12; cc2 = cc2 + 1) begin
                    K_k_matrix[rr][cc2] <= '0;
                end
            end
            k_state     <= K_IDLE;
            row_idx     <= '0;
            col_idx     <= '0;
            k_idx       <= '0;
            acc_reg     <= '0;
            mul_res_reg <= '0;
            mul_go      <= 1'b0;
            add_go      <= 1'b0;
            K_done      <= 1'b0;
        end else begin
            mul_go <= 1'b0;
            add_go <= 1'b0;
            K_done <= 1'b0;

            if (tp_start) begin
                for (rr = 0; rr < 12; rr = rr + 1) begin
                    for (cc2 = 0; cc2 < 12; cc2 = cc2 + 1) begin
                        K_k_matrix[rr][cc2] <= '0;
                    end
                end
                k_state     <= K_IDLE;
                row_idx     <= '0;
                col_idx     <= '0;
                k_idx       <= '0;
                acc_reg     <= '0;
                mul_res_reg <= '0;
            end

            unique case (k_state)
                K_IDLE: begin
                    if (mac_start_pulse) begin
                        row_idx <= 4'd0;
                        col_idx <= 3'd0;
                        k_idx   <= 3'd0;
                        acc_reg <= '0;
                        k_state <= K_ISSUE_MUL;
                    end
                end

                K_ISSUE_MUL: begin
                    if (mul_ready_k) begin
                        mul_go  <= 1'b1;
                        k_state <= K_WAIT_MUL;
                    end
                end

                K_WAIT_MUL: begin
                    if (mul_finish_k) begin
                        mul_res_reg <= mul_result_k;
                        k_state     <= K_ISSUE_ADD;
                    end
                end

                K_ISSUE_ADD: begin
                    if (add_ready_k) begin
                        add_go  <= 1'b1;
                        k_state <= K_WAIT_ADD;
                    end
                end

                K_WAIT_ADD: begin
                    if (add_finish_k) begin
                        acc_reg <= add_result_k;
                        if (k_idx == 3'd5) begin
                            k_state <= K_STORE;
                        end else begin
                            k_idx   <= k_idx + 1'b1;
                            k_state <= K_ISSUE_MUL;
                        end
                    end
                end

                K_STORE: begin
                    K_k_matrix[row_idx][col_idx] <= acc_reg;

                    acc_reg <= '0;
                    k_idx   <= 3'd0;

                    if (col_idx == 3'd5) begin
                        if (row_idx == 4'd11) begin
                            K_done  <= 1'b1;
                            k_state <= K_IDLE;
                            col_idx <= 3'd0;
                            row_idx <= 4'd0;
                        end else begin
                            row_idx <= row_idx + 1'b1;
                            col_idx <= 3'd0;
                            k_state <= K_ISSUE_MUL;
                        end
                    end else begin
                        col_idx <= col_idx + 1'b1;
                        k_state <= K_ISSUE_MUL;
                    end
                end

                default: k_state <= K_IDLE;
            endcase
        end
    end

    // CKG_Done level (clear on tp_start, set on K_done)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            CKG_Done <= 1'b0;
        end else begin
            if (tp_start)      CKG_Done <= 1'b0;
            else if (K_done)   CKG_Done <= 1'b1;
        end
    end

    // output K_k = K_k_matrix[0:11][0:5]
    generate
        for (gr = 0; gr < 12; gr = gr + 1) begin : GEN_K_OUT_R
            for (gc = 0; gc < 6; gc = gc + 1) begin : GEN_K_OUT_C
                assign K_k[gr][gc] = K_k_matrix[gr][gc];
            end
        end
    endgenerate

endmodule
