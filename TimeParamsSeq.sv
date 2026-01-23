`timescale 1ns/1ps
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

    // helper: is WAIT state?
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

    // combinational handshake
    always_comb begin
        mul_req_valid  = 1'b0;
        mul_req_a      = '0;
        mul_req_b      = '0;

        // FIX: only ready in WAIT states
        mul_resp_ready = is_wait_state(st);

        unique case (st)
            ST_REQ_DT_SQ: begin
                mul_req_valid = 1'b1; mul_req_a = dt_reg; mul_req_b = dt_reg;
            end
            ST_REQ_2DT: begin
                mul_req_valid = 1'b1; mul_req_a = C_2_0;  mul_req_b = dt_reg;
            end
            ST_REQ_HALF_DT2: begin
                mul_req_valid = 1'b1; mul_req_a = dt_sq;  mul_req_b = C_0_5;
            end
            ST_REQ_THREE2_DT2: begin
                mul_req_valid = 1'b1; mul_req_a = dt_sq;  mul_req_b = C_1_5;
            end
            ST_REQ_DT_CU: begin
                mul_req_valid = 1'b1; mul_req_a = dt_sq;  mul_req_b = dt_reg;
            end
            ST_REQ_HALF_DT3: begin
                mul_req_valid = 1'b1; mul_req_a = dt_cu;  mul_req_b = C_0_5;
            end
            ST_REQ_THREE_DT3: begin
                mul_req_valid = 1'b1; mul_req_a = dt_cu;  mul_req_b = C_0_333;
            end
            ST_REQ_SIXTH_DT3: begin
                mul_req_valid = 1'b1; mul_req_a = dt_cu;  mul_req_b = C_0_166;
            end
            ST_REQ_TWO3_DT3: begin
                mul_req_valid = 1'b1; mul_req_a = dt_cu;  mul_req_b = C_0_666;
            end
            ST_REQ_DT_QU: begin
                mul_req_valid = 1'b1; mul_req_a = dt_cu;  mul_req_b = dt_reg;
            end
            ST_REQ_QUARTER_DT4: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qu;  mul_req_b = C_0_25;
            end
            ST_REQ_SIXTH_DT4: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qu;  mul_req_b = C_0_166;
            end
            ST_REQ_TWIVE_DT4: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qu;  mul_req_b = C_0_0833;
            end
            ST_REQ_FIVE12_DT4: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qu;  mul_req_b = C_0_416;
            end
            ST_REQ_DT_QI: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qu;  mul_req_b = dt_reg;
            end
            ST_REQ_SIX_DT5: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qi;  mul_req_b = C_0_166;
            end
            ST_REQ_TWLEVE_DT5: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qi;  mul_req_b = C_0_0833;
            end
            ST_REQ_DT_SX: begin
                mul_req_valid = 1'b1; mul_req_a = dt_qi;  mul_req_b = dt_reg;
            end
            ST_REQ_THIRTYSIX_DT6: begin
                mul_req_valid = 1'b1; mul_req_a = dt_sx;  mul_req_b = C_0_0277;
            end
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

            // FIX: only accept start in IDLE (avoid level restart)
            if (start && (st == ST_IDLE)) begin
                dt_reg <= delta_t;
                st     <= ST_REQ_DT_SQ;
                valid  <= 1'b0;
            end else begin
                unique case (st)
                    ST_IDLE: begin end

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

                    // wait -> latch
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
