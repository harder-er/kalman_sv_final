`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/17 13:25:37
// Design Name: 
// Module Name: TimeParamsSeq
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module TimeParamsSeq #(
    parameter int DWIDTH = 64
)(
    input  logic                clk,
    input  logic                rst_n,
    input  logic                start,     // 1-cycle pulse
    input  logic [DWIDTH-1:0]   delta_t,

    output logic [DWIDTH-1:0]   delta_t2,        // 2*dt
    output logic [DWIDTH-1:0]   dt2,             // dt^2

    output logic [DWIDTH-1:0]   half_dt2,        // 1/2*dt^2
    output logic [DWIDTH-1:0]   three2_dt2,      // 3/2*dt^2

    output logic [DWIDTH-1:0]   half_dt3,        // 1/2*dt^3
    output logic [DWIDTH-1:0]   three_dt3,       // 1/3*dt^3
    output logic [DWIDTH-1:0]   sixth_dt3,       // 1/6*dt^3
    output logic [DWIDTH-1:0]   two3_dt3,        // 2/3*dt^3

    output logic [DWIDTH-1:0]   quarter_dt4,     // 1/4*dt^4
    output logic [DWIDTH-1:0]   sixth_dt4,       // 1/6*dt^4
    output logic [DWIDTH-1:0]   twive_dt4,       // 1/12*dt^4
    output logic [DWIDTH-1:0]   five12_dt4,      // 5/12*dt^4

    output logic [DWIDTH-1:0]   six_dt5,         // 1/6*dt^5
    output logic [DWIDTH-1:0]   twleve_dt5,      // 1/12*dt^5

    output logic [DWIDTH-1:0]   thirtysix_dt6,   // 1/36*dt^6

    output logic                done,            // 1-cycle pulse when finished
    output logic                valid            // stays 1 until next start/reset
);

    // IEEE754 double constants
    localparam logic [DWIDTH-1:0] C_2_0    = 64'h4000_0000_0000_0000; // 2.0
    localparam logic [DWIDTH-1:0] C_0_5    = 64'h3FE0_0000_0000_0000; // 0.5
    localparam logic [DWIDTH-1:0] C_1_5    = 64'h3FF8_0000_0000_0000; // 1.5
    localparam logic [DWIDTH-1:0] C_1_3    = 64'h3FD5_5555_5555_5555; // 1/3
    localparam logic [DWIDTH-1:0] C_1_6    = 64'h3FC5_5555_5555_5555; // 1/6
    localparam logic [DWIDTH-1:0] C_2_3    = 64'h3FE5_5555_5555_5555; // 2/3
    localparam logic [DWIDTH-1:0] C_1_4    = 64'h3FD0_0000_0000_0000; // 1/4
    localparam logic [DWIDTH-1:0] C_1_12   = 64'h3FB5_5555_5555_5555; // 1/12
    localparam logic [DWIDTH-1:0] C_5_12   = 64'h3FDB_6DB6_DB6D_B6DB; // 5/12
    localparam logic [DWIDTH-1:0] C_1_36   = 64'h3F72_2222_2222_2222; // 1/36

    // single multiplier reused
    logic              mul_valid;
    logic              mul_finish;
    logic [DWIDTH-1:0] mul_a, mul_b;
    logic [DWIDTH-1:0] mul_y;

    fp_multiplier u_mul (
        .clk    (clk),
        .valid  (mul_valid),
        .finish (mul_finish),
        .a      (mul_a),
        .b      (mul_b),
        .result (mul_y)
    );

    // internal powers
    logic [DWIDTH-1:0] dt_sq, dt_cu, dt_qu, dt_qi, dt_sx;

    typedef enum logic [4:0] {
        OP_DT_SQ,
        OP_2DT,
        OP_DT_CU,
        OP_DT_QU,
        OP_DT_QI,
        OP_DT_SX,

        OP_HALF_DT2,
        OP_3HALF_DT2,

        OP_HALF_DT3,
        OP_1_3_DT3,
        OP_1_6_DT3,
        OP_2_3_DT3,

        OP_1_4_DT4,
        OP_1_6_DT4,
        OP_1_12_DT4,
        OP_5_12_DT4,

        OP_1_6_DT5,
        OP_1_12_DT5,

        OP_1_36_DT6,

        OP_DONE
    } op_t;

    typedef enum logic [1:0] {S_IDLE, S_ISSUE, S_WAIT, S_DONE} st_t;

    st_t st;
    op_t op;

    // comb drive multiplier operands based on current op
    always_comb begin
        mul_valid = 1'b0;
        mul_a     = '0;
        mul_b     = '0;

        if (st == S_ISSUE) begin
            mul_valid = 1'b1;
            unique case (op)
                OP_DT_SQ:        begin mul_a = delta_t; mul_b = delta_t; end
                OP_2DT:          begin mul_a = C_2_0;   mul_b = delta_t; end
                OP_DT_CU:        begin mul_a = dt_sq;   mul_b = delta_t; end
                OP_DT_QU:        begin mul_a = dt_cu;   mul_b = delta_t; end
                OP_DT_QI:        begin mul_a = dt_qu;   mul_b = delta_t; end
                OP_DT_SX:        begin mul_a = dt_qi;   mul_b = delta_t; end

                OP_HALF_DT2:     begin mul_a = dt_sq;   mul_b = C_0_5;  end
                OP_3HALF_DT2:    begin mul_a = dt_sq;   mul_b = C_1_5;  end

                OP_HALF_DT3:     begin mul_a = dt_cu;   mul_b = C_0_5;  end
                OP_1_3_DT3:      begin mul_a = dt_cu;   mul_b = C_1_3;  end
                OP_1_6_DT3:      begin mul_a = dt_cu;   mul_b = C_1_6;  end
                OP_2_3_DT3:      begin mul_a = dt_cu;   mul_b = C_2_3;  end

                OP_1_4_DT4:      begin mul_a = dt_qu;   mul_b = C_1_4;  end
                OP_1_6_DT4:      begin mul_a = dt_qu;   mul_b = C_1_6;  end
                OP_1_12_DT4:     begin mul_a = dt_qu;   mul_b = C_1_12; end
                OP_5_12_DT4:     begin mul_a = dt_qu;   mul_b = C_5_12; end

                OP_1_6_DT5:      begin mul_a = dt_qi;   mul_b = C_1_6;  end
                OP_1_12_DT5:     begin mul_a = dt_qi;   mul_b = C_1_12; end

                OP_1_36_DT6:     begin mul_a = dt_sx;   mul_b = C_1_36; end
                default:         begin mul_a = '0;      mul_b = '0;     end
            endcase
        end
    end

    function automatic op_t next_op(input op_t cur);
        unique case (cur)
            OP_DT_SQ:       next_op = OP_2DT;
            OP_2DT:         next_op = OP_DT_CU;
            OP_DT_CU:       next_op = OP_DT_QU;
            OP_DT_QU:       next_op = OP_DT_QI;
            OP_DT_QI:       next_op = OP_DT_SX;

            OP_DT_SX:       next_op = OP_HALF_DT2;

            OP_HALF_DT2:    next_op = OP_3HALF_DT2;
            OP_3HALF_DT2:   next_op = OP_HALF_DT3;

            OP_HALF_DT3:    next_op = OP_1_3_DT3;
            OP_1_3_DT3:     next_op = OP_1_6_DT3;
            OP_1_6_DT3:     next_op = OP_2_3_DT3;

            OP_2_3_DT3:     next_op = OP_1_4_DT4;

            OP_1_4_DT4:     next_op = OP_1_6_DT4;
            OP_1_6_DT4:     next_op = OP_1_12_DT4;
            OP_1_12_DT4:    next_op = OP_5_12_DT4;

            OP_5_12_DT4:    next_op = OP_1_6_DT5;

            OP_1_6_DT5:     next_op = OP_1_12_DT5;
            OP_1_12_DT5:    next_op = OP_1_36_DT6;

            OP_1_36_DT6:    next_op = OP_DONE;
            default:        next_op = OP_DONE;
        endcase
    endfunction

    // registers + FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st   <= S_IDLE;
            op   <= OP_DT_SQ;

            dt_sq <= '0; dt_cu <= '0; dt_qu <= '0; dt_qi <= '0; dt_sx <= '0;

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

            done  <= 1'b0;
            valid <= 1'b0;
        end else begin
            done <= 1'b0;

            if (start) begin
                // restart sequence
                st    <= S_ISSUE;
                op    <= OP_DT_SQ;
                valid <= 1'b0;
            end

            unique case (st)
                S_IDLE: begin
                    // wait for start
                end

                S_ISSUE: begin
                    st <= S_WAIT;
                end

                S_WAIT: begin
                    if (mul_finish) begin
                        // latch result
                        unique case (op)
                            OP_DT_SQ:     begin dt_sq     <= mul_y; dt2 <= mul_y; end
                            OP_2DT:       begin delta_t2  <= mul_y; end
                            OP_DT_CU:     begin dt_cu     <= mul_y; end
                            OP_DT_QU:     begin dt_qu     <= mul_y; end
                            OP_DT_QI:     begin dt_qi     <= mul_y; end
                            OP_DT_SX:     begin dt_sx     <= mul_y; end

                            OP_HALF_DT2:  begin half_dt2   <= mul_y; end
                            OP_3HALF_DT2: begin three2_dt2 <= mul_y; end

                            OP_HALF_DT3:  begin half_dt3   <= mul_y; end
                            OP_1_3_DT3:   begin three_dt3  <= mul_y; end
                            OP_1_6_DT3:   begin sixth_dt3  <= mul_y; end
                            OP_2_3_DT3:   begin two3_dt3   <= mul_y; end

                            OP_1_4_DT4:   begin quarter_dt4 <= mul_y; end
                            OP_1_6_DT4:   begin sixth_dt4   <= mul_y; end
                            OP_1_12_DT4:  begin twive_dt4   <= mul_y; end
                            OP_5_12_DT4:  begin five12_dt4  <= mul_y; end

                            OP_1_6_DT5:   begin six_dt5     <= mul_y; end
                            OP_1_12_DT5:  begin twleve_dt5  <= mul_y; end

                            OP_1_36_DT6:  begin thirtysix_dt6 <= mul_y; end
                            default: ;
                        endcase

                        // advance
                        if (next_op(op) == OP_DONE) begin
                            st    <= S_DONE;
                        end else begin
                            op    <= next_op(op);
                            st    <= S_ISSUE;
                        end
                    end
                end

                S_DONE: begin
                    valid <= 1'b1;
                    done  <= 1'b1;
                    st    <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

