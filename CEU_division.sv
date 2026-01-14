`timescale 1ns / 1ps

module CEU_division #(
    parameter int DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   valid,
    output logic                   finish,
    input  logic [DBL_WIDTH-1:0]   numerator,
    input  logic [DBL_WIDTH-1:0]   denominator,
    output logic [DBL_WIDTH-1:0]   quotient
);

    // ----------------------------
    // Internal regs (power-up init to avoid X in sim)
    // ----------------------------
    typedef enum logic [1:0] {S_IDLE, S_SEND, S_WAIT_OUT} state_t;
    state_t st = S_IDLE;

    logic [DBL_WIDTH-1:0] num_r = '0;
    logic [DBL_WIDTH-1:0] den_r = '0;

    logic a_tvalid_r = 1'b0;
    logic b_tvalid_r = 1'b0;
    logic a_accepted = 1'b0;
    logic b_accepted = 1'b0;

    logic a_tready;
    logic b_tready;

    logic [DBL_WIDTH-1:0] res_tdata;
    logic res_tvalid;
    logic res_tready;

    assign res_tready = 1'b1;

    // ----------------------------
    // Xilinx/AMD floating point divider IP (AXI-Stream style)
    // ----------------------------
    floating_point_div u_floating_point_div (
        .aclk                 ( clk        ),

        .s_axis_a_tdata       ( num_r      ),
        .s_axis_a_tvalid      ( a_tvalid_r ),
        .s_axis_a_tready      ( a_tready   ),

        .s_axis_b_tdata       ( den_r      ),
        .s_axis_b_tvalid      ( b_tvalid_r ),
        .s_axis_b_tready      ( b_tready   ),

        .m_axis_result_tdata  ( res_tdata  ),
        .m_axis_result_tvalid ( res_tvalid ),
        .m_axis_result_tready ( res_tready )
    );

    // ----------------------------
    // Handshake FSM
    // ----------------------------
    always_ff @(posedge clk) begin
        finish <= 1'b0;

        unique case (st)
            S_IDLE: begin
                a_tvalid_r <= 1'b0;
                b_tvalid_r <= 1'b0;
                a_accepted <= 1'b0;
                b_accepted <= 1'b0;

                if (valid) begin
                    num_r     <= numerator;
                    den_r     <= denominator;
                    a_tvalid_r <= 1'b1;
                    b_tvalid_r <= 1'b1;
                    st        <= S_SEND;
                end
            end

            S_SEND: begin
                // Hold TVALID until each channel handshakes
                if (!a_accepted && a_tvalid_r && a_tready) begin
                    a_tvalid_r <= 1'b0;
                    a_accepted <= 1'b1;
                end
                if (!b_accepted && b_tvalid_r && b_tready) begin
                    b_tvalid_r <= 1'b0;
                    b_accepted <= 1'b1;
                end

                if (a_accepted && b_accepted) begin
                    st <= S_WAIT_OUT;
                end
            end

            S_WAIT_OUT: begin
                if (res_tvalid) begin
                    quotient <= res_tdata;
                    finish   <= 1'b1;   // 1-cycle pulse
                    st       <= S_IDLE;
                end
            end

            default: st <= S_IDLE;
        endcase
    end

endmodule
