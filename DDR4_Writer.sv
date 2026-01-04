`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// DDR4_Writer (fixed)
// - Hold AWVALID/WVALID until handshake
// - WLAST asserted on final W beat (beat_idx == AWLEN)
// - Correct beat indexing and WDATA/WSTRB packing
// - X: ceil(STATE_DIM/8) beats, P: ceil(STATE_DIM*STATE_DIM/8) beats
//////////////////////////////////////////////////////////////////////////////////

module DDR4_Writer #(
    parameter int  STATE_DIM          = 12,
    parameter int  MEASURE_DIM        = 6,  // not used, keep for consistency
    parameter logic [31:0] ADDR_X_RESULT_BASE = 32'h0050_0000,
    parameter logic [31:0] ADDR_P_RESULT_BASE = 32'h0060_0000
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             write_en,

    input  logic [63:0]      X_kk_in [STATE_DIM-1:0],
    input  logic [63:0]      P_kk_in [STATE_DIM-1:0][STATE_DIM-1:0],

    // AXI4-Full Write
    output logic [31:0]      axi_awaddr,
    output logic [7:0]       axi_awlen,
    output logic [2:0]       axi_awsize,
    output logic [1:0]       axi_awburst,
    output logic             axi_awvalid,
    input  logic             axi_awready,

    output logic [511:0]     axi_wdata,
    output logic [63:0]      axi_wstrb,
    output logic             axi_wvalid,
    input  logic             axi_wready,
    output logic             axi_wlast,

    input  logic [1:0]       axi_bresp,
    input  logic             axi_bvalid,
    output logic             axi_bready
);

    // ----------------------------
    // constants
    // ----------------------------
    localparam int LANES_PER_BEAT = 8;           // 512b / 64b
    localparam int BYTES_PER_BEAT = 64;          // 512b = 64 bytes

    localparam int X_WORDS  = STATE_DIM;
    localparam int X_BEATS  = (X_WORDS + LANES_PER_BEAT - 1) / LANES_PER_BEAT; // ceil
    localparam int X_AWLEN  = (X_BEATS > 0) ? (X_BEATS - 1) : 0;

    localparam int P_WORDS  = STATE_DIM * STATE_DIM;
    localparam int P_BEATS  = (P_WORDS + LANES_PER_BEAT - 1) / LANES_PER_BEAT; // ceil
    localparam int P_AWLEN  = (P_BEATS > 0) ? (P_BEATS - 1) : 0;

    localparam int X_BW = (X_BEATS < 2) ? 1 : $clog2(X_BEATS);
    localparam int P_BW = (P_BEATS < 2) ? 1 : $clog2(P_BEATS);

    // ----------------------------
    // FSM
    // ----------------------------
    typedef enum logic [2:0] {
        S_IDLE     = 3'd0,
        S_AW_X     = 3'd1,
        S_W_X      = 3'd2,
        S_B_X      = 3'd3,
        S_AW_P     = 3'd4,
        S_W_P      = 3'd5,
        S_B_P      = 3'd6,
        S_DONE     = 3'd7
    } state_t;

    state_t state;

    // ----------------------------
    // AXI output regs
    // ----------------------------
    logic [31:0]  awaddr_r;
    logic [7:0]   awlen_r;
    logic [2:0]   awsize_r;
    logic [1:0]   awburst_r;
    logic         awvalid_r;

    logic [511:0] wdata_r;
    logic [63:0]  wstrb_r;
    logic         wvalid_r;
    logic         wlast_r;

    logic         bready_r;

    assign axi_awaddr  = awaddr_r;
    assign axi_awlen   = awlen_r;
    assign axi_awsize  = awsize_r;
    assign axi_awburst = awburst_r;
    assign axi_awvalid = awvalid_r;

    assign axi_wdata   = wdata_r;
    assign axi_wstrb   = wstrb_r;
    assign axi_wvalid  = wvalid_r;
    assign axi_wlast   = wlast_r;

    assign axi_bready  = bready_r;

    // handshake
    wire aw_hs = awvalid_r && axi_awready;
    wire w_hs  = wvalid_r  && axi_wready;
    wire b_hs  = bready_r  && axi_bvalid;

    // ----------------------------
    // beat counters
    // ----------------------------
    logic [X_BW-1:0] x_beat;
    logic [P_BW-1:0] p_beat;

    // ----------------------------
    // helpers: pack 1 beat
    // ----------------------------
    function automatic void pack_x_beat(
        input  int beat_idx,
        output logic [511:0] data_o,
        output logic [63:0]  strb_o
    );
        logic [511:0] d;
        logic [63:0]  s;
        int lane;
        int word_idx;
        begin
            d = '0;
            s = '0;
            for (lane = 0; lane < LANES_PER_BEAT; lane++) begin
                word_idx = beat_idx*LANES_PER_BEAT + lane;
                if (word_idx < X_WORDS) begin
                    d[lane*64 +: 64] = X_kk_in[word_idx];
                    s[lane*8  +: 8 ] = 8'hFF; // this 64b lane valid
                end
            end
            data_o = d;
            strb_o = s;
        end
    endfunction

    function automatic void pack_p_beat(
        input  int beat_idx,
        output logic [511:0] data_o,
        output logic [63:0]  strb_o
    );
        logic [511:0] d;
        logic [63:0]  s;
        int lane;
        int lin;
        int row, col;
        begin
            d = '0;
            s = 64'hFFFF_FFFF_FFFF_FFFF; // P is exact multiple for 12x12 (144), but keep full strobes
            for (lane = 0; lane < LANES_PER_BEAT; lane++) begin
                lin = beat_idx*LANES_PER_BEAT + lane;
                if (lin < P_WORDS) begin
                    row = lin / STATE_DIM;
                    col = lin % STATE_DIM;
                    d[lane*64 +: 64] = P_kk_in[row][col];
                end
            end
            data_o = d;
            strb_o = s;
        end
    endfunction

    // ----------------------------
    // sequential
    // ----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;

            awaddr_r  <= 32'd0;
            awlen_r   <= 8'd0;
            awsize_r  <= 3'b110;   // 64 bytes per beat
            awburst_r <= 2'b01;    // INCR
            awvalid_r <= 1'b0;

            wdata_r   <= '0;
            wstrb_r   <= '0;
            wvalid_r  <= 1'b0;
            wlast_r   <= 1'b0;

            bready_r  <= 1'b0;

            x_beat    <= '0;
            p_beat    <= '0;

        end else begin
            case (state)

                // ----------------
                // IDLE
                // ----------------
                S_IDLE: begin
                    awvalid_r <= 1'b0;
                    wvalid_r  <= 1'b0;
                    wlast_r   <= 1'b0;
                    bready_r  <= 1'b0;

                    if (write_en) begin
                        // prepare AW for X (single burst)
                        awaddr_r  <= ADDR_X_RESULT_BASE;
                        awlen_r   <= 8'(X_AWLEN);
                        awsize_r  <= 3'b110;
                        awburst_r <= 2'b01;
                        awvalid_r <= 1'b1;

                        x_beat    <= '0;
                        state     <= S_AW_X;
                    end
                end

                // ----------------
                // AW_X: hold until handshake
                // ----------------
                S_AW_X: begin
                    if (aw_hs) begin
                        awvalid_r <= 1'b0;

                        // prepare first W beat
                        pack_x_beat(0, wdata_r, wstrb_r);
                        wlast_r  <= (X_AWLEN == 0); // only 1 beat?
                        wvalid_r <= 1'b1;

                        state    <= S_W_X;
                    end
                end

                // ----------------
                // W_X: send beats 0..X_AWLEN
                // ----------------
                S_W_X: begin
                    if (w_hs) begin
                        if (x_beat == X_AWLEN[X_BW-1:0]) begin
                            // last beat just sent
                            wvalid_r <= 1'b0;
                            wlast_r  <= 1'b0;

                            bready_r <= 1'b1;
                            state    <= S_B_X;
                        end else begin
                            // advance to next beat
                            x_beat <= x_beat + 1'b1;

                            // prepare next beat data (use x_beat+1)
                            pack_x_beat(int'(x_beat) + 1, wdata_r, wstrb_r);
                            wlast_r  <= ((int'(x_beat) + 1) == X_AWLEN);
                            wvalid_r <= 1'b1; // keep asserted
                        end
                    end
                end

                // ----------------
                // B_X: wait response
                // ----------------
                S_B_X: begin
                    if (b_hs) begin
                        bready_r <= 1'b0;

                        // prepare AW for P
                        awaddr_r  <= ADDR_P_RESULT_BASE;
                        awlen_r   <= 8'(P_AWLEN);
                        awsize_r  <= 3'b110;
                        awburst_r <= 2'b01;
                        awvalid_r <= 1'b1;

                        p_beat    <= '0;
                        state     <= S_AW_P;
                    end
                end

                // ----------------
                // AW_P
                // ----------------
                S_AW_P: begin
                    if (aw_hs) begin
                        awvalid_r <= 1'b0;

                        pack_p_beat(0, wdata_r, wstrb_r);
                        wlast_r  <= (P_AWLEN == 0);
                        wvalid_r <= 1'b1;

                        state    <= S_W_P;
                    end
                end

                // ----------------
                // W_P
                // ----------------
                S_W_P: begin
                    if (w_hs) begin
                        if (p_beat == P_AWLEN[P_BW-1:0]) begin
                            wvalid_r <= 1'b0;
                            wlast_r  <= 1'b0;

                            bready_r <= 1'b1;
                            state    <= S_B_P;
                        end else begin
                            p_beat <= p_beat + 1'b1;

                            pack_p_beat(int'(p_beat) + 1, wdata_r, wstrb_r);
                            wlast_r  <= ((int'(p_beat) + 1) == P_AWLEN);
                            wvalid_r <= 1'b1;
                        end
                    end
                end

                // ----------------
                // B_P
                // ----------------
                S_B_P: begin
                    if (b_hs) begin
                        bready_r <= 1'b0;
                        state    <= S_DONE;
                    end
                end

                // ----------------
                // DONE: wait write_en deassert then go idle
                // ----------------
                S_DONE: begin
                    if (!write_en) begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // (optional) you can check bresp here if needed
    // e.g. if (b_hs && axi_bresp != 2'b00) $error("AXI write error");

endmodule
