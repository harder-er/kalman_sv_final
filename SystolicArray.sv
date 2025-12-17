`timescale 1ns / 1ps

module SystolicArray #(
    parameter int DWIDTH   = 64,
    parameter int N        = 12,
    parameter int LATENCY  = 12
)(
    input  logic                clk,
    input  logic                rst_n,
    input  logic                load_en,
    input  logic [DWIDTH-1:0]   a_row  [0:N-1][0:N-1],
    input  logic [DWIDTH-1:0]   b_col  [0:N-1][0:N-1],
    input  logic                enb_1,
    input  logic                enb_2_6,
    input  logic                enb_7_12,
    output logic [DWIDTH-1:0]   c_out  [0:N-1][0:N-1],
    output logic                cal_finish
);

generate
if (N == 12) begin : GEN_TILED_12x12_T6
    localparam int T  = 6;
    localparam int NT = 2; // 12/6

    // ---------- tile indices ----------
    logic ti, tj, tk; // 0..1

    // ---------- edge-detect for start ----------
    logic load_en_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) load_en_d <= 1'b0;
        else        load_en_d <= load_en;
    end
    wire start = load_en & ~load_en_d;

    // ---------- core6 control ----------
    logic core6_load_en;
    logic core6_rst_n_int;
    wire  core6_rst_n = rst_n & core6_rst_n_int;

    // ---------- tile input/output buffers ----------
    logic [DWIDTH-1:0] a_tile   [0:T-1][0:T-1];
    logic [DWIDTH-1:0] b_tile   [0:T-1][0:T-1];
    logic [DWIDTH-1:0] core6_c  [0:T-1][0:T-1];
    logic              core6_done;
    logic [DWIDTH-1:0] tile_buf [0:T-1][0:T-1];

    // ---------- enable mapping ----------
    logic tile_enb_1, tile_enb_2_6;

    function automatic logic tile_j_enabled(input logic tj_in);
        if (tj_in == 1'b0) tile_j_enabled = (enb_1 | enb_2_6);  // cols 0..5
        else               tile_j_enabled = (enb_7_12);         // cols 6..11
    endfunction

    function automatic logic col_enabled(input int gcol);
        if (gcol == 0)                     col_enabled = enb_1;
        else if (gcol >= 1 && gcol <= 5)   col_enabled = enb_2_6;
        else if (gcol >= 6 && gcol <= 11)  col_enabled = enb_7_12;
        else                               col_enabled = 1'b0;
    endfunction

    always_comb begin
        if (tj == 1'b0) begin
            tile_enb_1   = enb_1;
            tile_enb_2_6 = enb_2_6;
        end else begin
            tile_enb_1   = enb_7_12;
            tile_enb_2_6 = enb_7_12;
        end
    end

    // ---------- build a_tile / b_tile from full matrices ----------
    always_comb begin
        int base_i, base_j, base_k;
        base_i = (ti ? T : 0);
        base_j = (tj ? T : 0);
        base_k = (tk ? T : 0);

        for (int i = 0; i < T; i++) begin
            for (int k = 0; k < T; k++) begin
                a_tile[i][k] = a_row[base_i + i][base_k + k];
            end
        end
        for (int k = 0; k < T; k++) begin
            for (int j = 0; j < T; j++) begin
                b_tile[k][j] = b_col[base_k + k][base_j + j];
            end
        end
    end

    // ---------- 6x6 core ----------
    SystolicArrayCore #(
        .DWIDTH  (DWIDTH),
        .N       (T),
        .LATENCY (LATENCY)
    ) u_core6 (
        .clk        ( clk          ),
        .rst_n      ( core6_rst_n   ),
        .load_en    ( core6_load_en ),
        .a_row      ( a_tile        ),
        .b_col      ( b_tile        ),
        .enb_1      ( tile_enb_1    ),
        .enb_2_6    ( tile_enb_2_6  ),
        .enb_7_12   ( 1'b0          ),
        .c_out      ( core6_c       ),
        .cal_finish ( core6_done    )
    );

    // ---------- single fp_adder for accumulation (tk=1) ----------
    logic              add_finish;
    logic              add_valid_r;
    logic [DWIDTH-1:0] add_a_r, add_b_r;
    wire  [DWIDTH-1:0] add_y;

    fp_adder u_add (
        .clk    ( clk        ),
        .a      ( add_a_r     ),
        .b      ( add_b_r     ),
        .valid  ( add_valid_r ),
        .finish ( add_finish  ),
        .result ( add_y       )
    );

    // ---------- accumulation indices ----------
    logic [2:0] ai, aj; // 0..5

    typedef enum logic [3:0] {
        S_IDLE,
        S_START_TILE,
        S_RUN_TILE,
        S_LATCH_TILE,
        S_WRITE_TK0,
        S_ACCUM_NEXT_ELEM,
        S_ACCUM_START,
        S_ACCUM_WAIT,
        S_NEXT_TILE,
        S_DONE
    } state_t;

    state_t st;

    // core6 state outputs（只驱动 core6_*，避免多驱动）
    always_comb begin
        core6_load_en   = 1'b0;
        core6_rst_n_int = 1'b0;

        // RUN_TILE：跑 core
        if (st == S_RUN_TILE) begin
            core6_rst_n_int = 1'b1;
            core6_load_en   = 1'b1;
        end
        // LATCH_TILE：保持不复位，避免 latch 时 core 输出被清
        else if (st == S_LATCH_TILE) begin
            core6_rst_n_int = 1'b1;
            core6_load_en   = 1'b0;
        end
    end

    // main FSM + adder transaction（adder 只在 always_ff 里驱动，彻底消除 multi-driver）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;
            ti <= 1'b0; tj <= 1'b0; tk <= 1'b0;
            ai <= '0;   aj <= '0;

            cal_finish  <= 1'b0;

            add_valid_r <= 1'b0;
            add_a_r     <= '0;
            add_b_r     <= '0;

            for (int i=0;i<N;i++)
              for (int j=0;j<N;j++)
                c_out[i][j] <= '0;

        end else begin
            if (!load_en) cal_finish <= 1'b0;

            unique case (st)
            S_IDLE: begin
                if (start) begin
                    for (int i=0;i<N;i++)
                      for (int j=0;j<N;j++)
                        c_out[i][j] <= '0;

                    ti <= 1'b0; tj <= 1'b0; tk <= 1'b0;
                    st <= S_START_TILE;
                end
            end

            S_START_TILE: begin
                if (!tile_j_enabled(tj)) st <= S_NEXT_TILE;
                else                     st <= S_RUN_TILE;
            end

            S_RUN_TILE: begin
                if (core6_done) st <= S_LATCH_TILE;
            end

            S_LATCH_TILE: begin
                for (int i=0;i<T;i++)
                  for (int j=0;j<T;j++)
                    tile_buf[i][j] <= core6_c[i][j];

                if (tk == 1'b0) st <= S_WRITE_TK0;
                else begin
                    ai <= '0; aj <= '0;
                    st <= S_ACCUM_NEXT_ELEM;
                end
            end

            S_WRITE_TK0: begin
                int base_i, base_j;
                base_i = (ti ? T : 0);
                base_j = (tj ? T : 0);

                for (int i=0;i<T;i++) begin
                    for (int j=0;j<T;j++) begin
                        int gi, gj;
                        gi = base_i + i;
                        gj = base_j + j;
                        c_out[gi][gj] <= col_enabled(gj) ? tile_buf[i][j] : '0;
                    end
                end

                tk <= 1'b1;
                st <= S_START_TILE;
            end

            S_ACCUM_NEXT_ELEM: begin
                int base_i, base_j, gi, gj;
                base_i = (ti ? T : 0);
                base_j = (tj ? T : 0);
                gi = base_i + ai;
                gj = base_j + aj;

                if (!col_enabled(gj)) begin
                    if (aj == T-1) begin
                        aj <= '0;
                        if (ai == T-1) st <= S_NEXT_TILE;
                        else           ai <= ai + 1'b1;
                    end else begin
                        aj <= aj + 1'b1;
                    end
                end else begin
                    st <= S_ACCUM_START;
                end
            end

            // 发起一次加法：锁存输入并把 valid 拉高（保持到 finish）
            S_ACCUM_START: begin
                if (!add_valid_r) begin
                    int base_i, base_j, gi, gj;
                    base_i = (ti ? T : 0);
                    base_j = (tj ? T : 0);
                    gi     = base_i + ai;
                    gj     = base_j + aj;

                    add_a_r     <= c_out[gi][gj];
                    add_b_r     <= tile_buf[ai][aj];
                    add_valid_r <= 1'b1;
                end
                st <= S_ACCUM_WAIT;
            end

            S_ACCUM_WAIT: begin
                if (add_valid_r && add_finish) begin
                    int base_i, base_j, gi, gj;
                    base_i = (ti ? T : 0);
                    base_j = (tj ? T : 0);
                    gi = base_i + ai;
                    gj = base_j + aj;

                    c_out[gi][gj] <= add_y;
                    add_valid_r   <= 1'b0;

                    if (aj == T-1) begin
                        aj <= '0;
                        if (ai == T-1) st <= S_NEXT_TILE;
                        else begin
                            ai <= ai + 1'b1;
                            st <= S_ACCUM_NEXT_ELEM;
                        end
                    end else begin
                        aj <= aj + 1'b1;
                        st <= S_ACCUM_NEXT_ELEM;
                    end
                end
            end

            S_NEXT_TILE: begin
                tk <= 1'b0;

                if (tj == NT-1) begin
                    tj <= 1'b0;
                    if (ti == NT-1) st <= S_DONE;
                    else begin
                        ti <= ti + 1'b1;
                        st <= S_START_TILE;
                    end
                end else begin
                    tj <= tj + 1'b1;
                    st <= S_START_TILE;
                end
            end

            S_DONE: begin
                cal_finish <= 1'b1;
                if (!load_en) st <= S_IDLE;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

end else begin : GEN_DIRECT_FALLBACK
    SystolicArrayCore #(
        .DWIDTH  (DWIDTH),
        .N       (N),
        .LATENCY (LATENCY)
    ) u_core (
        .clk        ( clk        ),
        .rst_n      ( rst_n      ),
        .load_en    ( load_en    ),
        .a_row      ( a_row      ),
        .b_col      ( b_col      ),
        .enb_1      ( enb_1      ),
        .enb_2_6    ( enb_2_6    ),
        .enb_7_12   ( enb_7_12   ),
        .c_out      ( c_out      ),
        .cal_finish ( cal_finish )
    );
end
endgenerate

endmodule
