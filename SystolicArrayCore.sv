`timescale 1ns / 1ps

module SystolicArrayCore #(
    parameter int DWIDTH   = 64,
    parameter int N        = 12,
    parameter int LATENCY  = 12   ) (
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

    // -----------------------------
    // load_en rising edge detect
    // -----------------------------
    logic load_en_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) load_en_d <= 1'b0;
        else        load_en_d <= load_en;
    end
    wire start_pulse = load_en & ~load_en_d;

    // -----------------------------
    // column enable function
    // -----------------------------
    function automatic logic col_enabled(input int unsigned gcol);
        if (N == 6) begin
            if (gcol == 0)                  col_enabled = enb_1;
            else if (gcol >= 1 && gcol <= 5) col_enabled = enb_2_6;
            else                             col_enabled = 1'b0;
        end else if (N == 12) begin
            if (gcol == 0)                     col_enabled = enb_1;
            else if (gcol >= 1 && gcol <= 5)   col_enabled = enb_2_6;
            else if (gcol >= 6 && gcol <= 11)  col_enabled = enb_7_12;
            else                               col_enabled = 1'b0;
        end else begin
            // 其它 N：默认全开（你也可以按需扩展�?            
            col_enabled = 1'b1;
        end
    endfunction

    // -----------------------------
    // shared fp ops (pulse valid)
    // -----------------------------
    logic              mul_go;
    logic              add_go;
    logic              mul_ready;
    logic              add_ready;
    logic              mul_finish;
    logic              add_finish;
    logic [DWIDTH-1:0] mul_a, mul_b;
    logic [DWIDTH-1:0] add_a, add_b;
    logic [DWIDTH-1:0] mul_y;
    logic [DWIDTH-1:0] add_y;

    fp_multiplier u_mul (
        .clk(clk),
        .rst_n(rst_n),
        .valid  (mul_go),
        .ready  (mul_ready),
        .finish (mul_finish),
        .a      (mul_a),
        .b      (mul_b),
        .result (mul_y)
    );

    fp_adder u_add (
        .clk(clk),
        .rst_n(rst_n),
        .valid  (add_go),
        .ready  (add_ready),
        .finish (add_finish),
        .a      (add_a),
        .b      (add_b),
        .result (add_y)
    );

    // -----------------------------
    // FSM indices/acc
    // -----------------------------
    localparam int IW = (N <= 1) ? 1 : $clog2(N);

    logic [IW-1:0] row_idx, col_idx, k_idx;
    logic [DWIDTH-1:0] acc;

    typedef enum logic [2:0] {
        S_IDLE,
        S_SKIP_OR_START, // 处理禁用列，启动第一个乘法器
        S_MUL_WAIT,
        S_ADD_WAIT,
        S_STORE,
        S_DONE_PULSE
    } state_t;

    state_t st;

    // -----------------------------
    // sequential
    // -----------------------------
    integer r, c;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            row_idx    <= '0;
            col_idx    <= '0;
            k_idx      <= '0;
            acc        <= '0;

            mul_go     <= 1'b0;
            add_go     <= 1'b0;

            cal_finish <= 1'b0;

            for (r = 0; r < N; r++) begin
                for (c = 0; c < N; c++) begin
                    c_out[r][c] <= '0;
                end
            end
        end else begin
            // defaults
            mul_go     <= 1'b0;
            add_go     <= 1'b0;
            cal_finish <= 1'b0;

            case (st)
                S_IDLE: begin
                    if (start_pulse) begin
                        // 清零输出（避免禁用列残留�?                        
                        for (r = 0; r < N; r++) begin
                            for (c = 0; c < N; c++) begin
                                c_out[r][c] <= '0;
                            end
                        end

                        row_idx <= '0;
                        col_idx <= '0;
                        k_idx   <= '0;
                        acc     <= '0;
                        st      <= S_SKIP_OR_START;
                    end
                end

                // 若该列禁用：强制输出 0 并跳到下一列；否则启动 mul(k=0)
                S_SKIP_OR_START: begin
                    int unsigned ci;
                    ci = col_idx;

                    if (!col_enabled(ci)) begin
                        c_out[row_idx][col_idx] <= '0;

                        // advance (row,col)
                        if (col_idx == N-1) begin
                            col_idx <= '0;
                            if (row_idx == N-1) st <= S_DONE_PULSE;
                            else begin
                                row_idx <= row_idx + 1'b1;
                                st      <= S_SKIP_OR_START;
                            end
                        end else begin
                            col_idx <= col_idx + 1'b1;
                            st      <= S_SKIP_OR_START;
                        end
                    end else begin
                        // start mul for k=0
                        k_idx <= '0;
                        acc   <= '0;

                        mul_a <= a_row[row_idx][0];
                        mul_b <= b_col[0][col_idx];
                        if (mul_ready) begin
                            mul_go <= 1'b1; // pulse
                            st    <= S_MUL_WAIT;
                        end
                    end
                end

                S_MUL_WAIT: begin
                    if (mul_finish) begin
                        add_a <= acc;
                        add_b <= mul_y;
                        if (add_ready) begin
                            add_go <= 1'b1; // pulse
                            st <= S_ADD_WAIT;
                        end
                    end
                end

                S_ADD_WAIT: begin
                    if (add_finish) begin
                        acc <= add_y;

                        if (k_idx == N-1) begin
                            st <= S_STORE;
                        end else begin
                            k_idx <= k_idx + 1'b1;

                            mul_a <= a_row[row_idx][k_idx + 1'b1];
                            mul_b <= b_col[k_idx + 1'b1][col_idx];
                            if (mul_ready) begin
                                mul_go <= 1'b1; // pulse
                                st <= S_MUL_WAIT;
                            end
                        end
                    end
                end

                S_STORE: begin
                    c_out[row_idx][col_idx] <= acc;

                    // advance (row,col)
                    if (col_idx == N-1) begin
                        col_idx <= '0;
                        if (row_idx == N-1) begin
                            st <= S_DONE_PULSE;
                        end else begin
                            row_idx <= row_idx + 1'b1;
                            st      <= S_SKIP_OR_START;
                        end
                    end else begin
                        col_idx <= col_idx + 1'b1;
                        st      <= S_SKIP_OR_START;
                    end
                end

                // finish pulse: assert for 1 cycle, independent of load_en
                S_DONE_PULSE: begin
                    cal_finish <= 1'b1;
                    st <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule



