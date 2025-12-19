`timescale 1ns / 1ps

module SystolicArrayCore #(
    parameter DWIDTH      = 64,
    parameter N           = 12,
    parameter LATENCY     = 12,
    // Set to 1 to use the serial implementation, 0 to use the original systolic array.
    parameter bit SERIAL_MODE = 1'b1
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
    if (SERIAL_MODE) begin : GEN_SERIAL_CORE
        typedef enum logic [2:0] {
            S_IDLE,
            S_MUL,
            S_WAIT_MUL,
            S_ADD,
            S_WAIT_ADD,
            S_WRITE,
            S_DONE
        } serial_state_t;

        serial_state_t st;

        logic [$clog2(2*N-1)-1:0] cnt;
        logic [$clog2(N)-1:0]     idx_i, idx_j;
        logic [$clog2(N):0]       idx_k;
        logic [DWIDTH-1:0]        acc;

        logic                     mul_valid, add_valid;
        logic                     mul_finish, add_finish;
        logic [DWIDTH-1:0]        mul_res, add_res;
        logic                     busy;

        // cnt: 0..2N-2 while load_en=1
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)               cnt <= '0;
            else if (!load_en)        cnt <= '0;
            else if (cnt < 2*N-2)     cnt <= cnt + 1'b1;
        end

        function automatic logic column_enabled(input int col);
            if (col == 0)                         column_enabled = enb_1;
            else if (col >= 1 && col <= 5)        column_enabled = enb_2_6;
            else if (col >= 6 && col <= 11)       column_enabled = enb_7_12;
            else                                   column_enabled = 1'b0;
        endfunction

        fp_multiplier u_serial_mul(
            .clk    ( clk       ),
            .valid  ( mul_valid ),
            .a      ( a_row[idx_i][idx_k[$clog2(N)-1:0]] ),
            .b      ( b_col[idx_k[$clog2(N)-1:0]][idx_j] ),
            .finish ( mul_finish ),
            .result ( mul_res    )
        );

        fp_adder u_serial_add(
            .clk    ( clk       ),
            .valid  ( add_valid ),
            .a      ( acc       ),
            .b      ( mul_res   ),
            .finish ( add_finish ),
            .result ( add_res    )
        );

        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                st         <= S_IDLE;
                idx_i      <= '0;
                idx_j      <= '0;
                idx_k      <= '0;
                acc        <= '0;
                cal_finish <= 1'b0;
                mul_valid  <= 1'b0;
                add_valid  <= 1'b0;
                busy       <= 1'b0;
                for (int i = 0; i < N; i++)
                    for (int j = 0; j < N; j++)
                        c_out[i][j] <= '0;
            end else if (!load_en) begin
                st         <= S_IDLE;
                cal_finish <= 1'b0;
                mul_valid  <= 1'b0;
                add_valid  <= 1'b0;
                busy       <= 1'b0;
            end else begin
                // default: generate single-cycle launch pulses for mul/add
                mul_valid <= 1'b0;
                add_valid <= 1'b0;

                case (st)
                    S_IDLE: begin
                        cal_finish <= 1'b0;
                        if (!busy) begin
                            busy  <= 1'b1;
                            acc   <= '0;
                            idx_i <= '0;
                            idx_j <= '0;
                            idx_k <= '0;
                            for (int i = 0; i < N; i++)
                                for (int j = 0; j < N; j++)
                                    c_out[i][j] <= '0;
                            st <= S_MUL;
                        end
                    end

                    S_MUL: begin
                        if (!column_enabled(idx_j)) begin
                            c_out[idx_i][idx_j] <= '0;
                            idx_k <= '0;
                            acc   <= '0;

                            if (idx_j == N-1) begin
                                idx_j <= '0;
                                if (idx_i == N-1) begin
                                    st         <= S_DONE;
                                    cal_finish <= 1'b1;
                                end else begin
                                    idx_i <= idx_i + 1'b1;
                                    st    <= S_MUL;
                                end
                            end else begin
                                idx_j <= idx_j + 1'b1;
                                st    <= S_MUL;
                            end
                        end else begin
                            mul_valid <= 1'b1;
                            st        <= S_WAIT_MUL;
                        end
                    end

                    S_WAIT_MUL: begin
                        if (mul_finish) begin
                            st <= S_ADD;
                        end
                    end

                    S_ADD: begin
                        add_valid <= 1'b1;
                        st        <= S_WAIT_ADD;
                    end

                    S_WAIT_ADD: begin
                        if (add_finish) begin
                            acc <= add_res;
                            if (idx_k == N-1) begin
                                st <= S_WRITE;
                            end else begin
                                idx_k <= idx_k + 1'b1;
                                st    <= S_MUL;
                            end
                        end
                    end

                    S_WRITE: begin
                        c_out[idx_i][idx_j] <= acc;
                        acc   <= '0;
                        idx_k <= '0;

                        if (idx_j == N-1) begin
                            idx_j <= '0;
                            if (idx_i == N-1) begin
                                st         <= S_DONE;
                                cal_finish <= 1'b1;
                            end else begin
                                idx_i <= idx_i + 1'b1;
                                st    <= S_MUL;
                            end
                        end else begin
                            idx_j <= idx_j + 1'b1;
                            st    <= S_MUL;
                        end
                    end

                    S_DONE: begin
                        cal_finish <= 1'b1;
                        busy       <= 1'b0;
                        st         <= S_IDLE;
                    end

                    default: st <= S_IDLE;
                endcase
            end
        end
    end else begin : GEN_ORIGINAL_CORE
        logic [DWIDTH-1:0] a_reg [0:N-1][0:N];
        logic [DWIDTH-1:0] b_reg [0:N][0:N-1];
        logic [DWIDTH-1:0] sum_reg [0:N-1][0:N-1];
        logic              ready   [0:N-1][0:N-1];

        logic [$clog2(2*N-1)-1:0] cnt;

        // cnt: 0..2N-2 while load_en=1
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)               cnt <= '0;
            else if (!load_en)        cnt <= '0;
            else if (cnt < 2*N-2)     cnt <= cnt + 1'b1;
        end

        // Boundary input with zero padding when cnt >= N to avoid out-of-range access.
        always_ff @(posedge clk) begin
            if (load_en) begin
                for (int i = 0; i < N; i++) begin
                    a_reg[i][0] <= (cnt < N) ? a_row[i][cnt] : '0;
                    b_reg[0][i] <= (cnt < N) ? b_col[cnt][i] : '0;
                end
            end
        end

        for (genvar i = 0; i < N; i++) begin : GEN_ROW
            for (genvar j = 0; j < N; j++) begin : GEN_COL
                logic pe_en;
                assign pe_en = (j==0 && enb_1)
                            || (j>=1 && j<=5 && enb_2_6)
                            || (j>=6 && j<=11 && enb_7_12);

                ProcessingElement #(
                    .DWIDTH(DWIDTH),
                    .ADD_PIPE_STAGES(LATENCY)
                ) u_pe (
                    .clk        ( clk              ),
                    .rst_n      ( rst_n            ),
                    .en         ( load_en & pe_en  ),
                    .a_in       ( a_reg[i][j]      ),
                    .b_in       ( b_reg[i][j]      ),
                    .sum_down   ( sum_reg[i][j]    ),
                    .a_out      ( a_reg[i][j+1]    ),
                    .b_out      ( b_reg[i+1][j]    ),
                    .sum_right  ( sum_reg[i][j]    ),
                    .data_ready ( ready[i][j]      )
                );
            end
        end

        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                cal_finish <= 1'b0;
                for (int i = 0; i < N; i++)
                    for (int j = 0; j < N; j++)
                        c_out[i][j] <= '0;
            end else if (!load_en) begin
                cal_finish <= 1'b0;
            end else if (cnt == 2*N-2) begin
                cal_finish <= 1'b1;
                for (int i = 0; i < N; i++)
                    for (int j = 0; j < N; j++)
                        c_out[i][j] <= sum_reg[i][j];
            end
        end
    end
    endgenerate

endmodule
