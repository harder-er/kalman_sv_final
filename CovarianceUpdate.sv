//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/25 19:46:55
// Design Name: 
// Module Name: CovarianceUpdate
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

module CovarianceUpdate #(
    parameter int STATE_DIM = 12,
    parameter int K_DIM     = 6,
    parameter int DWIDTH    = 64
)(
    input  logic                               clk,
    input  logic                               rst_n,

    input  logic [DWIDTH-1:0]                  K_k   [STATE_DIM-1:0][K_DIM-1:0],
    input  logic [DWIDTH-1:0]                  R_k   [K_DIM-1:0][K_DIM-1:0],
    input  logic [DWIDTH-1:0]                  P_kk1 [STATE_DIM-1:0][STATE_DIM-1:0],

    output logic [DWIDTH-1:0]                  P_kk  [STATE_DIM-1:0][STATE_DIM-1:0],

    input  logic                               CKG_Done,
    output logic                               SCU_Done
);

    localparam logic [DWIDTH-1:0] FP_ZERO = 64'h0000_0000_0000_0000;
    localparam logic [DWIDTH-1:0] FP_ONE  = 64'h3FF0_0000_0000_0000; // 1.0

    function automatic logic [DWIDTH-1:0] fp_neg(input logic [DWIDTH-1:0] x);
        fp_neg = {~x[DWIDTH-1], x[DWIDTH-2:0]}; // flip sign bit
    endfunction

    // ------------------------------------------------------------
    // start pulse detect (CKG_Done rising edge)
    // ------------------------------------------------------------
    logic ckg_done_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ckg_done_d <= 1'b0;
        else        ckg_done_d <= CKG_Done;
    end
    wire start = CKG_Done & ~ckg_done_d;

    // ------------------------------------------------------------
    // diag values for (I - K*H) diagonal (only i=0..5 needs 1-K[i][i])
    // store as 12 entries to avoid any out-of-range worries
    // diag_ikh[i] = IKH[i][i] when i<6 (computed), else 1.0 when i>=6
    // ------------------------------------------------------------
    logic [DWIDTH-1:0] diag_ikh [0:STATE_DIM-1];

    // ------------------------------------------------------------
    // Single fp_suber used for the 6 diagonal elements (1.0 - K[i][i])
    // ------------------------------------------------------------
    logic sub_valid, sub_finish;
    logic [DWIDTH-1:0] sub_a, sub_b, sub_y;

    fp_suber u_sub (.clk(clk),
        .valid  (sub_valid),
        .finish (sub_finish),
        .a      (sub_a),
        .b      (sub_b),
        .result (sub_y)
    );

    // Drive subtract operands from current diag index (stable in WAIT)
    logic [2:0] diag_idx; // 0..5
    always_comb begin
        sub_a = FP_ONE;
        sub_b = K_k[diag_idx][diag_idx];
    end

    // ------------------------------------------------------------
    // One shared SystolicArray
    // ------------------------------------------------------------
    logic sa_load_en;
    logic sa_done;

    logic sa_enb_1, sa_enb_2_6, sa_enb_7_12;

    logic [DWIDTH-1:0] sa_a [0:STATE_DIM-1][0:STATE_DIM-1];
    logic [DWIDTH-1:0] sa_b [0:STATE_DIM-1][0:STATE_DIM-1];
    logic [DWIDTH-1:0] sa_c [0:STATE_DIM-1][0:STATE_DIM-1];

    SystolicArray #(
        .DWIDTH  (DWIDTH),
        .N       (STATE_DIM),
        .LATENCY (12)
    ) u_sa (
        .clk        (clk),
        .rst_n      (rst_n),
        .load_en    (sa_load_en),
        .a_row      (sa_a),
        .b_col      (sa_b),
        .enb_1      (sa_enb_1),
        .enb_2_6    (sa_enb_2_6),
        .enb_7_12   (sa_enb_7_12),
        .c_out      (sa_c),
        .cal_finish (sa_done)
    );

    // ------------------------------------------------------------
    // Intermediate matrices (stored)
    // ------------------------------------------------------------
    logic [DWIDTH-1:0] KR        [0:STATE_DIM-1][0:STATE_DIM-1];
    logic [DWIDTH-1:0] KRKt      [0:STATE_DIM-1][0:STATE_DIM-1];
    logic [DWIDTH-1:0] IKHP      [0:STATE_DIM-1][0:STATE_DIM-1];
    logic [DWIDTH-1:0] IKHPIKHT  [0:STATE_DIM-1][0:STATE_DIM-1];

    // ------------------------------------------------------------
    // IKH element generator (I - K*H), with H selecting first 6 states:
    // KH = [K  0], so:
    //  - for j<6: IKH(i,j) = (i==j ? 1-K(i,i) : -K(i,j))
    //  - for j>=6: IKH(i,j) = (i==j ? 1 : 0)
    // ------------------------------------------------------------
    function automatic logic [DWIDTH-1:0] ikh_elem(input int i, input int j);
        if (j < K_DIM) begin
            if (i == j) ikh_elem = diag_ikh[i];       // i 0..5
            else        ikh_elem = fp_neg(K_k[i][j]); // -K
        end else begin
            ikh_elem = (i == j) ? FP_ONE : FP_ZERO;
        end
    endfunction

    // ------------------------------------------------------------
    // FSM: reuse systolic + reuse fp_adder
    // ------------------------------------------------------------
    typedef enum logic [3:0] {
        ST_IDLE,

        ST_SUB_DIAG_ISSUE,
        ST_SUB_DIAG_WAIT,

        ST_SA_KR_START,
        ST_SA_KR_WAIT,

        ST_SA_KRKt_START,
        ST_SA_KRKt_WAIT,

        ST_SA_IKHP_START,
        ST_SA_IKHP_WAIT,

        ST_SA_IKHPIKHT_START,
        ST_SA_IKHPIKHT_WAIT,

        ST_ADD_ISSUE,
        ST_ADD_WAIT,

        ST_DONE
    } st_t;

    st_t st;

    // ------------------------------------------------------------
    // One fp_adder used to compute P_kk = KRKt + IKHPIKHT (144 elems)
    // ------------------------------------------------------------
    logic add_valid, add_finish;
    logic [DWIDTH-1:0] add_a, add_b, add_y;

    fp_adder u_add (.clk(clk),
        .valid  (add_valid),
        .finish (add_finish),
        .a      (add_a),
        .b      (add_b),
        .result (add_y)
    );

    logic [3:0] add_i, add_j; // 0..11

    always_comb begin
        add_a = KRKt[add_i][add_j];
        add_b = IKHPIKHT[add_i][add_j];
    end

    // ------------------------------------------------------------
    // Drive sa_a/sa_b based on current stage
    // ------------------------------------------------------------
    typedef enum logic [2:0] {MODE_NONE, MODE_KR, MODE_KRKt, MODE_IKHP, MODE_IKHPIKHT} mode_t;
    mode_t mode;

    always_comb begin
        // default
        mode = MODE_NONE;
        if (st == ST_SA_KR_START || st == ST_SA_KR_WAIT)               mode = MODE_KR;
        else if (st == ST_SA_KRKt_START || st == ST_SA_KRKt_WAIT)      mode = MODE_KRKt;
        else if (st == ST_SA_IKHP_START || st == ST_SA_IKHP_WAIT)      mode = MODE_IKHP;
        else if (st == ST_SA_IKHPIKHT_START || st == ST_SA_IKHPIKHT_WAIT) mode = MODE_IKHPIKHT;

        // defaults for systolic inputs
        sa_enb_1    = 1'b1;
        sa_enb_2_6  = 1'b1;
        sa_enb_7_12 = 1'b1;

        for (int i = 0; i < STATE_DIM; i++) begin
            for (int j = 0; j < STATE_DIM; j++) begin
                sa_a[i][j] = FP_ZERO;
                sa_b[i][j] = FP_ZERO;
            end
        end

        unique case (mode)

            // KR = K(12x6) * R(6x6)
            MODE_KR: begin
                // compute only columns 0..5 (optional speed), resource not affected
                sa_enb_1    = 1'b1;
                sa_enb_2_6  = 1'b1;
                sa_enb_7_12 = 1'b0;

                for (int i = 0; i < STATE_DIM; i++) begin
                    for (int j = 0; j < STATE_DIM; j++) begin
                        sa_a[i][j] = (j < K_DIM) ? K_k[i][j] : FP_ZERO;
                        sa_b[i][j] = (i < K_DIM && j < K_DIM) ? R_k[i][j] : FP_ZERO;
                    end
                end
            end

            // KRKt = KR * K^T
            MODE_KRKt: begin
                sa_enb_1    = 1'b1;
                sa_enb_2_6  = 1'b1;
                sa_enb_7_12 = 1'b1;

                for (int i = 0; i < STATE_DIM; i++) begin
                    for (int j = 0; j < STATE_DIM; j++) begin
                        sa_a[i][j] = KR[i][j];
                        // K^T padded to 12x12: row 0..5 contains K^T
                        sa_b[i][j] = (i < K_DIM) ? K_k[j][i] : FP_ZERO;
                    end
                end
            end

            // IKHP = (I-KH) * P
            MODE_IKHP: begin
                sa_enb_1    = 1'b1;
                sa_enb_2_6  = 1'b1;
                sa_enb_7_12 = 1'b1;

                for (int i = 0; i < STATE_DIM; i++) begin
                    for (int j = 0; j < STATE_DIM; j++) begin
                        sa_a[i][j] = ikh_elem(i, j);
                        sa_b[i][j] = P_kk1[i][j];
                    end
                end
            end

            // IKHPIKHT = IKHP * (I-KH)^T
            MODE_IKHPIKHT: begin
                sa_enb_1    = 1'b1;
                sa_enb_2_6  = 1'b1;
                sa_enb_7_12 = 1'b1;

                for (int i = 0; i < STATE_DIM; i++) begin
                    for (int j = 0; j < STATE_DIM; j++) begin
                        sa_a[i][j] = IKHP[i][j];
                        sa_b[i][j] = ikh_elem(j, i); // transpose
                    end
                end
            end

            default: begin
                // keep zeros
            end
        endcase
    end

    // ------------------------------------------------------------
    // Sequential control
    // ------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= ST_IDLE;

            SCU_Done   <= 1'b0;
            sa_load_en <= 1'b0;
            sub_valid  <= 1'b0;
            add_valid  <= 1'b0;

            diag_idx <= '0;

            add_i <= '0;
            add_j <= '0;

            // init diag_ikh
            for (int i = 0; i < STATE_DIM; i++) begin
                diag_ikh[i] <= FP_ONE; // default diag = 1
            end

            // clear outputs
            for (int i = 0; i < STATE_DIM; i++) begin
                for (int j = 0; j < STATE_DIM; j++) begin
                    KR[i][j]       <= FP_ZERO;
                    KRKt[i][j]     <= FP_ZERO;
                    IKHP[i][j]     <= FP_ZERO;
                    IKHPIKHT[i][j] <= FP_ZERO;
                    P_kk[i][j]     <= FP_ZERO;
                end
            end

        end else begin
            // defaults each cycle
            sa_load_en <= 1'b0;
            sub_valid  <= 1'b0;
            add_valid  <= 1'b0;

            if (start) begin
                // restart a new covariance update run
                SCU_Done <= 1'b0;

                diag_idx <= 3'd0;
                add_i    <= 4'd0;
                add_j    <= 4'd0;

                // init diag_ikh (i>=6 stays 1.0)
                for (int i = 0; i < STATE_DIM; i++) begin
                    diag_ikh[i] <= FP_ONE;
                end

                st <= ST_SUB_DIAG_ISSUE;
            end

            unique case (st)

                ST_IDLE: begin
                    // wait start
                    if (!CKG_Done) SCU_Done <= 1'b0;
                end

                // ---- compute 6 diagonal values: diag_ikh[i] = 1 - K[i][i]
                ST_SUB_DIAG_ISSUE: begin
                    sub_valid <= 1'b1;
                    st        <= ST_SUB_DIAG_WAIT;
                end

                ST_SUB_DIAG_WAIT: begin
                    if (sub_finish) begin
                        diag_ikh[diag_idx] <= sub_y;
                        if (diag_idx == K_DIM-1) begin
                            st <= ST_SA_KR_START;
                        end else begin
                            diag_idx <= diag_idx + 1'b1;
                            st       <= ST_SUB_DIAG_ISSUE;
                        end
                    end
                end

                // ---- systolic: KR = K*R
                ST_SA_KR_START: begin
                    sa_load_en <= 1'b1;
                    st         <= ST_SA_KR_WAIT;
                end

                ST_SA_KR_WAIT: begin
                    sa_load_en <= 1'b0;  // �?拉低 load_en，允�?SystolicArray 完成
                    if (sa_done) begin
                        for (int i = 0; i < STATE_DIM; i++) begin
                            for (int j = 0; j < STATE_DIM; j++) begin
                                KR[i][j] <= sa_c[i][j];
                            end
                        end
                        st <= ST_SA_KRKt_START;
                    end
                end

                // ---- systolic: KRKt = KR * K^T
                ST_SA_KRKt_START: begin
                    sa_load_en <= 1'b1;
                    st         <= ST_SA_KRKt_WAIT;
                end

                ST_SA_KRKt_WAIT: begin
                    sa_load_en <= 1'b0;  // �?拉低 load_en
                    if (sa_done) begin
                        for (int i = 0; i < STATE_DIM; i++) begin
                            for (int j = 0; j < STATE_DIM; j++) begin
                                KRKt[i][j] <= sa_c[i][j];
                            end
                        end
                        st <= ST_SA_IKHP_START;
                    end
                end

                // ---- systolic: IKHP = (I-KH) * P
                ST_SA_IKHP_START: begin
                    sa_load_en <= 1'b1;
                    st         <= ST_SA_IKHP_WAIT;
                end

                ST_SA_IKHP_WAIT: begin
                    sa_load_en <= 1'b0;  // �?拉低 load_en
                    if (sa_done) begin
                        for (int i = 0; i < STATE_DIM; i++) begin
                            for (int j = 0; j < STATE_DIM; j++) begin
                                IKHP[i][j] <= sa_c[i][j];
                            end
                        end
                        st <= ST_SA_IKHPIKHT_START;
                    end
                end

                // ---- systolic: IKHPIKHT = IKHP * (I-KH)^T
                ST_SA_IKHPIKHT_START: begin
                    sa_load_en <= 1'b1;
                    st         <= ST_SA_IKHPIKHT_WAIT;
                end

                ST_SA_IKHPIKHT_WAIT: begin
                    sa_load_en <= 1'b0;  // �?拉低 load_en
                    if (sa_done) begin
                        for (int i = 0; i < STATE_DIM; i++) begin
                            for (int j = 0; j < STATE_DIM; j++) begin
                                IKHPIKHT[i][j] <= sa_c[i][j];
                            end
                        end
                        st <= ST_ADD_ISSUE;
                    end
                end

                // ---- fp add: P_kk = KRKt + IKHPIKHT (sequential 144 elems)
                ST_ADD_ISSUE: begin
                    add_valid <= 1'b1;
                    st        <= ST_ADD_WAIT;
                end

                ST_ADD_WAIT: begin
                    if (add_finish) begin
                        P_kk[add_i][add_j] <= add_y;

                        if (add_j == STATE_DIM-1) begin
                            add_j <= 0;
                            if (add_i == STATE_DIM-1) begin
                                st <= ST_DONE;
                            end else begin
                                add_i <= add_i + 1'b1;
                                st    <= ST_ADD_ISSUE;
                            end
                        end else begin
                            add_j <= add_j + 1'b1;
                            st    <= ST_ADD_ISSUE;
                        end
                    end
                end

                ST_DONE: begin
                    SCU_Done <= 1'b1;
                    // Return to IDLE after one cycle (when start goes low)
                    if (!CKG_Done) begin
                        st <= ST_IDLE;
                    end
                end

                default: st <= ST_IDLE;

            endcase
        end
    end

endmodule

