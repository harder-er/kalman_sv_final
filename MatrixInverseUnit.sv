`timescale 1ns/1ps
`default_nettype none
`default_nettype wire

module MatrixInverseUnit #(
    parameter int DWIDTH = 64
)(
    input  logic                clk,
    input  logic                rst_n,

    // start（上层给 SP_Done 也行，但建议�?“start_pulse/�?pending+tp_valid�?触发�?    
    input  logic                valid,

    // TimeParamsSeq ready（非常关键：time 系数没好之前不要启动 CEU�?    
    input  logic                tp_valid,

    // time coefficients (来自 TimeParamsSeq)
    input  logic [DWIDTH-1:0]   delta_t,
    input  logic [DWIDTH-1:0]   delta_t2,        // 2*dt
    input  logic [DWIDTH-1:0]   dt2,             // dt^2
    input  logic [DWIDTH-1:0]   half_dt2,        // 1/2 dt^2
    input  logic [DWIDTH-1:0]   half_dt3,        // 1/2 dt^3
    input  logic [DWIDTH-1:0]   three_dt3,       // 1/3 dt^3
    input  logic [DWIDTH-1:0]   sixth_dt3,       // 1/6 dt^3
    input  logic [DWIDTH-1:0]   quarter_dt4,     // 1/4 dt^4
    input  logic [DWIDTH-1:0]   twive_dt4,       // 1/12 dt^4
    input  logic [DWIDTH-1:0]   five12_dt4,      // 5/12 dt^4
    input  logic [DWIDTH-1:0]   six_dt5,         // 1/6 dt^5
    input  logic [DWIDTH-1:0]   twleve_dt5,      // 1/12 dt^5
    input  logic [DWIDTH-1:0]   thirtysix_dt6,   // 1/36 dt^6

    input  logic [DWIDTH-1:0]   P_k1k1 [0:12-1][0:12-1],
    input  logic [DWIDTH-1:0]   Q_k    [0:12-1][0:12-1],
    input  logic [DWIDTH-1:0]   R_k    [0:5][0:5],

    output logic                finish,          // 1-cycle pulse
    output logic [DWIDTH-1:0]   inv_matrix [0:5][0:5]
);

    // ------------------------------------------------------------
    // 0) start pending（解�?SP_Done 先到、tp_valid 后到会丢触发的问题）
    // ------------------------------------------------------------
    logic valid_d;
    logic start_pulse;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_d <= 1'b0;
        else        valid_d <= valid;
    end
    assign start_pulse = valid & ~valid_d;

    logic pending;
    logic do_start;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) pending <= 1'b0;
        else begin
            if (start_pulse) pending <= 1'b1;
            else if (do_start) pending <= 1'b0;
        end
    end
    assign do_start = pending & tp_valid;  // tp_valid 起来才真的启�?
    // ------------------------------------------------------------
    // 1) CEU stage1：产�?a,b,c,d,e,f,x,y,z + valid_out
    //    �?gated reset：未启动�?CEU 保持 reset，避免乱�?    // ------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE,
        S_WAIT_STAGE1,
        S_WAIT_ALPHA,
        S_DIV,
        S_WAIT_NEG,
        S_WRITE,
        S_DONE
    } state_t;

    state_t st;

    logic ceu1_en, alpha_en;
    logic rst_ceu1_n, rst_alpha_n;
    assign rst_ceu1_n  = rst_n & ceu1_en;
    assign rst_alpha_n = rst_n & alpha_en;
    logic ceu1_start;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ceu1_start <= 1'b0;
        else        ceu1_start <= do_start;
    end

    // stage1 wires + done flags
    logic [DWIDTH-1:0] a_w,b_w,c_w,d_w,e_w,f_w,x_w,y_w,z_w;
    logic v_a,v_b,v_c,v_d,v_e,v_f,v_x,v_y,v_z;

    logic [DWIDTH-1:0] a,b,c,d,e,f,x,y,z;
    logic got_a,got_b,got_c,got_d,got_e,got_f,got_x,got_y,got_z;

    wire stage1_done = got_a & got_b & got_c & got_d & got_e & got_f & got_x & got_y & got_z;

    // ---- 这里保持你原来的 CEU 结构，但�?valid_out 分开、把 dt 映射修正 ----
    CEU_a #(.DBL_WIDTH(64)) u_CEU_a (
        .clk        (clk),
        .rst_n      (rst_ceu1_n),
        // .Theta_1_1  (P_k1k1[0][0]),
        .Theta_4_1  (P_k1k1[3][0]),
        .Theta_7_1  (P_k1k1[6][0]),
        .Theta_4_4  (P_k1k1[3][3]),
        .Theta_10_1 (P_k1k1[9][0]),
        .Theta_7_4  (P_k1k1[6][3]),
        .Theta_10_4 (P_k1k1[9][3]),
        .Theta_7_7  (P_k1k1[6][6]),
        .Theta_10_7 (P_k1k1[9][6]),
        .Theta_10_10(P_k1k1[9][9]),
        .Q_1_1      (Q_k[0][0]),
        .R_1_1      (R_k[0][0]),
        .dt_1       (delta_t2),
        .dt_2       (dt2),
        .dt_3       (three_dt3),
        .dt_4       (twive_dt4),
        .dt_5       (six_dt5),
        .dt_6       (thirtysix_dt6),
        .a_out      (a_w),
        .valid_out  (v_a)
    );

    CEU_a #(.DBL_WIDTH(64)) u_CEU_b (
        .clk        (clk),
        .rst_n      (rst_ceu1_n),
        // .Theta_1_1  (P_k1k1[1][1]),
        .Theta_4_1  (P_k1k1[4][1]),
        .Theta_7_1  (P_k1k1[7][1]),
        .Theta_4_4  (P_k1k1[4][4]),
        .Theta_10_1 (P_k1k1[10][1]),
        .Theta_7_4  (P_k1k1[7][4]),
        .Theta_10_4 (P_k1k1[10][4]),
        .Theta_7_7  (P_k1k1[7][7]),
        .Theta_10_7 (P_k1k1[10][7]),
        .Theta_10_10(P_k1k1[10][10]),
        .Q_1_1      (Q_k[1][1]),
        .R_1_1      (R_k[1][1]),
        .dt_1       (delta_t2),
        .dt_2       (dt2),
        .dt_3       (three_dt3),
        .dt_4       (twive_dt4),
        .dt_5       (six_dt5),
        .dt_6       (thirtysix_dt6),
        .a_out      (b_w),
        .valid_out  (v_b)
    );

    CEU_a #(.DBL_WIDTH(64)) u_CEU_c (
        .clk        (clk),
        .rst_n      (rst_ceu1_n),
        // .Theta_1_1  (P_k1k1[2][2]),
        .Theta_4_1  (P_k1k1[5][2]),
        .Theta_7_1  (P_k1k1[8][2]),
        .Theta_4_4  (P_k1k1[5][5]),
        .Theta_10_1 (P_k1k1[11][2]),
        .Theta_7_4  (P_k1k1[8][5]),
        .Theta_10_4 (P_k1k1[11][5]),
        .Theta_7_7  (P_k1k1[8][8]),
        .Theta_10_7 (P_k1k1[11][8]),
        .Theta_10_10(P_k1k1[11][11]),
        .Q_1_1      (Q_k[2][2]),
        .R_1_1      (R_k[2][2]),
        .dt_1       (delta_t2),
        .dt_2       (dt2),
        .dt_3       (three_dt3),
        .dt_4       (twive_dt4),
        .dt_5       (six_dt5),
        .dt_6       (thirtysix_dt6),
        .a_out      (c_w),
        .valid_out  (v_c)
    );

    CEU_d #(.DBL_WIDTH(64)) u_CEU_d (
        .clk         (clk),
        .rst_n       (rst_ceu1_n),
        //.valid_in    (ceu1_start),
        .Theta_10_7  (P_k1k1[9][6]),
        .Theta_7_4   (P_k1k1[6][3]),
        .Theta_10_4  (P_k1k1[9][3]),
        .Theta_4_7   (P_k1k1[3][6]),
        .Theta_10_10 (P_k1k1[9][9]),
        .Theta_4_4   (P_k1k1[3][3]),
        .Q_4_4       (Q_k[3][3]),
        .R_4_4       (R_k[3][3]),
        .delta_t2    (delta_t2),
        .delta_t_sq  (dt2),
        .delta_t_hcu (half_dt3),
        .delta_t_qr  (quarter_dt4),
        .d           (d_w),
        .valid_out   (v_d)
    );

    CEU_d #(.DBL_WIDTH(64)) u_CEU_e (
        .clk         (clk),
        .rst_n       (rst_ceu1_n),
        //.valid_in    (ceu1_start),
        .Theta_10_7  (P_k1k1[10][7]),
        .Theta_7_4   (P_k1k1[7][4]),
        .Theta_10_4  (P_k1k1[10][4]),
        .Theta_4_7   (P_k1k1[4][7]),
        .Theta_10_10 (P_k1k1[10][10]),
        .Theta_4_4   (P_k1k1[4][4]),
        .Q_4_4       (Q_k[4][4]),
        .R_4_4       (R_k[4][4]),
        .delta_t2    (delta_t2),
        .delta_t_sq  (dt2),
        .delta_t_hcu (half_dt3),
        .delta_t_qr  (quarter_dt4),
        .d           (e_w),
        .valid_out   (v_e)
    );

    CEU_d #(.DBL_WIDTH(64)) u_CEU_f (
        .clk         (clk),
        .rst_n       (rst_ceu1_n),
        //.valid_in    (ceu1_start),
        .Theta_10_7  (P_k1k1[11][8]),
        .Theta_7_4   (P_k1k1[8][5]),
        .Theta_10_4  (P_k1k1[11][5]),
        .Theta_4_7   (P_k1k1[5][8]),
        .Theta_10_10 (P_k1k1[11][11]),
        .Theta_4_4   (P_k1k1[5][5]),
        .Q_4_4       (Q_k[5][5]),
        .R_4_4       (R_k[5][5]),
        .delta_t2    (delta_t2),
        .delta_t_sq  (dt2),
        .delta_t_hcu (half_dt3),
        .delta_t_qr  (quarter_dt4),
        .d           (f_w),
        .valid_out   (v_f)
    );

    CEU_x #(.DBL_WIDTH(64)) u_CEU_x (
        .clk        (clk),
        .rst_n      (rst_ceu1_n),
        .Theta_1_7  (P_k1k1[0][6]),
        .Theta_4_4  (P_k1k1[3][3]),
        .Theta_7_4  (P_k1k1[6][3]),
        .Theta_10_4 (P_k1k1[9][3]),
        .Theta_7_7  (P_k1k1[6][6]),
        .Theta_10_1 (P_k1k1[9][0]),
        .Theta_10_7 (P_k1k1[9][6]),
        .Theta_10_10(P_k1k1[9][9]),
        .Theta_1_4  (P_k1k1[0][3]),
        .Q_1_4      (Q_k[0][3]),
        .R_1_4      (R_k[0][3]),
        .delta_t    (delta_t),
        .half_dt2   (half_dt2),
        .sixth_dt3  (sixth_dt3),
        .five12_dt4 (five12_dt4),
        .one12_dt5  (twleve_dt5),
        .x          (x_w),
        .valid_out  (v_x)
    );

    CEU_x #(.DBL_WIDTH(64)) u_CEU_y (
        .clk        (clk),
        .rst_n      (rst_ceu1_n),
        .Theta_1_7  (P_k1k1[1][7]),
        .Theta_4_4  (P_k1k1[4][4]),
        .Theta_7_4  (P_k1k1[7][4]),
        .Theta_10_4 (P_k1k1[10][4]),
        .Theta_7_7  (P_k1k1[7][7]),
        .Theta_10_1 (P_k1k1[10][1]),
        .Theta_10_7 (P_k1k1[10][7]),
        .Theta_10_10(P_k1k1[10][10]),
        .Theta_1_4  (P_k1k1[1][4]),
        .Q_1_4      (Q_k[1][4]),
        .R_1_4      (R_k[1][4]),
        .delta_t    (delta_t),
        .half_dt2   (half_dt2),
        .sixth_dt3  (sixth_dt3),
        .five12_dt4 (five12_dt4),
        .one12_dt5  (twleve_dt5),
        .x          (y_w),
        .valid_out  (v_y)
    );

    CEU_x #(.DBL_WIDTH(64)) u_CEU_z (
        .clk        (clk),
        .rst_n      (rst_ceu1_n),
        .Theta_1_7  (P_k1k1[2][8]),
        .Theta_4_4  (P_k1k1[5][5]),
        .Theta_7_4  (P_k1k1[8][5]),
        .Theta_10_4 (P_k1k1[11][5]),
        .Theta_7_7  (P_k1k1[8][8]),
        .Theta_10_1 (P_k1k1[11][2]),
        .Theta_10_7 (P_k1k1[11][8]),
        .Theta_10_10(P_k1k1[11][11]),
        .Theta_1_4  (P_k1k1[2][5]),
        .Q_1_4      (Q_k[2][5]),
        .R_1_4      (R_k[2][5]),
        .delta_t    (delta_t),
        .half_dt2   (half_dt2),
        .sixth_dt3  (sixth_dt3),
        .five12_dt4 (five12_dt4),
        .one12_dt5  (twleve_dt5),
        .x          (z_w),
        .valid_out  (v_z)
    );

    // ------------------------------------------------------------
    // 2) alpha stage
    // ------------------------------------------------------------
    logic [DWIDTH-1:0] alpha1_w, alpha2_w, alpha3_w;
    logic v_alpha1, v_alpha2, v_alpha3;
    logic [DWIDTH-1:0] alpha1, alpha2, alpha3;
    logic got_alpha1, got_alpha2, got_alpha3;
    wire alpha_done = got_alpha1 & got_alpha2 & got_alpha3;

    CEU_alpha u_alpha1 (.clk(clk), .rst_n(rst_alpha_n), .in1(a), .in2(d), .in3(x), .out(alpha1_w), .valid_out(v_alpha1));
    CEU_alpha u_alpha2 (.clk(clk), .rst_n(rst_alpha_n), .in1(b), .in2(e), .in3(y), .out(alpha2_w), .valid_out(v_alpha2));
    CEU_alpha u_alpha3 (.clk(clk), .rst_n(rst_alpha_n), .in1(c), .in2(f), .in3(z), .out(alpha3_w), .valid_out(v_alpha3));

    // ------------------------------------------------------------
    // 3) division (9x) + neg (3x)
    // ------------------------------------------------------------
    typedef enum logic [1:0] {DIV_IDLE, DIV_BUSY} div_state_e;
    div_state_e div_state;
    logic [3:0] div_idx;
    logic div_go, div_finish;
    logic [DWIDTH-1:0] div_num, div_den, div_q;

    CEU_division u_div (
        .clk        (clk),
        //.rst_n      (rst_n),
        .valid      (div_go),
        .finish     (div_finish),
        .numerator  (div_num),
        .denominator(div_den),
        .quotient   (div_q)
    );

    logic [DWIDTH-1:0] inv_a1, inv_d1, inv_x1;
    logic [DWIDTH-1:0] inv_b2, inv_e2, inv_y2;
    logic [DWIDTH-1:0] inv_c3, inv_f3, inv_z3;

    logic div_all_done;

    // negation by fp_suber
    logic neg_go;
    logic negx_finish, negy_finish, negz_finish;
    logic [DWIDTH-1:0] n_inv_x1, n_inv_y2, n_inv_z3;

    fp_suber u_negx (.clk(clk), 
    // .rst_n(rst_n), 
    .valid(neg_go), .finish(negx_finish), .a(64'h0), .b(inv_x1), .result(n_inv_x1));
    fp_suber u_negy (.clk(clk), 
    // .rst_n(rst_n), 
    .valid(neg_go), .finish(negy_finish), .a(64'h0), .b(inv_y2), .result(n_inv_y2));
    fp_suber u_negz (.clk(clk), 
    // rst_n(rst_n), 
    .valid(neg_go), 
    .finish(negz_finish), .a(64'h0), .b(inv_z3), .result(n_inv_z3));

    logic got_negx, got_negy, got_negz;
    wire  neg_done = got_negx & got_negy & got_negz;

    // ------------------------------------------------------------
    // 4) main FSM + latching
    // ------------------------------------------------------------
    integer i,j;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            ceu1_en  <= 1'b0;
            alpha_en <= 1'b0;

            // clear flags
            {got_a,got_b,got_c,got_d,got_e,got_f,got_x,got_y,got_z} <= '0;
            {got_alpha1,got_alpha2,got_alpha3} <= '0;

            div_state <= DIV_IDLE;
            div_idx   <= '0;
            div_go    <= 1'b0;
            div_all_done <= 1'b0;

            neg_go    <= 1'b0;
            {got_negx,got_negy,got_negz} <= '0;

            finish <= 1'b0;

            // init outputs
            for (i=0;i<6;i++) begin
                for (j=0;j<6;j++) begin
                    inv_matrix[i][j] <= '0;
                end
            end
        end else begin
            finish <= 1'b0;
            div_go <= 1'b0;
            neg_go <= 1'b0;
            div_all_done <= 1'b0;

            // start
            if (do_start) begin
                st       <= S_WAIT_STAGE1;
                ceu1_en  <= 1'b1;
                alpha_en <= 1'b0;

                {got_a,got_b,got_c,got_d,got_e,got_f,got_x,got_y,got_z} <= '0;
                {got_alpha1,got_alpha2,got_alpha3} <= '0;

                div_state <= DIV_IDLE;
                div_idx   <= '0;

                {got_negx,got_negy,got_negz} <= '0;
            end

            // -------- latch stage1 outputs
            if (st == S_WAIT_STAGE1) begin
                if (!got_a && v_a) begin a <= a_w; got_a <= 1'b1; end
                if (!got_b && v_b) begin b <= b_w; got_b <= 1'b1; end
                if (!got_c && v_c) begin c <= c_w; got_c <= 1'b1; end
                if (!got_d && v_d) begin d <= d_w; got_d <= 1'b1; end
                if (!got_e && v_e) begin e <= e_w; got_e <= 1'b1; end
                if (!got_f && v_f) begin f <= f_w; got_f <= 1'b1; end
                if (!got_x && v_x) begin x <= x_w; got_x <= 1'b1; end
                if (!got_y && v_y) begin y <= y_w; got_y <= 1'b1; end
                if (!got_z && v_z) begin z <= z_w; got_z <= 1'b1; end

                if (stage1_done) begin
                    alpha_en <= 1'b1;   // release alpha pipeline
                    st       <= S_WAIT_ALPHA;
                end
            end

            // -------- latch alpha
            if (st == S_WAIT_ALPHA) begin
                if (!got_alpha1 && v_alpha1) begin alpha1 <= alpha1_w; got_alpha1 <= 1'b1; end
                if (!got_alpha2 && v_alpha2) begin alpha2 <= alpha2_w; got_alpha2 <= 1'b1; end
                if (!got_alpha3 && v_alpha3) begin alpha3 <= alpha3_w; got_alpha3 <= 1'b1; end

                if (alpha_done) begin
                    st <= S_DIV;
                end
            end

            // -------- division 9x
            if (st == S_DIV) begin
                case (div_state)
                    DIV_IDLE: begin
                        div_idx <= 4'd0;
                        // start first division
                        div_num <= a;      div_den <= alpha1;
                        div_go  <= 1'b1;
                        div_state <= DIV_BUSY;
                    end

                    DIV_BUSY: begin
                        if (div_finish) begin
                            // store
                            unique case (div_idx)
                                4'd0: inv_a1 <= div_q;
                                4'd1: inv_d1 <= div_q;
                                4'd2: inv_x1 <= div_q;
                                4'd3: inv_b2 <= div_q;
                                4'd4: inv_e2 <= div_q;
                                4'd5: inv_y2 <= div_q;
                                4'd6: inv_c3 <= div_q;
                                4'd7: inv_f3 <= div_q;
                                4'd8: inv_z3 <= div_q;
                                default: ;
                            endcase

                            if (div_idx == 4'd8) begin
                                div_state <= DIV_IDLE;
                                div_all_done <= 1'b1;
                                st <= S_WAIT_NEG;
                                // kick negation (3 subers)
                                neg_go <= 1'b1;
                            end else begin
                                div_idx <= div_idx + 1'b1;
                                unique case (div_idx + 1'b1)
                                    4'd1: begin div_num <= d; div_den <= alpha1; end
                                    4'd2: begin div_num <= x; div_den <= alpha1; end
                                    4'd3: begin div_num <= b; div_den <= alpha2; end
                                    4'd4: begin div_num <= e; div_den <= alpha2; end
                                    4'd5: begin div_num <= y; div_den <= alpha2; end
                                    4'd6: begin div_num <= c; div_den <= alpha3; end
                                    4'd7: begin div_num <= f; div_den <= alpha3; end
                                    4'd8: begin div_num <= z; div_den <= alpha3; end
                                    default: begin div_num <= '0; div_den <= '0; end
                                endcase
                                div_go <= 1'b1;
                            end
                        end
                    end
                endcase
            end

            // -------- wait negation completes (3 independent finishes)
            if (st == S_WAIT_NEG) begin
                if (!got_negx && negx_finish) got_negx <= 1'b1;
                if (!got_negy && negy_finish) got_negy <= 1'b1;
                if (!got_negz && negz_finish) got_negz <= 1'b1;

                if (neg_done) begin
                    st <= S_WRITE;
                end
            end

            // -------- write output matrix (1-cycle)
            if (st == S_WRITE) begin
                // clear
                for (i=0;i<6;i++) begin
                    for (j=0;j<6;j++) begin
                        inv_matrix[i][j] <= '0;
                    end
                end

                // block1: rows/cols (0,3)
                inv_matrix[0][0] <= inv_d1;
                inv_matrix[0][3] <= n_inv_x1;
                inv_matrix[3][0] <= n_inv_x1;
                inv_matrix[3][3] <= inv_a1;

                // block2: rows/cols (1,4)
                inv_matrix[1][1] <= inv_e2;
                inv_matrix[1][4] <= n_inv_y2;
                inv_matrix[4][1] <= n_inv_y2;
                inv_matrix[4][4] <= inv_b2;

                // block3: rows/cols (2,5)
                inv_matrix[2][2] <= inv_f3;
                inv_matrix[2][5] <= n_inv_z3;
                inv_matrix[5][2] <= n_inv_z3;
                inv_matrix[5][5] <= inv_c3;

                finish <= 1'b1; // 1-cycle pulse
                st <= S_DONE;
            end

            if (st == S_DONE) begin
                // idle until next do_start
                st <= S_IDLE;
                ceu1_en  <= 1'b0;
                alpha_en <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire

