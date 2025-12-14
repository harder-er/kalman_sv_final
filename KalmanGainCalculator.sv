//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/25 19:32:51
// Design Name: 
// Module Name: KalmanGainCalculator
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

module KalmanGainCalculator #(
    parameter DWIDTH = 64
)(
    input  logic                  clk                       ,
    input  logic                  rst_n                     ,
    input  logic [DWIDTH-1:0]     delta_t                   ,   // 时间间隔
    
    input  logic                  SP_Done                   ,   // 状态预测完成信号
    output logic                  CKG_Done                  ,   // 测量更新完成信号
    // 
    input  logic [DWIDTH-1:0]     P_k1k1 [0:12-1][0:12-1]   ,   //p_k-1,k-1
    input  logic [DWIDTH-1:0]     Q_k [0:12-1][0:12-1]      ,  
    // 
    input  logic [DWIDTH-1:0]     R_k [0:5][0:5]            ,  // 测量噪声矩阵
    
    // 输出接口（OMBus）
    output logic [DWIDTH-1:0]     K_k [0:12-1][0:6-1]  // 卡尔曼增益
);

// █████ CMU模块（矩阵计算单元）
//-----------------------------------------------------------------
// 参数定义
localparam N = 12;
localparam M = 6;
  
// 预测协方差矩阵
logic [DWIDTH-1:0] P_predicted    [0:N-1][0:N-1];
// 结果：P_predicted * H^T, 大小 12×6
logic [DWIDTH-1:0] P_predicted_HT [0:N-1][0:M-1];

// integer i, j;
always_comb begin
    for (integer i = 0; i < N; i = i + 1) begin
        for (integer j = 0; j < M; j = j + 1) begin
            // 直接取 P_predicted 的前 6 列
            P_predicted_HT[i][j] = P_predicted[i][j];
        end 
    end
end


// Output signals
logic [DWIDTH-1:0] delta_t2; // 2*dt
logic [DWIDTH-1:0] dt2;          // dt^2
logic [DWIDTH-1:0] half_dt2;     // 1/2*dt^2
logic [DWIDTH-1:0] three2_dt2;   // 3/2*dt^2
logic [DWIDTH-1:0] half_dt3;     // 1/2*dt^3
logic [DWIDTH-1:0] three_dt3;    // 1/3*dt^3
logic [DWIDTH-1:0] sixth_dt3;    // 1/6*dt^3
logic [DWIDTH-1:0] two3_dt3;     // 2/3*dt^3
logic [DWIDTH-1:0] quarter_dt4;  // 1/4*dt^4
logic [DWIDTH-1:0] sixth_dt4;    // 1/6*dt^4
logic [DWIDTH-1:0] twive_dt4;    // 1/12*dt^4
logic [DWIDTH-1:0] five12_dt4;   // 5/12*dt^4
logic [DWIDTH-1:0] six_dt5;      // 1/6*dt^5
logic [DWIDTH-1:0] twleve_dt5;   // 1/12*dt^5
logic [DWIDTH-1:0] thirtysix_dt6;// 1/36*dt^6

// Intermediate signals for powers of delta_t
// These are still sequential dependencies: dt^3 needs dt^2, dt^4 needs dt^3, etc.
logic [DWIDTH-1:0] dt_sq;    // dt^2
logic [DWIDTH-1:0] dt_cu;    // dt^3
logic [DWIDTH-1:0] dt_qu;    // dt^4
logic [DWIDTH-1:0] dt_qi;    // dt^5
logic [DWIDTH-1:0] dt_sx;    // dt^6

// Finish signals for each operation
// These are internal to the flow and define when a result is ready
logic finish_dt_sq, finish_dt_cu, finish_dt_qu, finish_dt_qi, finish_dt_sx;
logic finish_mult_2dt;
logic finish_mult_half_dt2, finish_mult_three2_dt2;
logic finish_mult_half_dt3, finish_mult_three_dt3, finish_mult_sixth_dt3, finish_mult_two3_dt3;
logic finish_mult_quarter_dt4, finish_mult_sixth_dt4, finish_mult_twive_dt4, finish_mult_five12_dt4;
logic finish_mult_six_dt5, finish_mult_twleve_dt5;
logic finish_mult_thirtysix_dt6;


// Constants (IEEE 754 double-precision floating-point)
localparam [DWIDTH-1:0] C_2_0      = 64'h4000000000000000; // 2.0
localparam [DWIDTH-1:0] C_0_5      = 64'h3fe0000000000000; // 0.5 (1/2)
localparam [DWIDTH-1:0] C_1_5      = 64'h3ff8000000000000; // 1.5 (3/2)
localparam [DWIDTH-1:0] C_0_333    = 64'h3fd5555555555555; // 1/3
localparam [DWIDTH-1:0] C_0_166    = 64'h3fc5555555555555; // 1/6
localparam [DWIDTH-1:0] C_0_666    = 64'h3fe5555555555555; // 2/3
localparam [DWIDTH-1:0] C_0_25     = 64'h3fd0000000000000; // 1/4
localparam [DWIDTH-1:0] C_0_0833   = 64'h3fb5555555555555; // 1/12
localparam [DWIDTH-1:0] C_0_416    = 64'h3fdb6db6db6db6db; // 5/12
localparam [DWIDTH-1:0] C_0_0277   = 64'h3f72222222222222; // 1/36

// --- Time Parameter Calculation Module ---

// delta_t^2 (Starts when global_valid is asserted)
fp_multiplier delta_t_sq (
    .clk(clk),
    .valid(global_valid),
    .finish(finish_dt_sq),
    .a(delta_t),
    .b(delta_t),
    .result(dt_sq)
);

// delta_t^3 (Starts as soon as dt_sq is ready)
fp_multiplier delta_t_cu (
    .clk(clk),
    .valid(finish_dt_sq), // Only depends on dt_sq being ready
    .finish(finish_dt_cu),
    .a(dt_sq),
    .b(delta_t),
    .result(dt_cu)
);

// delta_t^4 (Starts as soon as dt_cu is ready)
fp_multiplier delta_t_qu (
    .clk(clk),
    .valid(finish_dt_cu), // Only depends on dt_cu being ready
    .finish(finish_dt_qu),
    .a(dt_cu),
    .b(delta_t),
    .result(dt_qu)
);

// delta_t^5 (Starts as soon as dt_qu is ready)
fp_multiplier delta_t_qi (
    .clk(clk),
    .valid(finish_dt_qu), // Only depends on dt_qu being ready
    .finish(finish_dt_qi),
    .a(dt_qu),
    .b(delta_t),
    .result(dt_qi)
);

// delta_t^6 (Starts as soon as dt_qi is ready)
fp_multiplier delta_t_sx (
    .clk(clk),
    .valid(finish_dt_qi), // Only depends on dt_qi being ready
    .finish(finish_dt_sx),
    .a(dt_qi),
    .b(delta_t),
    .result(dt_sx)
);

// --- Coefficient Adjustment Module (Parallel Operations) ---

// 2*dt (Starts when global_valid is asserted, in parallel with dt_sq)
fp_multiplier mult_2dt (
    .clk(clk),
    .valid(global_valid),
    .finish(finish_mult_2dt),
    .a(C_2_0),
    .b(delta_t),
    .result(delta_t2)
);

// dt^2 (Direct assignment after dt_sq is ready)
assign dt2 = dt_sq;

// 1/2*dt^2 (Starts as soon as dt_sq is ready)
fp_multiplier mult_half_dt2 (
    .clk(clk),
    .valid(finish_dt_sq),
    .finish(finish_mult_half_dt2),
    .a(dt_sq),
    .b(C_0_5),
    .result(half_dt2)
);

// 3/2*dt^2 (Starts as soon as dt_sq is ready)
fp_multiplier mult_three2_dt2 (
    .clk(clk),
    .valid(finish_dt_sq),
    .finish(finish_mult_three2_dt2),
    .a(dt_sq),
    .b(C_1_5),
    .result(three2_dt2)
);

// 1/2*dt^3 (Starts as soon as dt_cu is ready)
fp_multiplier mult_half_dt3 (
    .clk(clk),
    .valid(finish_dt_cu),
    .finish(finish_mult_half_dt3),
    .a(dt_cu),
    .b(C_0_5),
    .result(half_dt3)
);

// 1/3*dt^3 (Starts as soon as dt_cu is ready)
fp_multiplier mult_three_dt3 (
    .clk(clk),
    .valid(finish_dt_cu),
    .finish(finish_three_dt3),
    .a(dt_cu),
    .b(C_0_333),
    .result(three_dt3)
);

// 1/6*dt^3 (Starts as soon as dt_cu is ready)
fp_multiplier mult_sixth_dt3 (
    .clk(clk),
    .valid(finish_dt_cu),
    .finish(finish_sixth_dt3),
    .a(dt_cu),
    .b(C_0_166),
    .result(sixth_dt3)
);

// 2/3*dt^3 (Starts as soon as dt_cu is ready)
fp_multiplier mult_two3_dt3 (
    .clk(clk),
    .valid(finish_dt_cu),
    .finish(finish_two3_dt3),
    .a(dt_cu),
    .b(C_0_666),
    .result(two3_dt3)
);

// 1/4*dt^4 (Starts as soon as dt_qu is ready)
fp_multiplier mult_quarter_dt4 (
    .clk(clk),
    .valid(finish_dt_qu),
    .finish(finish_quarter_dt4),
    .a(dt_qu),
    .b(C_0_25),
    .result(quarter_dt4)
);

// 1/6*dt^4 (Starts as soon as dt_qu is ready)
fp_multiplier mult_sixth_dt4 (
    .clk(clk),
    .valid(finish_dt_qu),
    .finish(finish_sixth_dt4),
    .a(dt_qu),
    .b(C_0_166),
    .result(sixth_dt4)
);

// 1/12*dt^4 (Starts as soon as dt_qu is ready)
fp_multiplier mult_twive_dt4 (
    .clk(clk),
    .valid(finish_dt_qu),
    .finish(finish_twive_dt4),
    .a(dt_qu),
    .b(C_0_0833),
    .result(twive_dt4)
);

// 5/12*dt^4 (Starts as soon as dt_qu is ready)
fp_multiplier mult_five12_dt4 (
    .clk(clk),
    .valid(finish_dt_qu),
    .finish(finish_five12_dt4),
    .a(dt_qu),
    .b(C_0_416),
    .result(five12_dt4)
);

// 1/6*dt^5 (Starts as soon as dt_qi is ready)
fp_multiplier mult_six_dt5 (
    .clk(clk),
    .valid(finish_dt_qi),
    .finish(finish_six_dt5),
    .a(dt_qi),
    .b(C_0_166),
    .result(six_dt5)
);

// 1/12*dt^5 (Starts as soon as dt_qi is ready)
fp_multiplier mult_twleve_dt5 (
    .clk(clk),
    .valid(finish_dt_qi),
    .finish(finish_twleve_dt5),
    .a(dt_qi),
    .b(C_0_0833),
    .result(twleve_dt5)
);

// 1/36*dt^6 (Starts as soon as dt_sx is ready)
fp_multiplier mult_thirtysix_dt6 (
    .clk(clk),
    .valid(finish_dt_sx),
    .finish(finish_thirtysix_dt6),
    .a(dt_sx),
    .b(C_0_0277),
    .result(thirtysix_dt6)
);

generate
    // Φ1,1块专用CMU
    genvar i;
    for (i = 0;i < 3;i ++) begin
        CMU_PHi11 #(
            .DBL_WIDTH(64)  // 双精度浮点位宽
        ) u_CMU_PHi11 (
            .clk          (clk),       
            .rst_n        (rst_n),     
            
            // 协方差矩阵参数输入（对应P矩阵位置）
            .Theta_1_1    (P_k1k1[0+i][0+i]),
            .Theta_4_1    (P_k1k1[3+i][0+i]),
            .Theta_7_1    (P_k1k1[6+i][0+i]),
            .Theta_4_4    (P_k1k1[3+i][3+i]),
            .Theta_10_1   (P_k1k1[9+i][0+i]),
            .Theta_4_7    (P_k1k1[3+i][6+i]),
            .Theta_7_7    (P_k1k1[6+i][6+i]),
            .Theta_4_10   (P_k1k1[3+i][9+i]),
            .Theta_7_10   (P_k1k1[6+i][9+i]),
            .Theta_10_10  (P_k1k1[9+i][9+i]),
            
            // 过程噪声参数
            .Q_1_1        (Q_k[0+i][0+i]), 
            
            // 时间参数（需预计算）
            .delta_t1     (delta_t2         ), //2delta
            .delta_t2     (dt2              ), //delta^2
            .delta_t3     (three_dt3        ), //1/3delta^3     
            .delta_t4     (twive_dt4        ), //1/12delta^4
            .delta_t5     (six_dt5          ), //1/6delta^5
            .delta_t6     (thirtysix_dt6    ), //1/36delta^6
            // 输出
            .a            (P_predicted[0+i][0+i]), 
            .valid_out    (phi11_valid)   
        ); 
    end
    
    for (i = 0;i < 3;i ++) begin
        CMU_PHi12 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi12_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),
            // —— 动态输入：索引全都减一 —— 
            .Theta_1_4   (P_k1k1[0+i][3+i]),   
            .Theta_1_1   (P_k1k1[0+i][0+i]),   
            .Theta_1_7   (P_k1k1[0+i][6+i]),   
            .Theta_4_7   (P_k1k1[3+i][6+i]),   
            .Theta_1_10  (P_k1k1[0+i][9+i]),   
            .Theta_4_10  (P_k1k1[3+i][9+i]),   
            .Theta_7_7   (P_k1k1[6+i][6+i]),   
            .Theta_7_10  (P_k1k1[6+i][9+i]),   
            .Theta_10_10 (P_k1k1[9+i][9+i]),   
            .Q_1_4       (Q_k   [0+i][3+i]),   

            // —— 时间参数（常量） —— 
            .delta_t     (delta_t),        
            .half_dt2    (half_dt2),       
            .sixth_dt3   (sixth_dt3),      
            .five12_dt4  (five12_dt4),     
            .twleve_dt5  (twleve_dt5),     

            // —— 输出 —— 
            .a           (P_predicted[0+i][1+i]),
            .valid_out   (phi12_valid_minus1)
        );
    end
   
    // 在上层模块中，使用 generate 块并行例化 CMU_PHi13 三条流水线（i = 0,1,2）
// 假设 P_k1k1 和 Q_k 是预先定义好的矩阵，P_predicted 用于保存输出


    for (i = 0; i < 3; i++) begin : GEN_PHI13
        CMU_PHi13 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi13 (
            .clk        (clk),
            .rst_n      (rst_n),
            // —— 动态输入：按照索引减一逻辑映射到 P_k1k1 —— 
            .Theta_1_7   (P_k1k1[0+i][6+i]),   // Θ[1,7] → P_k1k1[1+i-1][7+i-1]
            .Theta_4_7   (P_k1k1[3+i][6+i]),   // Θ[4,7] → P_k1k1[4+i-1][7+i-1]
            .Theta_1_10  (P_k1k1[0+i][9+i]),   // Θ[1,10]
            .Theta_7_7   (P_k1k1[6+i][6+i]),   // Θ[7,7]
            .Theta_4_10  (P_k1k1[3+i][9+i]),   // Θ[4,10]
            .Theta_7_10  (P_k1k1[6+i][9+i]),   // Θ[7,10]
            .Theta_10_10 (P_k1k1[9+i][9+i]),   // Θ[10,10]
            .Q_1_7       (Q_k   [0+i][6+i]),    // Q[1,7]

            // —— 时间参数（常量） —— 
            .delta_t     (delta_t),       
            .half_dt2    (half_dt2),      
            .two3_dt3    (two3_dt3),      
            .sixth_dt4   (sixth_dt4),     

            // —— 输出 —— 
            .a           (P_predicted[0+i][2+i]),   // 输出 a 存入 P_predicted[1+i][3+i]
            .valid_out   (phi13_valid_minus1)
        );
    end

    // 在上层模块中，使用 generate 并行例化 CMU_PHi14 三条流水线（i = 0,1,2），并将所有下标减一

    for (i = 0; i < 3; i++) begin : GEN_PHI14_MINUS1
        CMU_PHi14 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi14_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),
            // —— 动态输入：索引全都减一 —— 
            .Theta_1_10   (P_k1k1[0+i][9+i]),   // was [1][10]
            .Theta_4_10   (P_k1k1[3+i][9+i]),   // was [4][10]
            .Theta_7_10   (P_k1k1[6+i][9+i]),   // was [7][10]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_1_10       (Q_k   [0+i][9+i]),    // was [1][10]

            // —— 时间参数（常量） —— 
            .delta_t     (delta_t),
            .half_dt2    (half_dt2),
            .two3_dt3    (sixth_dt3),

            // —— 输出 —— 
            .a           (P_predicted[0+i][9+i]),  // 存入 [1,10] 减一后的 [0+i][9+i]
            .valid_out   (phi14_valid_minus1)
        );
    end

    // 在上层模块中，使用 generate 并行例化 CMU_PHi21 三条流水线（i = 0,1,2），并将所有下标减一

    for (i = 0; i < 3; i++) begin : GEN_PHI21_MINUS1
        CMU_PHi21 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi21_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),
            // —— 动态输入：索引全都减一 —— 
            .Theta_4_1   (P_k1k1[3+i][0+i]),   // was [4][1]
            .Theta_7_1   (P_k1k1[6+i][0+i]),   // was [7][1]
            .Theta_4_4   (P_k1k1[3+i][3+i]),   // was [4][4]
            .Theta_1_10  (P_k1k1[0+i][9+i]),   // was [1][10]
            .Theta_4_7   (P_k1k1[3+i][6+i]),   // was [4][7]
            .Theta_7_7   (P_k1k1[6+i][6+i]),   // was [7][7]
            .Theta_4_10  (P_k1k1[3+i][9+i]),   // was [4][10]
            .Theta_7_10  (P_k1k1[6+i][9+i]),   // was [7][10]
            .Theta_10_10 (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_4_1       (Q_k   [3+i][0+i]),    // was [4][1]

            // —— 时间参数（常量） —— 
            .delta_t        (delta_t),
            .dt2_half       (half_dt2),
            .dt3_sixth      (sixth_dt3),
            .dt4_twelth     (twive_dt4),
            .dt5_twelth     (twleve_dt5),
            .dt6_thirtysix  (thirtysix_dt6),

            // —— 输出 —— 
            .a           (P_predicted[3+i][0+i]), 
            .valid_out   (phi21_valid_minus1)
        );
    end

    // 在上层模块中，使用 generate 并行例化 CMU_PHi22 三条流水线（i = 0,1,2），并将所有下标减一

    for (i = 0; i < 3; i++) begin : GEN_PHI22_MINUS1
        CMU_PHi22 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi22_minus1 (
            .clk          (clk),
            .rst_n        (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_4_4    (P_k1k1[3+i][3+i]),   // was [4][4]
            .Theta_4_7    (P_k1k1[3+i][6+i]),   // was [4][7]
            .Theta_4_10   (P_k1k1[3+i][9+i]),   // was [4][10]
            .Theta_7_7    (P_k1k1[6+i][6+i]),   // was [7][7]
            .Theta_7_10   (P_k1k1[6+i][9+i]),   // was [7][10]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_4_4        (Q_k   [3+i][3+i]),    // was [4][4]

            // —— 时间参数（常量） —— 
            .two_dt       (delta_t2),
            .dt2          (dt2),
            .half_dt3     (half_dt3),
            .quarter_dt4  (quarter_dt4),

            // —— 输出 —— 
            .a            (P_predicted[3+i][3+i]),  // 将结果写回 [4,4] 减一后位置
            .valid_out    (phi22_valid_minus1)
        );
    end

// 在上层模块中，使用 generate 并行例化 CMU_PHi23 三条流水线（i = 0,1,2），并将所有下标减一

    for (i = 0; i < 3; i++) begin : GEN_PHI23_MINUS1
        CMU_PHi23 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi23_minus1 (
            .clk          (clk),
            .rst_n        (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_4_7    (P_k1k1[3+i][6+i]),   // was [4][7]
            .Theta_7_7    (P_k1k1[6+i][6+i]),   // was [7][7]
            .Theta_4_10   (P_k1k1[3+i][9+i]),   // was [4][10]
            .Theta_10_7   (P_k1k1[9+i][6+i]),   // was [10][7]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_4_7        (Q_k   [3+i][6+i]),    // was [4][7]

            // —— 时间参数（常量） —— 
            .delta_t      (delta_t),
            .half_dt2     (half_dt2),
            .half_dt3     (half_dt3),

            // —— 输出 —— 
            .a            (P_predicted[3+i][6+i]),  // 写回 [4,7] 减一后位置
            .valid_out    (phi23_valid_minus1)
        );
    end

    // 在上层模块中，使用 generate 并行例化 CMU_PHi24 三条流水线（i = 0,1,2），并将所有下标减一

    for (i = 0; i < 3; i++) begin : GEN_PHI24_MINUS1
        CMU_PHi24 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi24_minus1 (
            .clk          (clk),
            .rst_n        (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_4_10   (P_k1k1[3+i][9+i]),   // was Θ[4,10]
            .Theta_7_4    (P_k1k1[6+i][3+i]),   // was Θ[7,4]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was Θ[10,10]
            .Q_4_10       (Q_k   [3+i][9+i]),    // was Q[4,10]

            // —— 时间参数（常量） —— 
            .delta_t      (delta_t),
            .half_dt2     (half_dt2),

            // —— 输出 —— 
            .a            (P_predicted[3+i][9+i]),  // 写回 [4,10] 减一后位置
            .valid_out    (phi24_valid_minus1)
        );
    end

// 在上层模块中，使用 generate 并行例化 CMU_PHi31 三条流水线（i = 0,1,2），并将所有下标减一
    for (i = 0; i < 3; i++) begin : GEN_PHI31_MINUS1
        CMU_PHi31 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi31_minus1 (
            .clk        (clk),
            .rst_n      (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_7_1    (P_k1k1[6+i][0+i]),   // was Θ[7,1]
            .Theta_1_10   (P_k1k1[0+i][9+i]),   // was Θ[1,10]
            .Theta_4_7    (P_k1k1[3+i][6+i]),   // was Θ[4,7]
            .Theta_7_7    (P_k1k1[6+i][6+i]),   // was Θ[7,7]
            .Theta_4_10   (P_k1k1[3+i][9+i]),   // was Θ[4,10]
            .Theta_7_10   (P_k1k1[6+i][9+i]),   // was Θ[7,10]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was Θ[10,10]
            .Q_7_1        (Q_k   [6+i][0+i]),    // was Q[7,1]

            // —— 时间参数 —— 
            .delta_t      (delta_t),
            .half_dt2     (half_dt2),
            .two3_dt3     (two3_dt3),
            .sixth_dt4    (sixth_dt4),

            // —— 输出 —— 
            .a            (P_predicted[6+i][0+i]),  // 写回 [7,1] 减一后位置
            .valid_out    (phi31_valid_minus1)
        );
    end

// 在上层模块中，使用 generate 并行例化 CMU_PHi32 三条流水线（i = 0,1,2），并将所有下标减一

    for (i = 0; i < 3; i++) begin : GEN_PHI32_MINUS1
        CMU_PHi32 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi32_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_7_4   (P_k1k1[6+i][3+i]),   // was Θ[7,4]
            .Theta_4_10  (P_k1k1[3+i][9+i]),   // was Θ[4,10] (unused)
            .Theta_7_7   (P_k1k1[6+i][6+i]),   // was Θ[7,7]
            .Theta_7_10  (P_k1k1[6+i][9+i]),   // was Θ[7,10]
            .Theta_10_10 (P_k1k1[9+i][9+i]),   // was Θ[10,10]
            .Q_7_4       (Q_k   [6+i][3+i]),    // was Q[7,4]

            // —— 时间参数 —— 
            .delta_t     (delta_t),
            .three2_dt2  (three2_dt2),
            .half_dt3    (half_dt3),

            // —— 输出 —— 
            .a           (P_predicted[6+i][3+i]),  // 写回 [7,4] 减一后位置
            .valid_out   (phi32_valid_minus1)
        );
    end

// 在上层模块中，使用 generate 并行例化 CMU_PHi33 三条流水线（i = 0,1,2），并将所有下标减一
    for (i = 0; i < 3; i++) begin : GEN_PHI33_MINUS1
        CMU_PHi33 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi33_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_7_7    (P_k1k1[6+i][6+i]),   // was Θ[7,7]
            .Theta_7_10   (P_k1k1[6+i][9+i]),   // was Θ[7,10]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was Θ[10,10]
            .Q_7_7        (Q_k   [6+i][6+i]),    // was Q[7,7]

            // —— 时间参数 —— 
            .two_dt       (delta_t2 ),
            .dt2          (dt2      ),

            // —— 输出 —— 
            .a            (P_predicted[6+i][6+i]),  // 写回 [7,7] 减一后位置
            .valid_out    (phi33_valid_minus1)
        );
    end


    for (i = 0; i < 3; i++) begin : GEN_PHI34_MINUS1
        CMU_PHi34 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi34_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_7_10   (P_k1k1[6+i][9+i]),   // was Θ[7,10]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was Θ[10,10]
            .Q_7_10       (Q_k   [6+i][9+i]),    // was Q[7,10]

            // —— 时间参数 —— 
            .delta_t      (delta_t),

            // —— 输出 —— 
            .a            (P_predicted[6+i][9+i]),  // 写入 [7,10]，减一后为 [6+i][9+i]
            .valid_out    (phi34_valid_minus1)
        );
    end

    for (i = 0; i < 3; i++) begin : GEN_PHI41_MINUS1
        CMU_PHi41 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi41_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_10_1   (P_k1k1[9+i][0+i]),   // was [10][1]
            .Theta_10_4   (P_k1k1[9+i][3+i]),   // was [10][4]
            .Theta_10_7   (P_k1k1[9+i][6+i]),   // was [10][7]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_10_1       (Q_k   [9+i][0+i]),    // was [10][1]

            // —— 时间参数 —— 
            .delta_t      (delta_t),
            .half_dt2     (half_dt2),
            .sixth_dt3    (sixth_dt3),

            // —— 输出 —— 
            .a            (P_predicted[9+i][0+i]), // 对应 [10][1] 减一后索引
            .valid_out    (phi41_valid_minus1)
        );
    end


    for (i = 0; i < 3; i++) begin : GEN_PHI42_MINUS1
        CMU_PHi42 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi42_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_10_4   (P_k1k1[9+i][3+i]),   // was [10][4]
            .Theta_10_7   (P_k1k1[9+i][6+i]),   // was [10][7]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_10_4       (Q_k   [9+i][3+i]),    // was [10][4]

            // —— 时间参数 —— 
            .delta_t      (delta_t),
            .half_dt2     (half_dt2),

            // —— 输出 —— 
            .a            (P_predicted[9+i][3+i]),  // 对应 [10][4] 减一后索引
            .valid_out    (phi42_valid_minus1)
        );
    end


    for (i = 0; i < 3; i++) begin : GEN_PHI43_MINUS1
        CMU_PHi43 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi43_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_10_7   (P_k1k1[9+i][6+i]),   // was [10][7]
            .Theta_10_10  (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_10_7       (Q_k   [9+i][6+i]),    // was [10][7]

            // —— 时间参数 —— 
            .delta_t      (delta_t),

            // —— 输出 —— 
            .a            (P_predicted[9+i][6+i]),  // 对应 [10][7] 索引减一
            .valid_out    (phi43_valid_minus1)
        );
    end


    for (i = 0; i < 3; i++) begin : GEN_PHI44_MINUS1
        CMU_PHi44 #(
            .DBL_WIDTH(64)
        ) u_CMU_PHi44_minus1 (
            .clk         (clk),
            .rst_n       (rst_n),

            // —— 动态输入：索引全都减一 —— 
            .Theta_10_10 (P_k1k1[9+i][9+i]),   // was [10][10]
            .Q_10_10     (Q_k    [9+i][9+i]),   // was [10][10]

            // —— 输出 —— 
            .a           (P_predicted[9+i][9+i]), // was [10][10]
            .valid_out   (phi44_valid_minus1)
        );
    end
            

endgenerate


// █████ CEU模块（元素计算单元）
//-----------------------------------------------------------------
logic [DWIDTH-1:0] inv_matrix [0:5][0:5];
logic              inv_complete;

MatrixInverseUnit #(
    .DWIDTH(DWIDTH)
) u_MatrixInverseUnit (
    .clk        (clk            ),
    .rst_n      (rst_n          ),
    .P_k1k1     (P_k1k1         ),
    .R_k        (R_k            ),
    .Q_k        (Q_k            ),
    .valid      (SP_Done        ),
    .inv_matrix (inv_matrix     ),
    .finish     (inv_complete   )
);



// 扩展到 12×12
logic [DWIDTH-1:0] inv_matrix12 [0:11][0:11];

generate
  for (genvar i = 0; i < 12; i = i + 1) begin
    for (genvar j = 0; j < 12; j = j + 1) begin
      // 左上 6×6 保持原 inv_matrix6，其余位置填 0
      assign inv_matrix12[i][j] =
               (i < 6 && j < 6) ? inv_matrix[i][j]
                                : {DWIDTH{1'b0}};
    end
  end
endgenerate

logic [DWIDTH-1:0] P_predicted_HT12 [0:11][0:11];

generate
  for (genvar i = 0; i < 12; i = i + 1) begin
    for (genvar j = 0; j < 12; j = j + 1) begin
      // 左上 6×6 保持原 P_predicted_HT6，其余位置填 0
      assign P_predicted_HT12[i][j] =
               (j < 6) ? P_predicted_HT[i][j]
                                : {DWIDTH{1'b0}};
    end
  end
endgenerate

//-----------------------------------------------------------------
logic [DWIDTH-1:0] K_k_matrix [0:11][0:12-1]; // 卡尔曼增益矩阵
SystolicArray #(
    .DWIDTH(64),
    .N(12),
    .LATENCY(12)
) u_systolic (
    .clk        (   clk                 ),
    .rst_n      (   rst_n               ),
    .a_row      (   P_predicted_HT12    ),
    .b_col      (   inv_matrix12        ),
    .load_en    (   inv_complete        ),
    .enb_1      (   1'b1                ),
    .enb_2_6    (   1'b1                ),
    .enb_7_12   (   1'b0                ),
    .c_out      (   K_k_matrix          ) 
);

generate
    for (genvar i = 0; i < 12; i = i + 1) begin
        for (genvar j = 0; j < 6; j = j + 1) begin
            assign K_k[i][j] = K_k_matrix[i][j];
        end
    end
endgenerate


endmodule
