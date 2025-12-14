`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
//
// Create Date: 2025/03/10 11:35:25
// Module Name: KalmanFilterTop
// Description: 卡尔曼滤波器顶层模块，支持 12 维状态和 6 维测量
//////////////////////////////////////////////////////////////////////////////////

module kalman_core #(
    parameter STATE_DIM     = 12,
    parameter MEASURE_DIM   = 6 ,
    parameter deltat        = 0.01
)(
    input  logic                         clk        ,
    input  logic                         rst_n      ,
    input  logic                         start      ,


    // 系统模型参数
    input  logic [64-1:0]                Q_k    [STATE_DIM-1:0][STATE_DIM-1:0]      ,
    input  logic [64-1:0]                R_k    [MEASURE_DIM-1:0][MEASURE_DIM-1:0]  ,

    // 实时数据接口
    input  logic [64-1:0]                Z_k    [MEASURE_DIM-1:0]                   , // 观测值 
    input  logic                         En_MDI                                     ,
    input  logic [64-1:0]                X_00   [STATE_DIM-1:0]                     , // 初始状态      
    input  logic [64-1:0]                P_00   [STATE_DIM-1:0][STATE_DIM-1:0]      ,

    // 滤波结果输出
    output logic [63:0]                  X_kkout  [STATE_DIM-1:0]                   , // 估计状态
    output logic [63:0]                  P_kkout  [STATE_DIM-1:0][STATE_DIM-1:0]    ,

    output logic                         filter_done

);

    // -------------------------------------------------
    //  内部信号声明
    // -------------------------------------------------
    logic [63:0]                         X_k1k [STATE_DIM-1:0]; // x_k+1,k
    logic [63:0]                         X_kk1 [STATE_DIM-1:0]; // x_k,k-1
    logic [63:0]                         X_kk  [STATE_DIM-1:0]; // x_k,k


    logic [63:0]                         P_k1k        [STATE_DIM-1:0][STATE_DIM-1:0]; // P_k+1,k
    logic [63:0]                         P_kk1        [STATE_DIM-1:0][STATE_DIM-1:0]; // P_k,k-1
    logic [63:0]                         P_kk         [STATE_DIM-1:0][STATE_DIM-1:0]; // P_k,k
    logic [63:0]                         P_k1k1       [STATE_DIM-1:0][STATE_DIM-1:0]; // P_k-1,k-1
    
    logic [63:0]                         K_k      [STATE_DIM-1:0][MEASURE_DIM-1:0];

    logic [63:0]                         F        [STATE_DIM-1:0][STATE_DIM-1:0]; // K_k


    logic       Init_Valid    ;
    logic       SP_Done       ;
    logic       SCU_Done_s    ;
    logic       SCU_Done_p    ;
    logic       CKG_Done      ;
    logic       SCO_Valid     ;
    logic       MDI_Valid     ;
    logic       End_valid     ;
    logic       en_init       ;
    logic       en_sp         ;
    logic       en_ckg        ;
    logic       en_scu        ;
    logic       en_sco        ;
    logic       finish        ;
    // -------------------------------------------------
    // 控制单元
    // -------------------------------------------------
    KF_ControlUnit u_KF_ControlUnit(
    .clk        (  clk        ),
    .rst_n      (  rst_n      ),
    .Init_Valid (  Init_Valid ),
    .SP_Done    (  SP_Done    ),
    .SCU_Done_s (  SCU_Done_s   ),
    .SCU_Done_p (  SCU_Done_p   ),
    .CKG_Done   (  CKG_Done   ),
    .SCO_Valid  (  SCO_Valid  ),
    .MDI_Valid  (  MDI_Valid  ),
    .End_valid  (  End_valid  ),
    .en_init    (  en_init    ),
    .en_sp      (  en_sp      ),
    .en_ckg     (  en_ckg     ),
    .en_scu     (  en_scu     ),
    .en_sco     (  en_sco     ),
    .finish     (  finish     )
);


    F_make u_Fmake (
        .clk        ( clk       ),
        .rst_n      ( rst_n     ),
        .finish     ( F_finish  ),
        .deltat     ( deltat    ),
        .F          ( F         )
    );
    // -------------------------------------------------
    // 状态预测
    // -------------------------------------------------
    StateUpdate u_StateUpdator (
        .clk            ( clk        ),
        .rst_n          ( rst_n      ),
        .F              ( F          ),
        .X_kk           ( X_kk       ),
        .X_k1k          ( X_k1k      ),
        .CKG_Done       ( en_ckg     ),
        .MDI_Valid      ( en_mdi     ),
        .SCU_Done       ( SCU_Done_s ) 
    );


    KalmanGainCalculator #(
        .DWIDTH    (64)
    ) u_KalmanGainCalc (
        .clk             ( clk       ),
        .rst_n           ( rst_n     ),
        .Q_k             ( Q_k       ),
        .delta_t         ( deltat    ),
        .P_k1k1          ( P_k1k1    ),
        .R_k             ( R_k       ),
        .K_k             ( K_k       ),
        .SP_Done         ( en_sp     ),
        .CKG_Done        ( CKG_Done  )  
    );


    // -------------------------------------------------
    // 状态更新
    // -------------------------------------------------
    StatePredictor #(
        .VEC_WIDTH(64),
        .MAT_DIM  (12)
    ) u_StatePredictor (
        .clk           ( clk        ),
        .rst_n         ( rst_n      ),
        .K_k           ( K_k        ),
        .Z_k           ( Z_k        ),
        .X_kk1         ( X_kk1      ),
        .X_kk          ( X_kk       ),
        .Init_Valid    ( en_init    ),
        .SP_DONE       ( SP_Done    )
    );

    // -------------------------------------------------
    // 协方差更新
    // -------------------------------------------------
    CovarianceUpdate #(
        .STATE_DIM(STATE_DIM),
        .DWIDTH   (64)
    ) u_CovUpdate (
        .clk           ( clk         ),
        .rst_n         ( rst_n       ),
        .K_k           ( K_k         ),
        .R_k           ( R_k         ),
        .P_kk1         ( P_kk1       ),
        .P_kk          ( P_kk        ),
        .CKG_Done      ( en_ckg      ),
        .SCU_Done      ( SCU_Done    ) 
    );
    logic [64-1:0] Xk1k_delay [STATE_DIM-1:0][0:0];
    logic [64-1:0] Xkk1_delay [STATE_DIM-1:0][0:0];
generate
    for (genvar i = 0; i < STATE_DIM; i++) begin : gen_Xk1k_delay
        assign Xk1k_delay[i][0] = X_k1k[i];
    end
    for (genvar i = 0; i < STATE_DIM; i++) begin : gen_Xkk1_delay
        assign X_kk1[i] = Xkk1_delay[i][0];
    end
    
endgenerate
    DelayUnit #(
        .DELAY_CYCLES(1 ),
        .ROWS        (12),
        .COLS        (1 ),
        .DATA_WIDTH  (64)
    ) u_DelayX (
        .clk       ( clk          ),
        .rst_n     ( rst_n        ),
        .data_in   ( Xk1k_delay   ),
        .data_out  ( Xkk1_delay   )
    );

    DelayUnit #(
        .DELAY_CYCLES(1 ),
        .ROWS        (12),
        .COLS        (12),
        .DATA_WIDTH  (64)
    ) u_DelayP (
        .clk       ( clk        ),
        .rst_n     ( rst_n      ),
        .data_in   ( P_kk       ),
        .data_out  ( P_k1k1     )
    );


endmodule
