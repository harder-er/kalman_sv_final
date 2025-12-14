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
    parameter STATE_DIM = 12,
    parameter K_DIM = 6,
    parameter DWIDTH = 64
)(
    input  logic                    clk                                     ,
    input  logic                    rst_n                                   ,          

    input  logic [DWIDTH-1:0]       K_k     [STATE_DIM-1:0][K_DIM-1:0]      ,
    input  logic [DWIDTH-1:0]       R_k     [K_DIM-1:0][K_DIM-1:0]          ,
    input  logic [DWIDTH-1:0]       P_kk1   [STATE_DIM-1:0][STATE_DIM-1:0]  ,

    output logic [DWIDTH-1:0]       P_kk    [STATE_DIM-1:0][STATE_DIM-1:0]  ,

    input  logic                    CKG_Done                                ,
    output logic                    SCU_Done
);


logic [DWIDTH-1:0] Kkmatrix [STATE_DIM-1:0][6-1:0]; // Kk结果
logic [DWIDTH-1:0] KkTmatrix [6-1:0][STATE_DIM-1:0]; // Kk转置结果


MatrixTransBridge #(
    .ROWS(12),        
    .COLS(6),       
    .DATA_WIDTH(DWIDTH) 
) u_MatrixBridge (
    .clk        (   clk             ),            
    .rst_n      (   rst_n           ),         
    .mat_in     (   K_k             ),  
    .mat_org    (   Kkmatrix        ),  
    .mat_trans  (   KkTmatrix       ), 
    .valid_out  (   process_done    ) 
);

logic [DWIDTH-1:0] KkH  [STATE_DIM-1:0][STATE_DIM-1:0]; 
logic [DWIDTH-1:0] IKkH [STATE_DIM-1:0][STATE_DIM-1:0];

generate
    // Kk转置计算单元（对应图示路径②）
    for (genvar i = 0; i < STATE_DIM; i++) begin : gen_KkT
        for (genvar j = 6; j < STATE_DIM; j++) begin : gen_KkT_col
            assign KkH[i][j] = 64'h0; // 转置操作
        end
    end

    for (genvar i = 0; i < STATE_DIM; i++) begin : gen_KkT_row
        for (genvar j = 0; j < 6; j++) begin : gen_KkT_col
            assign KkH[i][j] = Kkmatrix[i][j]; // 转置操作
        end
    end

    
endgenerate

generate
    for (genvar i = 0; i < STATE_DIM; i++) begin : gen_IKkH
        for (genvar j = 0; j < STATE_DIM; j++) begin : gen_IKkH_col
            begin
                if (i == j) begin
                    fp_suber u_fp_suber(
                        .clk    (   clk         ),   
                        .valid  (   1'b1        ),
                        .finish (               ),
                        .a      (   64'h3FF0000000000000), 
                        .b      (   KkH[i][j]   ), 
                        .result (   IKkH[i][j]  ) 
                    );
                end else begin
                    fp_suber u_fp_suber(
                        .clk    (   clk         ),
                        .valid  (   1'b1        ),
                        .finish (               ),
                        .a      (   64'h000000000000000), 
                        .b      (   KkH[i][j]   ), 
                        .result (   IKkH[i][j]  ) 
                    );
                end
            end
        end
    end
endgenerate
    
    logic [DWIDTH-1:0] K_k_matrix           [STATE_DIM-1:0][STATE_DIM-1:0]; 
    logic [DWIDTH-1:0] R_k_matrix           [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [DWIDTH-1:0] KR_result            [STATE_DIM-1:0][STATE_DIM-1:0]; 
    logic [DWIDTH-1:0] KkTmatrix_systolic   [STATE_DIM-1:0][STATE_DIM-1:0]; // 矩阵赋值

    logic [DWIDTH-1:0] IKHmatrix    [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [DWIDTH-1:0] IKHT         [STATE_DIM-1:0][STATE_DIM-1:0];
    logic [DWIDTH-1:0] IKHP         [STATE_DIM-1:0][STATE_DIM-1:0];

    logic [DWIDTH-1:0] IKHPIKHT     [STATE_DIM-1:0][STATE_DIM-1:0]; // 矩阵赋值
    logic [DWIDTH-1:0] KRKt         [STATE_DIM-1:0][STATE_DIM-1:0]; // 矩阵赋值
generate
    for (genvar i = 0; i < STATE_DIM; i++) begin 
        for (genvar j = 0; j < STATE_DIM; j++) begin 
            if (j < 6) begin
                assign K_k_matrix[i][j] = K_k[i][j]; // K矩阵赋值
            end else begin
                assign K_k_matrix[i][j] = 64'h0; // 非K矩阵元素赋值为0
            end
        end
    end
    for (genvar i = 0; i < STATE_DIM; i++) begin 
        for (genvar j = 0; j < STATE_DIM; j++) begin 
            if (i < 6 && j < 6) begin
                assign R_k_matrix[i][j] = R_k[i][j]; // K矩阵赋值
            end else begin
                assign R_k_matrix[i][j] = 64'h0; // 非K矩阵元素赋值为0
            end
        end
    end
    // K*R计算单元（对应图示路径①）
    SystolicArray #(
        .DWIDTH(64),
        .N(12),
        .LATENCY(12)
    ) u_systolic (
        .clk        ( clk           ),
        .rst_n      ( rst_n         ),
        .a_row      ( K_k_matrix    ),         
        .b_col      ( R_k_matrix    ),         
        .load_en    ( valid_in      ),
        .enb_1      ( 1'b0          ),       
        .enb_2_6    ( 1'b0          ),
        .enb_7_12   ( 1'b1          ),
        .c_out      ( KR_result     )   
    );

    for (genvar i = 0; i < STATE_DIM; i++) begin 
        for (genvar j = 0; j < STATE_DIM; j++) begin 
            if (i < 6) begin
                assign KkTmatrix_systolic[i][j] = KkTmatrix[i][j]; // K矩阵赋值
            end else begin
                assign KkTmatrix_systolic[i][j] = 64'h0; // 非K矩阵元素赋值为0
            end
        end
    end
    // (K*R)*K^T计算单元（对应图示路径②）
    SystolicArray #(
        .DWIDTH(64),
        .N(12),
        .LATENCY(12)
    ) u_systolic_1 (
        .clk        (   clk                 ),
        .rst_n      (   rst_n               ),
        .a_row      (   KR_result           ),  
        .b_col      (   KkTmatrix_systolic  ), 
        .load_en    (   kr_done             ),
        .enb_1      (   1'b0                ),
        .enb_2_6    (   1'b0                ),
        .enb_7_12   (   1'b1                ),
        .c_out      (   KRKt                )
    );

    

    MatrixTransBridge #(
        .ROWS(12),        
        .COLS(12),        
        .DATA_WIDTH(DWIDTH)
    ) u_MatrixBridge_2 (
        .clk        ( clk          ),             
        .rst_n      ( rst_n        ),        
        .mat_in     ( IKkH         ), 
        .mat_org    ( IKHmatrix    ),  
        .mat_trans  ( IKHT         ), 
        .valid_out  ( process_done ) 
    );
    
    SystolicArray #(
        .DWIDTH(64),
        .N(12),
        .LATENCY(12)
    ) u_systolic_2 (
        .clk        (   clk         ),
        .rst_n      (   rst_n       ),
        .a_row      (   IKHmatrix   ), // 硬件生成单位矩阵
        .b_col      (   P_kk1       ),
        .load_en    (   ikh_done    ),
        .enb_1      (   1'b1        ),
        .enb_2_6    (   1'b1        ),      // 激活中间区域
        .enb_7_12   (   1'b1        ),
        .c_out      (   IKHP        )
    );

    // 
    SystolicArray #(
        .DWIDTH(64),
        .N(12),
        .LATENCY(12)
    ) u_systolic_3 (
        .clk        (   clk         ),
        .rst_n      (   rst_n       ),
        .a_row      (   IKHP        ), // 硬件生成单位矩阵
        .b_col      (   IKHT        ),
        .load_en    (   ikh_done    ),
        .enb_1      (   1'b0        ),
        .enb_2_6    (   1'b1        ),      // 激活中间区域
        .enb_7_12   (   1'b0        ),
        .c_out      (   IKHPIKHT    )
    );

    for (genvar i = 0; i < STATE_DIM; i++) begin 
        for (genvar j = 0; j < STATE_DIM; j++) begin 
            assign P_kk[i][j] = KRKt[i][j]+IKHPIKHT[i][j]; // K矩阵赋值
        end
    end

endgenerate



endmodule