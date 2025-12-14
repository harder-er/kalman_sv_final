`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/03/10 15:30:48
// Design Name: 
// Module Name: StateUpdate
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

module StateUpdate (
    input  logic         clk                    , 
    input  logic         rst_n                  ,
    
    // 状态转移矩阵输入
    input  logic [63:0]  F      [11:0][11:0]    , // 
    input  logic [63:0]  X_kk   [11:0]          , // 
    // FIFO输出接口
    output logic [63:0]  X_k1k  [11:0]          ,   // X_{k+1,k}输出
    
    input  logic         MDI_Valid              , 
    input  logic         CKG_Done               ,
    output logic         SCU_Done
);
logic [64-1:0] matrix_out [0:12-1][0:12-1];
generate//填充为12x12矩阵
    for (genvar i = 0; i < 12; i++) begin : row_gen
        for (genvar j = 0; j < 12; j++) begin : col_gen  
            assign matrix_out[i][j] = (j == 0) ? X_kk[i] : 64'h0;
        end
    end
endgenerate
logic load_en;
assign load_en = MDI_Valid & CKG_Done;

logic [64-1:0] Xk1kmatrix [11:0][11:0];
SystolicArray #(
    .DWIDTH(64),
    .N(12),
    .LATENCY(12)
) u_systolic (
    .clk        ( clk               ),
    .rst_n      ( rst_n             ),
    .a_row      ( F                 ),   
    .b_col      ( matrix_out        ),      
    .load_en    ( load_en           ), 
    .enb_1      ( 1'b1              ), 
    .enb_2_6    ( 1'b0              ), 
    .enb_7_12   ( 1'b0              ), 
    .c_out      ( Xk1kmatrix        ),
    .cal_finish ( SCU_Done          ) 
);
generate
    for (genvar i = 0; i < 12; i++) begin : gen_Xk1k
        assign X_k1k[i] = Xk1kmatrix[i][0];
    end
endgenerate

endmodule