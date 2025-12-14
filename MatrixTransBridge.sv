`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/25 21:14:58
// Design Name: 
// Module Name: MatrixTransBridge
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

module MatrixTransBridge #(
    parameter ROWS = 12,        // 矩阵行数
    parameter COLS = 12,        // 矩阵列数
    parameter DATA_WIDTH = 64  // 数据位宽（网页6中8x8方案扩展）
)(
    input  logic                     clk,       // 系统时钟（网页8时序控制）
    input  logic                     rst_n,     // 异步复位
    input  logic [DATA_WIDTH-1:0]    mat_in [0:ROWS-1][0:COLS-1], // 输入矩阵
    output logic [DATA_WIDTH-1:0]    mat_org [0:ROWS-1][0:COLS-1],// 原矩阵输出
    output logic [DATA_WIDTH-1:0]    mat_trans [0:COLS-1][0:ROWS-1],// 转置矩阵
    output logic                     valid_out  // 输出有效标志
);

// ████ 输入寄存器组（网页7移位寄存器方案改进）
logic [DATA_WIDTH-1:0] input_buffer [0:ROWS-1][0:COLS-1];

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        foreach(input_buffer[i,j]) 
            input_buffer[i][j] <= '0;
    end else begin
        input_buffer <= mat_in;  // 同步锁存输入（网页8控制逻辑）
    end
end

// ████ 转置生成逻辑（网页6循环方案优化）
generate
    for(genvar i=0; i<ROWS; i++) begin : row_gen
        for(genvar j=0; j<COLS; j++) begin : col_gen
            // 原矩阵直通输出（网页1矩阵操作原理）
            assign mat_org[i][j] = input_buffer[i][j];
            
            // 转置矩阵生成（网页6转置逻辑核心）
            assign mat_trans[j][i] = input_buffer[i][j]; 
        end
    end
endgenerate

// ████ 时序控制单元（网页8状态机改进）
typedef enum {IDLE, PROCESS} state_t;
state_t curr_state;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        curr_state <= IDLE;
        valid_out <= 1'b0;
    end else begin
        case(curr_state)
            IDLE: begin
                valid_out <= 1'b0;
                if(&mat_in[ROWS-1][COLS-1]) // 检测输入完成（网页8配置寄存器思想）
                    curr_state <= PROCESS;
            end
            PROCESS: begin
                valid_out <= 1'b1;          // 输出有效信号
                curr_state <= IDLE;
            end
        endcase
    end
end

endmodule
