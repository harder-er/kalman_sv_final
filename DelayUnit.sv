`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
//
// Create Date: 2025/05/10
// Module Name: DelayUnit
// Description: 参数化矩阵延迟单元，将数据矩阵 data_in 延迟 DELAY_CYCLES 个时钟周期后输出，
//              在此期间保持不变。
// 
// 支持可配置的行数 ROWS、列数 COLS 以及每元素的位宽 DATA_WIDTH。
// 
//////////////////////////////////////////////////////////////////////////////////

module DelayUnit #(
    parameter integer DELAY_CYCLES = 1,
    parameter integer ROWS         = 12,
    parameter integer COLS         = 12,
    parameter integer DATA_WIDTH   = 64
)(
    input  logic                   clk      ,
    input  logic                   rst_n    ,
    input  logic [DATA_WIDTH-1:0]  data_in  [0:ROWS-1][0:COLS-1],
    // input  logic [DATA_WIDTH-1:0]  data_in  [ROWS-1:0] [COLS>1?COLS:1];
    output logic [DATA_WIDTH-1:0]  data_out [0:ROWS-1][0:COLS-1]
);

    // 如果不需要任何延迟，直接连线
    generate
        if (DELAY_CYCLES == 0) begin
            for (genvar r = 0; r < ROWS; r++) begin
                for (genvar c = 0; c < COLS; c++) begin
                    assign data_out[r][c] = data_in[r][c];
                end
            end
        end else begin
            // pipeline registers: stage[0] ... stage[DELAY_CYCLES-1]
            logic [DATA_WIDTH-1:0] stage [0:DELAY_CYCLES-1][0:ROWS-1][0:COLS-1];

            integer i, r, c;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    // 复位：清零所有寄存器
                    for (i = 0; i < DELAY_CYCLES; i = i + 1) begin
                        for (r = 0; r < ROWS; r = r + 1) begin
                            for (c = 0; c < COLS; c = c + 1) begin
                                stage[i][r][c] <= '0;
                            end
                        end
                    end
                end else begin
                    // 第一级接受输入
                    for (r = 0; r < ROWS; r = r + 1) begin
                        for (c = 0; c < COLS; c = c + 1) begin
                            stage[0][r][c] <= data_in[r][c];
                        end
                    end
                    // 其余级联移位
                    for (i = 1; i < DELAY_CYCLES; i = i + 1) begin
                        for (r = 0; r < ROWS; r = r + 1) begin
                            for (c = 0; c < COLS; c = c + 1) begin
                                stage[i][r][c] <= stage[i-1][r][c];
                            end
                        end
                    end
                end
            end

            // 输出最后一级
            for (genvar r = 0; r < ROWS; r = r + 1) begin
                for (genvar c = 0; c < COLS; c = c + 1) begin
                    assign data_out[r][c] = stage[DELAY_CYCLES-1][r][c];
                end
            end
        end
    endgenerate

endmodule
