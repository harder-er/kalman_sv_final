`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/05/21 15:33:30
// Design Name: 
// Module Name: AXI_Matrix_Loader
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


module AXI_Matrix_Loader #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 64
)(
    // AXI4-Lite接口
    input  logic                     S_AXI_ACLK,
    input  logic                     S_AXI_ARESETN,
    input  logic [ADDR_WIDTH-1:0]    S_AXI_AWADDR,
    input  logic                     S_AXI_AWVALID,
    output logic                     S_AXI_AWREADY,
    input  logic [DATA_WIDTH-1:0]    S_AXI_WDATA,
    input  logic [(DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  logic                     S_AXI_WVALID,
    output logic                     S_AXI_WREADY,
    output logic [1:0]               S_AXI_BRESP,
    output logic                     S_AXI_BVALID,
    input  logic                     S_AXI_BREADY,

    // 矩阵存储接口
    output logic [DATA_WIDTH-1:0]    matrix_data,
    output logic [9:0]               matrix_row,
    output logic [9:0]               matrix_col,
    output logic                     matrix_wr_en,
    output logic [1:0]               matrix_sel  // 0:Q_k, 1:R_k, 2:P_00
);

    // AXI状态机
    enum logic [2:0] {IDLE, WRITE_ADDR, WRITE_DATA, WRITE_RESP} axi_state;

    // 地址解码
    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_state <= IDLE;
            matrix_wr_en <= 0;
            S_AXI_AWREADY <= 0;
            S_AXI_WREADY <= 0;
            S_AXI_BVALID <= 0;
        end else begin
            case (axi_state)
                IDLE: begin
                    S_AXI_AWREADY <= 1;
                    if (S_AXI_AWVALID) begin
                        // 解析矩阵选择与坐标
                        matrix_sel <= S_AXI_AWADDR[ADDR_WIDTH-1:ADDR_WIDTH-2];
                        matrix_row <= S_AXI_AWADDR[9:0] / 12; // Q_k行号
                        matrix_col <= S_AXI_AWADDR[9:0] % 12; // Q_k列号
                        axi_state <= WRITE_ADDR;
                    end
                end

                WRITE_ADDR: begin
                    S_AXI_AWREADY <= 0;
                    S_AXI_WREADY <= 1;
                    axi_state <= WRITE_DATA;
                end

                WRITE_DATA: begin
                    if (S_AXI_WVALID) begin
                        matrix_data <= S_AXI_WDATA;
                        matrix_wr_en <= 1;
                        S_AXI_WREADY <= 0;
                        axi_state <= WRITE_RESP;
                    end
                end

                WRITE_RESP: begin
                    matrix_wr_en <= 0;
                    S_AXI_BVALID <= 1;
                    S_AXI_BRESP <= 2'b00; // OKAY
                    if (S_AXI_BREADY) begin
                        S_AXI_BVALID <= 0;
                        axi_state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule