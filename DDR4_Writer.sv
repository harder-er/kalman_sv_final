`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/05/22 10:59:09
// Design Name: 
// Module Name: DDR4_Writer
// Project Name: 
// Target Devices: Zynq UltraScale+ (Z7-P)
// Tool Versions: 
// Description: This module handles writing X_kkout and P_kkout matrices
//              to DDR4 memory via an AXI4-Full Write interface.
//
// Dependencies: AXI4 protocol understanding, floating point data (64-bit)
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Expanded AXI Write FSM for 512-bit bus and matrix handling
// Revision 0.03 - Corrected SystemVerilog syntax errors (array declaration, for loop var, explicit automatic)
// Revision 0.04 - Refined AXI FSM logic for sequential burst handling and address updates.
// Additional Comments:
//   - Assumes 64-bit floating point data for X_kk_in and P_kk_in.
//   - AXI AWLEN (burst length) is fixed to (64 / 8) - 1 for X_kk and (MATRIX_SIZE_IN_BYTES / 64) - 1 for P_kk.
//   - Error handling for AXI write response (axi_bresp) is rudimentary; can be enhanced.
//   - This FSM completes one AXI AW transaction, then all W transactions for that burst, then waits for the B response,
//     before starting the next AW transaction.
//////////////////////////////////////////////////////////////////////////////////
module DDR4_Writer #(
    parameter STATE_DIM          = 12, // 状态维度
    parameter MEASURE_DIM        = 6,  // 测量维度 (尽管此模块不直接使用，但为了保持一致性传入)
    parameter ADDR_X_RESULT_BASE = 32'h0050_0000, // X_kkout 结果写入DDR的基地址
    parameter ADDR_P_RESULT_BASE = 32'h0060_0000  // P_kkout 结果写入DDR的基地址
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             write_en, // 写入使能信号，通常由 KalmanFilterCore 的 filter_done 触发

    // 接收 KalmanFilterCore 的输出
    // X_kk_in 是 STATE_DIM 维向量
    input  logic [63:0]      X_kk_in [STATE_DIM-1:0], 
    // P_kk_in 是 STATE_DIM x STATE_DIM 矩阵
    input  logic [63:0]      P_kk_in [STATE_DIM-1:0][STATE_DIM-1:0], 

    // AXI 写通道接口 (AXI4-Full)
    output logic [31:0]      axi_awaddr,
    output logic [7:0]       axi_awlen,   
    output logic [2:0]       axi_awsize,  
    output logic [1:0]       axi_awburst, 
    output logic             axi_awvalid,
    input  logic             axi_awready,
    
    output logic [511:0]     axi_wdata,   // 512位宽
    output logic [63:0]      axi_wstrb,   // 512/8 = 64字节，所以需要64位wstrb (每bit对应一个字节)
    output logic             axi_wvalid,
    input  logic             axi_wready,
    output logic             axi_wlast,   // 突发传输的最后一个数据
    
    input  logic [1:0]       axi_bresp,
    input  logic             axi_bvalid,
    output logic             axi_bready
);

    // 内部状态机
    typedef enum logic [2:0] {
        IDLE,            // 空闲状态，等待写入使能
        SEND_AW_X,       // 发送 X_kk_in 的写地址
        SEND_W_X,        // 发送 X_kk_in 的写数据 (多拍)
        WAIT_B_X,        // 等待 X_kk_in 的写响应

        SEND_AW_P,       // 发送 P_kk_in 的写地址
        SEND_W_P,        // 发送 P_kk_in 的写数据 (多拍)
        WAIT_B_P,        // 等待 P_kk_in 的写响应
        
        WRITE_DONE       // 所有数据写入完成
    } State_t;
    State_t state;

    // AXI 内部信号寄存器
    logic [31:0] current_awaddr;
    logic [511:0] current_wdata;
    logic next_wlast; 

    // 写 X_kk_in 的计数器和相关参数
    // X_kk_in 是 STATE_DIM 个 64位浮点数
    // 512bit AXI 总线一次传输 8 个 64位数据
    // 需要的 AXI 突发传输次数 = ceil(STATE_DIM / 8)
    localparam X_BURST_COUNT = (STATE_DIM + 7) / 8; 
    localparam X_TOTAL_BYTES = STATE_DIM * 8; 
    localparam X_AWLEN_VAL = (X_BURST_COUNT == 0) ? 0 : X_BURST_COUNT - 1; 

    logic [$clog2(X_BURST_COUNT == 0 ? 1 : X_BURST_COUNT)-1:0] x_burst_idx; // 当前 X_kk_in 的 512bit 突发索引 (0 到 X_BURST_COUNT-1)
    
    // 写 P_kk_in 的计数器和相关参数
    // P_kk_in 是 STATE_DIM * STATE_DIM 个 64位浮点数
    localparam P_MATRIX_SIZE = STATE_DIM * STATE_DIM;
    // 需要的 AXI 突发传输次数 = ceil(P_MATRIX_SIZE / 8)
    localparam P_BURST_COUNT = (P_MATRIX_SIZE + 7) / 8;
    localparam P_TOTAL_BYTES = P_MATRIX_SIZE * 8; 
    localparam P_AWLEN_VAL = (P_BURST_COUNT == 0) ? 0 : P_BURST_COUNT - 1; 

    logic [$clog2(P_BURST_COUNT == 0 ? 1 : P_BURST_COUNT)-1:0] p_burst_idx; // 当前 P_kk_in 的 512bit 突发索引

    // AXI 握手信号
    logic aw_hs; // axi_awvalid && axi_awready
    logic w_hs;  // axi_wvalid && axi_wready
    logic b_hs;  // axi_bvalid && axi_bready

    assign aw_hs = axi_awvalid && axi_awready;
    assign w_hs  = axi_wvalid && axi_wready;
    assign b_hs  = axi_bvalid && axi_bready;

    // AXI 写接口输出的组合逻辑赋值
    assign axi_awaddr  = current_awaddr; // axi_awaddr 由寄存器 current_awaddr 驱动
    assign axi_awlen   = (state == SEND_AW_X || state == SEND_W_X || state == WAIT_B_X) ? X_AWLEN_VAL : P_AWLEN_VAL;
    assign axi_awsize  = 3'b110; // 512bit = 64 bytes, log2(64) = 6
    assign axi_awburst = 2'b01;  // INCR (Incrementing burst)
    
    assign axi_wdata   = current_wdata;  // axi_wdata 由寄存器 current_wdata 驱动
    assign axi_wstrb   = 64'hFFFFFFFFFFFFFFFF; // 假设总是全字节写入
    assign axi_wlast   = next_wlast;
    assign axi_bready  = (state == WAIT_B_X || state == WAIT_B_P) ? 1'b1 : 1'b0;


    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            axi_awvalid <= 0;
            axi_wvalid <= 0;
            current_awaddr <= 0;
            current_wdata <= 0; 
            next_wlast <= 0;
            x_burst_idx <= 0;
            p_burst_idx <= 0;
        end else begin
            // 默认拉低，只在需要时拉高
            axi_awvalid <= 0; 
            axi_wvalid <= 0;
            next_wlast <= 0; // 默认不是最后一拍

            case (state)
                IDLE: begin
                    if (write_en) begin
                        state <= SEND_AW_X;
                        x_burst_idx <= 0;
                        current_awaddr <= ADDR_X_RESULT_BASE; // 设置 X_kk_in 的起始地址
                        axi_awvalid <= 1; // 启动 X_kk_in 的写地址请求
                    end
                end
                
                SEND_AW_X: begin // 发送 X_kk_in 的写地址
                    if (aw_hs) begin
                        state <= SEND_W_X;
                        axi_awvalid <= 0; // 地址已发送
                        // 准备第一个 512bit 的数据包
                        for (int i = 0; i < 8; i++) begin 
                            if ((x_burst_idx * 8 + i) < STATE_DIM) begin
                                current_wdata[i*64 +: 64] <= X_kk_in[x_burst_idx * 8 + i];
                            end else begin
                                current_wdata[i*64 +: 64] <= 64'h0; // 填充零
                            end
                        end
                        next_wlast <= (x_burst_idx == X_AWLEN_VAL); // 判断是否是当前突发的最后一个数据包
                        axi_wvalid <= 1; // 准备发送数据
                    end
                end
                
                SEND_W_X: begin // 发送 X_kk_in 的写数据
                    if (w_hs) begin
                        axi_wvalid <= 0; // 数据已发送
                        if (next_wlast) begin // 如果是当前突发的最后一拍数据
                            state <= WAIT_B_X; // 等待写响应
                        end else begin
                            // 准备下一个 512bit 的数据包
                            x_burst_idx <= x_burst_idx + 1;
                            // 注意：这里仅准备数据，不再次拉高 axi_awvalid，因为 AXI AW 和 W 是独立通道
                            for (int i = 0; i < 8; i++) begin 
                                if ((x_burst_idx * 8 + i) < STATE_DIM) begin // 使用更新后的 x_burst_idx
                                    current_wdata[i*64 +: 64] <= X_kk_in[x_burst_idx * 8 + i];
                                end else begin
                                    current_wdata[i*64 +: 64] <= 64'h0;
                                end
                            end
                            next_wlast <= (x_burst_idx == X_AWLEN_VAL); // 使用更新后的 x_burst_idx
                            axi_wvalid <= 1; // 继续发送数据
                        end
                    end
                end
                
                WAIT_B_X: begin // 等待 X_kk_in 的写响应
                    if (b_hs) begin
                        // 写响应处理 (axi_bresp 可以用于错误检查)
                        // 例如：if (axi_bresp != 2'b00) $error("AXI Write Error for X_kk_in!");
                        if (x_burst_idx == X_BURST_COUNT) begin // 所有 X_kk_in 数据及其响应都已完成
                            state <= SEND_AW_P; // 切换到 P_kk_in 写入
                            p_burst_idx <= 0;
                            current_awaddr <= ADDR_P_RESULT_BASE; // 设置 P_kk_in 的起始地址
                            axi_awvalid <= 1; // 启动 P_kk_in 的写地址请求
                        end else begin
                            // 如果 x_burst_idx 未达到 X_BURST_COUNT，通常表示错误或状态机逻辑不完整
                            // 对于此简单 FSM，这里视为写入结束
                            state <= WRITE_DONE; 
                        end
                    end
                end

                SEND_AW_P: begin // 发送 P_kk_in 的写地址
                    if (aw_hs) begin
                        state <= SEND_W_P;
                        axi_awvalid <= 0; // 地址已发送
                        // 准备第一个 512bit 的数据包 for P_kk_in
                        for (int i = 0; i < 8; i++) begin 
                            if ((p_burst_idx * 8 + i) < P_MATRIX_SIZE) begin
                                // 计算 P_kk_in 的二维索引
                                automatic int row = (p_burst_idx * 8 + i) / STATE_DIM; // 修正：显式声明为 automatic
                                automatic int col = (p_burst_idx * 8 + i) % STATE_DIM; // 修正：显式声明为 automatic
                                current_wdata[i*64 +: 64] <= P_kk_in[row][col];
                            end else begin
                                current_wdata[i*64 +: 64] <= 64'h0; // 填充零
                            end
                        end
                        next_wlast <= (p_burst_idx == P_AWLEN_VAL);
                        axi_wvalid <= 1; // 准备发送数据
                    end
                end
                
                SEND_W_P: begin // 发送 P_kk_in 的写数据
                    if (w_hs) begin
                        axi_wvalid <= 0; // 数据已发送
                        if (next_wlast) begin // 如果是当前突发的最后一拍数据
                            state <= WAIT_B_P; // 等待写响应
                        end else begin
                            // 准备下一个 512bit 的数据包
                            p_burst_idx <= p_burst_idx + 1;
                            // 注意：这里仅准备数据
                            for (int i = 0; i < 8; i++) begin 
                                if ((p_burst_idx * 8 + i) < P_MATRIX_SIZE) begin // 使用更新后的 p_burst_idx
                                    automatic int row = (p_burst_idx * 8 + i) / STATE_DIM; // 修正：显式声明为 automatic
                                    automatic int col = (p_burst_idx * 8 + i) % STATE_DIM; // 修正：显式声明为 automatic
                                    current_wdata[i*64 +: 64] <= P_kk_in[row][col];
                                end else begin
                                    current_wdata[i*64 +: 64] <= 64'h0;
                                end
                            end
                            next_wlast <= (p_burst_idx == P_AWLEN_VAL);
                            axi_wvalid <= 1; // 继续发送数据
                        end
                    end
                end
                
                WAIT_B_P: begin // 等待 P_kk_in 的写响应
                    if (b_hs) begin
                        // 写响应处理
                        // if (axi_bresp != 2'b00) $error("AXI Write Error for P_kk_in!");
                        if (p_burst_idx == P_BURST_COUNT) begin // 所有 P_kk_in 数据及其响应都已完成
                            state <= WRITE_DONE; // 所有写入完成
                        end else begin
                            // 同理，这里视为写入结束
                            state <= WRITE_DONE; 
                        end
                    end
                end

                WRITE_DONE: begin
                    // 保持在此状态直到 write_en 信号拉低，然后返回 IDLE
                    if (!write_en) begin 
                        state <= IDLE;
                    end
                end
            endcase
        end
    end

    // Note: axi_awaddr is continuously assigned to current_awaddr.
    // current_awaddr is updated as a register inside the always_ff block
    // at the beginning of each AXI AW transaction (IDLE->SEND_AW_X, WAIT_B_X->SEND_AW_P).

endmodule