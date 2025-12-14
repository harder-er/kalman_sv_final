`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/05/22 10:31:56
// Design Name: 
// Module Name: NoiseGenerator
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


module NoiseGenerator #(
    parameter STATE_DIM   = 12,
    parameter MEASURE_DIM = 6,
    parameter deltat      = 0.01, // 从顶层传入 deltat

    // 示例噪声参数 (双精度浮点数的十六进制表示)
    // 您需要根据您的实际噪声模型来定义和传递这些参数
    parameter real NOISE_VAR_POS_REAL = 0.001, // 位置噪声方差
    parameter real NOISE_VAR_VEL_REAL = 0.01,  // 速度噪声方差
    parameter real MEASURE_VAR_REAL   = 0.1   // 测量噪声方差 (假设所有测量维度相同)
    // 如果测量噪声方差不同，需要更多参数：
    // parameter real MEASURE_VAR_X_REAL = 0.1,
    // parameter real MEASURE_VAR_Y_REAL = 0.1,
    // ...
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             enable_gen, // 触发噪声矩阵生成，通常在系统启动时一次性
    
    output logic [63:0] Q_k [STATE_DIM-1:0][STATE_DIM-1:0],
    output logic [63:0] R_k [MEASURE_DIM-1:0][MEASURE_DIM-1:0],
    output logic         matrices_ready // 指示 Q_k 和 R_k 已生成完成
);

    // 将 real 类型的参数转换为 64位浮点数表示
    // Vivado 综合工具通常支持在参数中使用 $realtobits
    localparam [63:0] NOISE_VAR_POS_FP = $realtobits(NOISE_VAR_POS_REAL);
    localparam [63:0] NOISE_VAR_VEL_FP = $realtobits(NOISE_VAR_VEL_REAL);
    localparam [63:0] MEASURE_VAR_FP   = $realtobits(MEASURE_VAR_REAL);
    localparam [63:0] DELTAT_FP        = $realtobits(deltat);
    localparam [63:0] DELTAT_SQ_FP     = $realtobits(deltat * deltat);
    localparam [63:0] DELTAT_CU_FP     = $realtobits(deltat * deltat * deltat);
    localparam [63:0] HALF_FP          = 64'h3FE0000000000000; // 0.5
    localparam [63:0] ONE_SIXTH_FP     = $realtobits(1.0/6.0); // 1/6
    localparam [63:0] ONE_THIRD_FP     = $realtobits(1.0/3.0); // 1/3
    localparam [63:0] ZERO_FP          = 64'h0;
    localparam [63:0] NOISE_VAR_POS_VEL_REAL = 0.001; // 位置-速度交叉项噪声方差

    // 假设您使用浮点运算IP核，或者您自己实现了浮点乘法/加法
    // 这里我们假设存在浮点乘法模块 `fp_mul_64`
    // 实际实现需要浮点运算IP (如Xilinx Floating Point Operator IP) 或自定义浮点运算逻辑

    // 内部状态机，用于控制Q和R的生成
    enum {IDLE, GEN_Q, GEN_R, DONE} state;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            matrices_ready <= 0;
            // 初始化 Q_k 和 R_k 为0
            for (int i=0; i<STATE_DIM; i++) begin
                for (int j=0; j<STATE_DIM; j++) begin
                    Q_k[i][j] <= ZERO_FP;
                end
            end
            for (int i=0; i<MEASURE_DIM; i++) begin
                for (int j=0; j<MEASURE_DIM; j++) begin
                    R_k[i][j] <= ZERO_FP;
                end
            end
        end else begin
            case (state)
                IDLE:
                    if (enable_gen) begin
                        state <= GEN_Q;
                        matrices_ready <= 0;
                    end
                GEN_Q: // 生成 Q_k 矩阵
                    begin
                        // 示例：Q_k 矩阵的噪声模型（高斯白噪声）
                        // 假设状态向量是 [x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz]
                        // 且噪声模型是基于加速度的白噪声，或者更复杂的模型
                        // 这里使用你之前在 `always_comb` 中填充的类似逻辑，但现在是时序逻辑
                        // 如果Q和R只计算一次，可以在enable_gen触发时一次性计算并锁存

                        // 示例：Q_k 对角线元素基于 deltat 和噪声方差
                        // 这需要使用浮点乘法器
                        // Q_k[0][0] = DELTAT_CU_FP * ONE_THIRD_FP * NOISE_VAR_POS_FP;
                        // Q_k[0][3] = DELTAT_SQ_FP * HALF_FP * NOISE_VAR_POS_VEL_CROSS_FP;
                        // ...
                        
                        // 由于是复杂浮点计算，通常需要多个时钟周期
                        // 这是一个简化的示例，假设计算在一个周期内完成（否则需要FSM和浮点IP核）
                        
                        // 假设的Q_k生成逻辑 (伪代码，需要替换为真实的浮点运算)
                        // 通常这种复杂计算会实例化浮点运算IP，并等待其输出
                        // 这里的赋值将在下一个时钟沿发生
                        for (int i = 0; i < STATE_DIM; i ++) begin
                            for (int j = 0; j < STATE_DIM; j ++) begin
                                Q_k[i][j] <= ZERO_FP; // 默认置零

                                // 位置噪声部分 (x,y,z)
                                if (i < 3 && j < 3) begin
                                    if (i == j) begin
                                        Q_k[i][j] <= $realtobits(deltat * deltat * deltat / 3.0 * NOISE_VAR_POS_REAL);
                                    end
                                end
                                // 速度噪声部分 (vx,vy,vz)
                                if (i >= 3 && i < 6 && j >= 3 && j < 6) begin
                                    if (i == j) begin
                                        Q_k[i][j] <= $realtobits(deltat * NOISE_VAR_VEL_REAL); // 假设速度噪声与deltat一次方相关
                                    end
                                end
                                // 示例交叉项
                                if (i < 3 && j >=3 && j < 6 && (j-i == 3) ) begin // 例如Q[0][3], Q[1][4], Q[2][5]
                                     Q_k[i][j] <= $realtobits(deltat * deltat / 2.0 * NOISE_VAR_POS_VEL_REAL); // 位置-速度交叉项
                                end
                                // 还有对称的 Q[3][0] 等等
                                if (i >= 3 && i < 6 && j < 3 && (i-j == 3)) begin // 例如Q[3][0], Q[4][1], Q[5][2]
                                    Q_k[i][j] <= $realtobits(deltat * deltat / 2.0 * NOISE_VAR_POS_VEL_REAL);
                                end
                                
                                // ... 补充您的实际 Q_k 生成公式 ...
                            end
                        end
                        state <= GEN_R; // Q_k 生成完成后进入 R_k 生成
                    end
                GEN_R: // 生成 R_k 矩阵
                    begin
                        // 示例：R_k 矩阵的噪声模型（高斯白噪声）
                        // 假设测量噪声是独立的，R_k是对角矩阵
                        for (int i = 0; i < MEASURE_DIM; i ++) begin
                            for (int j = 0; j < MEASURE_DIM; j ++) begin
                                R_k[i][j] <= ZERO_FP; // 默认置零
                                if (i == j) begin
                                    R_k[i][j] <= MEASURE_VAR_FP; // 每个测量维度独立方差
                                    // 如果测量噪声方差不同，需要更多参数
                                    // if (i == 0) R_k[0][0] <= MEASURE_VAR_X_FP;
                                    // else if (i == 1) R_k[1][1] <= MEASURE_VAR_Y_FP;
                                    // ...
                                end
                            end
                        end
                        state <= DONE; // R_k 生成完成后进入完成状态
                    end
                DONE: begin
                    matrices_ready <= 1; // 噪声矩阵已生成完毕
                    // 如果 enable_gen 再次拉低，可以回到 IDLE
                    if (!enable_gen) state <= IDLE;
                end
            endcase
        end
    end

endmodule