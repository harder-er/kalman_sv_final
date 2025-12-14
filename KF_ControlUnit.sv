`timescale 1ns / 1ps

module KF_ControlUnit (
    input  logic         clk,
    input  logic         rst_n,          // 异步复位（低有效）
    // 状态条件输入
    input  logic         Init_Valid,     // 初始化验证
    input  logic         SP_Done,        // 状态预测完成
    input  logic         SCU_Done_s,       // 状态更新完成
    input  logic         SCU_Done_p,       // 协方差更新完成
    input  logic         CKG_Done,       // 卡尔曼增益计算完成
    input  logic         SCO_Valid,      // 协方差输出有效
    input  logic         MDI_Valid,      // 测量数据有效
    input  logic         End_valid,      // 结束验证
    // 控制信号输出
    output logic         en_init,        // 初始化使能
    output logic         en_sp,         // 状态预测使能
    output logic         en_ckg,        // 卡尔曼增益计算使能
    output logic         en_scu,        // 协方差更新使能
    output logic         en_sco,        // 协方差状态输出使能
    output logic         finish
);

    // 状态编码（严格对应图片流程）
    typedef enum logic [2:0] {
        INIT                = 3'b000,
        STATE_PREDICTION    = 3'b001,
        CAL_KALMAN_GAIN     = 3'b010,
        STATE_COV_UPDATE    = 3'b011,
        END_STATE           = 3'b100
    } state_t;

    // 双寄存器消除亚稳态
    reg  rst_sync;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 1'b0;
        else rst_sync <= 1'b1;
    end

    // 时序逻辑状态寄存器
    (* syn_encoding = "one-hot" *) 
    state_t current_state, next_state;

    always_ff @(posedge clk or negedge rst_sync) begin
        if (!rst_sync) begin
            current_state <= INIT;  // 同步复位
        end else begin
            current_state <= next_state;  // 状态转移
        end
    end

  
    always_ff @(posedge clk or negedge rst_sync) begin
        if (!rst_sync) begin
            next_state <= INIT;  // 异步复位到初始状态
        end else begin
            case (current_state) 
                // 初始化状态转移路径
                INIT: begin
                    if (Init_Valid) begin
                        next_state <= STATE_PREDICTION;
                    end else begin
                        next_state <= INIT;  // 循环状态
                    end
                end
                
                
                // 状态预测阶段转移逻辑
                STATE_PREDICTION: begin
                    if (End_valid) 
                        next_state <= END_STATE;
                    else if (SP_Done) begin
                        next_state <= CAL_KALMAN_GAIN;  
                    end else 
                        next_state <= STATE_PREDICTION;  // 循环状态
                end
                
                // 卡尔曼增益计算阶段
                CAL_KALMAN_GAIN: begin
                    if (CKG_Done) next_state <= STATE_COV_UPDATE;
                    else next_state <= CAL_KALMAN_GAIN;  // 循环状态
                end 
                
                // 协方差更新阶段
                STATE_COV_UPDATE: begin
                    if (SCU_Done_s && SCU_Done_p && MDI_Valid) begin
                        next_state <= STATE_PREDICTION;
                    end else next_state <= STATE_COV_UPDATE;  // 循环状态
                end
                
                default: next_state <= INIT;  // 容错处理
            endcase
        end
    end


    always_comb begin
        // 默认值
        {en_init, en_sp, en_ckg, en_scu, en_sco, finish} = 6'b0;

        case (current_state)
            INIT:               en_init = 1'b1;
            STATE_PREDICTION:   en_sp   = 1'b1;
            CAL_KALMAN_GAIN:    en_ckg  = 1'b1;
            STATE_COV_UPDATE: begin 
                en_scu = 1'b1;
                en_sco = 1'b1;
            end
            END_STATE:          finish  = 1'b1; // 无控制信号
        endcase
    end


endmodule