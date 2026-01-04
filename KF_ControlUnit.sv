`timescale 1ns / 1ps

module KF_ControlUnit (
    input  logic         clk,
    input  logic         rst_n,          // 异步复位（低有效）
    input  logic         start,

    // 条件输入
    input  logic         Init_Valid,
    input  logic         SP_Done,
    input  logic         SCU_Done_s,
    input  logic         SCU_Done_p,
    input  logic         CKG_Done,
    input  logic         SCO_Valid,
    input  logic         MDI_Valid,
    input  logic         End_valid,

    // 控制输出
    output logic         en_init,
    output logic         en_sp,
    output logic         en_ckg,
    output logic         en_scu,
    output logic         en_sco,
    output logic         finish
);

    //============================================================
    // start latch（start 可能是脉冲）
    //============================================================
    logic start_d, start_seen;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_d    <= 1'b0;
            start_seen <= 1'b0;
        end else begin
            start_d <= start;
            if (start && !start_d)
                start_seen <= 1'b1;
            if (finish)
                start_seen <= 1'b0;
        end
    end

    //============================================================
    // 状态定义（按图）
    // Init -> State_Prediction -> Cal_Kalman_Gain -> Measure_Data_Input
    // Measure_Data_Input -> State_Covariance_Update -> State_Covariance_Output -> State_Prediction
    // State_Prediction -> End (End_valid)
    //============================================================
    typedef enum logic [2:0] {
        S_INIT = 3'd0,
        S_SP   = 3'd1,
        S_CKG  = 3'd2,
        S_MDI  = 3'd3,
        S_SCU  = 3'd4,
        S_SCO  = 3'd5,
        S_END  = 3'd6
    } state_t;

    state_t current_state, next_state;

    logic scu_done_all;
    assign scu_done_all = SCU_Done_s & SCU_Done_p;

    //============================================================
    // 状态寄存器
    //============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= S_INIT;
        else
            current_state <= next_state;
    end

    //============================================================
    // next_state
    //============================================================
    always_comb begin
        next_state = current_state;

        unique case (current_state)

            S_INIT: begin
                if (!start_seen)
                    next_state = S_INIT;
                else if (Init_Valid)
                    next_state = S_SP;
                else
                    next_state = S_INIT;
            end

            S_SP: begin
                if (End_valid)
                    next_state = S_END;
                else if (SP_Done)
                    next_state = S_CKG;
                else
                    next_state = S_SP;
            end

            S_CKG: begin
                if (CKG_Done)
                    next_state = S_MDI;
                else
                    next_state = S_CKG;
            end

            S_MDI: begin
                if (MDI_Valid)
                    next_state = S_SCU;
                else
                    next_state = S_MDI;
            end

            S_SCU: begin
                // 真实情况下可用 SCO_Valid 做 SCU->SCO 条件；
                // 但你现在 SCO_Valid 仅 debug，因此这里允许 “SCO_Valid 或 SCU_DONE” 进入输出态
                if (SCO_Valid || scu_done_all)
                    next_state = S_SCO;
                else
                    next_state = S_SCU;
            end

            S_SCO: begin
                // 图上是 SCU_Done 返回预测；这里用 scu_done_all（一般在进入/保持时都已满足）
                if (scu_done_all)
                    next_state = S_SP;
                else
                    next_state = S_SCO;
            end

            S_END: begin
                if (start && !start_d)
                    next_state = S_INIT;
                else
                    next_state = S_END;
            end

            default: next_state = S_INIT;
        endcase
    end

    //============================================================
    // Moore 输出
    //============================================================
    always_comb begin
        en_init = 1'b0;
        en_sp   = 1'b0;
        en_ckg  = 1'b0;
        en_scu  = 1'b0;
        en_sco  = 1'b0;
        finish  = 1'b0;

        unique case (current_state)
            S_INIT: en_init = start_seen;
            S_SP:   en_sp   = 1'b1;
            S_CKG:  en_ckg  = 1'b1;
            S_MDI:  ; // 测量输入态一般不给其它模块使能
            S_SCU:  en_scu  = 1'b1;
            S_SCO:  en_sco  = 1'b1;
            S_END:  finish  = 1'b1;
            default: ;
        endcase
    end

endmodule
