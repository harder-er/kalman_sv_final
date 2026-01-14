`timescale 1ns / 1ps

module KF_ControlUnit (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         start,

    input  logic         Init_Valid,
    input  logic         SP_Done,
    input  logic         SCU_Done_s,
    input  logic         SCU_Done_p,
    input  logic         CKG_Done,
    input  logic         SCO_Valid,
    input  logic         MDI_Valid,
    input  logic         End_valid,

    output logic         en_init,
    output logic         en_sp,
    output logic         en_ckg,
    output logic         en_scu,
    output logic         en_sco,
    output logic         finish
);

    // start latch and state tracking
    logic start_d, start_seen, has_run;
    
    typedef enum logic [2:0] {
        S_IDLE = 3'd0,        // 空闲/复位状态
        S_INIT = 3'd1,        // 初始化状态
        S_SP   = 3'd2,        // State Prediction
        S_CKG  = 3'd3,        // Kalman Gain Calculate
        S_MDI  = 3'd4,        // Measure Data Input
        S_SCU  = 3'd5,        // State Covariance Update
        S_SCO  = 3'd6         // State Covariance Output
    } state_t;

    state_t current_state, next_state;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_d    <= 1'b0;
            start_seen <= 1'b0;
            has_run    <= 1'b0;
        end else begin
            start_d <= start;
            if (start && !start_d)
                start_seen <= 1'b1;
            if (has_run && next_state == S_IDLE && current_state != S_IDLE)
                has_run <= 1'b0;  // clear when exiting back to IDLE
            if (start_seen && current_state != S_IDLE)
                has_run <= 1'b1;  // set when FSM leaves IDLE
            if (finish)
                start_seen <= 1'b0;
        end
    end

    // state reg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= S_IDLE;
        else
            current_state <= next_state;
    end

    logic scu_done_all;
    assign scu_done_all = SCU_Done_s & SCU_Done_p;

    // next_state
    always_comb begin
        next_state = current_state;

        unique case (current_state)
            // ★S_IDLE: 空闲/复位状态，等待 start 信号
            S_IDLE: begin
                if (start_seen)
                    next_state = S_INIT;
                else
                    next_state = S_IDLE;
            end

            // ★S_INIT: 初始化状态，等待 Init_Valid
            S_INIT: begin
                if (Init_Valid)
                    next_state = S_SP;
                else
                    next_state = S_INIT;
            end

            // ★S_SP: State Prediction，等�?SP_Done
            S_SP: begin
                if (SP_Done)
                    next_state = S_CKG;
                else
                    next_state = S_SP;
            end

            // ★S_CKG: Kalman Gain Calculate，等�?CKG_Done
            S_CKG: begin
                if (CKG_Done)
                    next_state = S_MDI;       // 进入 Measure Data Input
                else
                    next_state = S_CKG;
            end

            // ★S_MDI: Measure Data Input，等�?MDI_Valid（即 En_MDI�?            
            S_MDI: begin
                if (MDI_Valid)
                    next_state = S_SCU;
                else
                    next_state = S_MDI;
            end

            // ★S_SCU: State Covariance Update，等�?SCU_Done_all
            S_SCU: begin
                if (scu_done_all)
                    next_state = S_SCO;       // 协方差更新完成，进入输出
                else
                    next_state = S_SCU;
            end

            // ★S_SCO: State Covariance Output，等�?SCO_Done
            S_SCO: begin
                if (SCO_Valid)
                    next_state = S_SP;        // 输出完成，回�?State Prediction（迭代）
                else if (End_valid)
                    next_state = S_IDLE;      // End_valid 返回 IDLE
                else
                    next_state = S_SCO;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // outputs
    always_comb begin
        en_init = 1'b0;
        en_sp   = 1'b0;
        en_ckg  = 1'b0;
        en_scu  = 1'b0;
        en_sco  = 1'b0;
        finish  = 1'b0;

        unique case (current_state)
            S_IDLE: finish  = has_run ? 1'b1 : 1'b0;  // ★Only finish if returning from running
            S_INIT: en_init = 1'b1;
            S_SP:   en_sp   = 1'b1;
            S_CKG:  en_ckg  = 1'b1;
            S_MDI:  /* no enable */;   // MDI 阶段�?MDI_Valid 控制
            S_SCU:  en_scu  = 1'b1;
            S_SCO:  en_sco  = 1'b1;
            default: ;
        endcase
    end

endmodule
