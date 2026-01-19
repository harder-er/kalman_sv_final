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
    
    // 信号锁存：在相应状态中保持各个完成信号为高
    logic init_valid_locked;
    logic sp_done_locked;
    logic ckg_done_locked;
    logic mdi_valid_locked;
    logic scu_done_s_locked;
    logic scu_done_p_locked;
    logic sco_valid_locked;
    logic end_valid_locked;
    
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
            // 复位时清除所有锁存信号
            init_valid_locked   <= 1'b0;
            sp_done_locked      <= 1'b0;
            ckg_done_locked     <= 1'b0;
            mdi_valid_locked    <= 1'b0;
            scu_done_s_locked   <= 1'b0;
            scu_done_p_locked   <= 1'b0;
            sco_valid_locked    <= 1'b0;
            end_valid_locked    <= 1'b0;
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

            // ===================================================================
            // 信号锁存逻辑：在各状态中锁存对应的完成/有效信号
            // ===================================================================
            
            // 回到IDLE时，清除所有锁存信号
            if (next_state == S_IDLE) begin
                init_valid_locked   <= 1'b0;
                sp_done_locked      <= 1'b0;
                ckg_done_locked     <= 1'b0;
                mdi_valid_locked    <= 1'b0;
                scu_done_s_locked   <= 1'b0;
                scu_done_p_locked   <= 1'b0;
                sco_valid_locked    <= 1'b0;
                end_valid_locked    <= 1'b0;
            end else begin
                // S_INIT 状态：锁存 Init_Valid
                if (current_state == S_INIT) begin
                    if (Init_Valid)
                        init_valid_locked <= 1'b1;
                end else if (next_state != S_INIT)
                    init_valid_locked <= 1'b0;

                // S_SP 状态：锁存 SP_Done
                if (current_state == S_SP) begin
                    if (SP_Done)
                        sp_done_locked <= 1'b1;
                end else if (next_state != S_SP)
                    sp_done_locked <= 1'b0;

                // S_CKG 状态：锁存 CKG_Done
                if (current_state == S_CKG) begin
                    if (CKG_Done)
                        ckg_done_locked <= 1'b1;
                end else if (next_state != S_CKG)
                    ckg_done_locked <= 1'b0;

                // S_MDI 状态：锁存 MDI_Valid
                if (current_state == S_MDI) begin
                    if (MDI_Valid)
                        mdi_valid_locked <= 1'b1;
                end else if (next_state != S_MDI)
                    mdi_valid_locked <= 1'b0;

                // S_SCU 状态：锁存 SCU_Done_s 和 SCU_Done_p
                // 关键：一旦进入 SCU 状态，就开始锁存这两个信号
                // 它们可能在不同周期到达，所以用 OR 逻辑累积
                if (current_state == S_SCU) begin
                    if (SCU_Done_s)
                        scu_done_s_locked <= 1'b1;
                    if (SCU_Done_p)
                        scu_done_p_locked <= 1'b1;
                end else if (next_state != S_SCU) begin
                    // 只有当离开 SCU 状态时才清除
                    scu_done_s_locked <= 1'b0;
                    scu_done_p_locked <= 1'b0;
                end

                // S_SCO 状态：锁存 SCO_Valid 和 End_valid
                if (current_state == S_SCO) begin
                    if (SCO_Valid)
                        sco_valid_locked <= 1'b1;
                    if (End_valid)
                        end_valid_locked <= 1'b1;
                end else if (next_state != S_SCO) begin
                    sco_valid_locked <= 1'b0;
                    end_valid_locked <= 1'b0;
                end
            end
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
    assign scu_done_all = scu_done_s_locked & scu_done_p_locked;

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

            // ★S_INIT: 初始化状态，等待 Init_Valid（锁存版本）
            S_INIT: begin
                if (init_valid_locked)
                    next_state = S_SP;
                else
                    next_state = S_INIT;
            end

            // ★S_SP: State Prediction，等待 SP_Done（锁存版本）
            S_SP: begin
                if (sp_done_locked)
                    next_state = S_CKG;
                else
                    next_state = S_SP;
            end

            // ★S_CKG: Kalman Gain Calculate，等待 CKG_Done（锁存版本）
            S_CKG: begin
                if (ckg_done_locked)
                    next_state = S_MDI;       // 进入 Measure Data Input
                else
                    next_state = S_CKG;
            end

            // ★S_MDI: Measure Data Input，等待 MDI_Valid（锁存版本）
            S_MDI: begin
                if (mdi_valid_locked)
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

            // ★S_SCO: State Covariance Output，等待 SCO_Valid/End_valid（锁存版本）
            S_SCO: begin
                if (sco_valid_locked)
                    next_state = S_SP;        // 输出完成，回到 State Prediction（迭代）
                else if (end_valid_locked)
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
