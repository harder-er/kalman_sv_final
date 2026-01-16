`timescale 1ns / 1ps

module CEU_division #(
    parameter int DBL_WIDTH = 64
)(
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   valid,
    output logic                   finish,
    input  logic [DBL_WIDTH-1:0]   numerator,
    input  logic [DBL_WIDTH-1:0]   denominator,
    output logic [DBL_WIDTH-1:0]   quotient
);

    // ===================================================================
    // 正确的AXI-Stream握手实现（参考PG060协议）
    // 关键：
    // 1. 输入侧保留pending标志，将单拍valid变成持续tvalid
    // 2. 输出侧用res_tvalid & res_tready确定finish时刻
    // ===================================================================

    logic req_pending;
    logic [DBL_WIDTH-1:0] num_r;
    logic [DBL_WIDTH-1:0] den_r;

    logic s_axis_a_tready, s_axis_b_tready;
    logic m_axis_result_tvalid, m_axis_result_tdata_w;
    logic [DBL_WIDTH-1:0] m_axis_result_tdata;

    // 接收输入的握手信号
    wire accept_in = req_pending && s_axis_a_tready && s_axis_b_tready;

    // ===================================================================
    // 输入侧FSM：把单拍valid变成持续tvalid，直到IP接走
    // ===================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_pending <= 1'b0;
            num_r       <= '0;
            den_r       <= '0;
        end else begin
            // 从上游接收一次请求（允许valid是单拍脉冲）
            if (valid && ~req_pending) begin
                req_pending <= 1'b1;
                num_r       <= numerator;
                den_r       <= denominator;
            end

            // 当IP真正接走了输入（握手成功），清pending
            if (accept_in) begin
                req_pending <= 1'b0;
            end
        end
    end

    // 驱动IP的tvalid：只要pending=1，就保持tvalid=1，直到tready接走
    wire s_axis_a_tvalid = req_pending;
    wire s_axis_b_tvalid = req_pending;

    // ===================================================================
    // 实例化：Floating-Point Divider（Double）
    // ===================================================================
    floating_point_div u_floating_point_div (
        .aclk                 ( clk                    ),
        .aresetn              ( rst_n                  ),

        .s_axis_a_tvalid      ( s_axis_a_tvalid       ),
        .s_axis_a_tready      ( s_axis_a_tready       ),
        .s_axis_a_tdata       ( num_r                  ),

        .s_axis_b_tvalid      ( s_axis_b_tvalid       ),
        .s_axis_b_tready      ( s_axis_b_tready       ),
        .s_axis_b_tdata       ( den_r                  ),

        .m_axis_result_tvalid ( m_axis_result_tvalid  ),
        .m_axis_result_tready ( 1'b1                  ),
        .m_axis_result_tdata  ( m_axis_result_tdata   )
    );

    // ===================================================================
    // 输出侧：直接锁存IP的结果，finish在输出被消费时产生
    // ===================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quotient <= '0;
        end else begin
            // 当IP输出有效且被我们接走时，锁存结果
            if (m_axis_result_tvalid && 1'b1) begin  // 1'b1 is m_axis_result_tready
                quotient <= m_axis_result_tdata;
            end
        end
    end

    // finish = 输出被消费的那一拍（m_axis_result_tvalid & m_axis_result_tready）
    // 不要用"输入侧tready"去门控finish（那样是错的）
    assign finish = m_axis_result_tvalid && 1'b1;  // 1'b1 is m_axis_result_tready

endmodule
