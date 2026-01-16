`timescale 1ns / 1ps

module fp_multiplier #(
    parameter int DW = 64
)(
    input  logic         clk,
    input  logic         rst_n,
    
    // 上游请求（脉冲也可以）
    input  logic         valid,
    output logic         ready,
    input  logic [DW-1:0] a,
    input  logic [DW-1:0] b,

    // 下游结果
    output logic         finish,
    output logic [DW-1:0] result
);

    // ----------------------------
    // 把"单拍 valid"变成 AXIS 持续 tvalid，直到 tready
    // ----------------------------
    logic req_pending;
    logic [DW-1:0] a_r, b_r;

    logic s_axis_a_tready, s_axis_b_tready;
    logic m_axis_result_tvalid;
    logic [DW-1:0] m_axis_result_tdata;

    wire accept_in = req_pending && s_axis_a_tready && s_axis_b_tready;

    // 上游 ready：我们没有 pending 才能接新活
    assign ready = ~req_pending;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_pending <= 1'b0;
            a_r <= '0;
            b_r <= '0;
        end else begin
            // 接收上游一次请求（允许 valid 是单拍）
            if (valid && ready) begin
                req_pending <= 1'b1;
                a_r <= a;
                b_r <= b;
            end

            // 当 IP 真正接走了输入（握手成功），清 pending
            if (accept_in) begin
                req_pending <= 1'b0;
            end
        end
    end

    // 驱动 IP 的 tvalid：pending 就保持 1，直到 tready 接走
    wire s_axis_a_tvalid = req_pending;
    wire s_axis_b_tvalid = req_pending;

    // ----------------------------
    // 实例化：Floating-Point Multiplier（Double）
    // ----------------------------
    floating_point_mul u_floating_point_mul (
        .aclk                 (clk),

        .s_axis_a_tvalid      (s_axis_a_tvalid),
        .s_axis_a_tready      (s_axis_a_tready),
        .s_axis_a_tdata       (a_r),

        .s_axis_b_tvalid      (s_axis_b_tvalid),
        .s_axis_b_tready      (s_axis_b_tready),
        .s_axis_b_tdata       (b_r),

        .m_axis_result_tvalid (m_axis_result_tvalid),
        .m_axis_result_tready (1'b1),
        .m_axis_result_tdata  (m_axis_result_tdata)
    );

    // 输出赋值
    assign result = m_axis_result_tdata;
    
    // finish 脉冲：当结果有效时
    assign finish = m_axis_result_tvalid;

endmodule
