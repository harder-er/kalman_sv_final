`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/03/13 09:42:35
// Design Name: 
// Module Name: fp_multiplier       // ����˷���ģ�飨ʵ����������?
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: ʵ���������ݵ��������㣬����������ˮ�߼ܹ�
// 
// Dependencies: ��������ƽ�����㵥Ԫfp_square
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments: ��ˮ�߽׶ΰ������ݼĴ桢ƽ�����㡢�����ϳ�
// 
//////////////////////////////////////////////////////////////////////////////////

module fp_multiplier (
    input  logic clk             ,
    input  logic [64-1:0] a, b   ,
    input  logic          valid  ,
    output logic          finish ,
    output logic [64-1:0] result
);
    logic s_axis_a_tready;
    logic s_axis_b_tready;
    logic m_axis_result_tvalid;
    // ʵ����˫���ȳ˷���IP
    floating_point_mul u_floating_point_mul(
        .aclk                   ( clk                   ),             // ʱ��
        // .aresetn                ( rst_n                 ),             // 异步复位
		// A ͨ��   
		.s_axis_a_tvalid        ( valid                 ),        // ���� A ��Ч
		.s_axis_a_tready        ( s_axis_a_tready       ),        // ���� A ����
		.s_axis_a_tdata         ( a                     ),         // ���� A ����
		// B ͨ��   
		.s_axis_b_tvalid        ( valid                 ),        // ���� B ��Ч
		.s_axis_b_tready        ( s_axis_b_tready       ),        // ���� B ����
		.s_axis_b_tdata         ( b                     ),         // ���� B ����
		// ������ͨ��
		.m_axis_result_tvalid   ( m_axis_result_tvalid  ),   // ������?
		.m_axis_result_tready   ( 1'b1                  ),   // �������?
		.m_axis_result_tdata    ( result          		)     // �������?
	);

assign finish = m_axis_result_tvalid;  // �?只看结果有效信号

endmodule
