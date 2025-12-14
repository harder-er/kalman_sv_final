`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/25 15:19:38
// Design Name: 
// Module Name: fp_adder
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

module fp_adder(
    input  logic 		  clk       ,
    input  logic [64-1:0] a, b      ,
    input  logic          valid     ,
    output logic          finish    ,
    output logic [64-1:0] result
);
    logic s_axis_a_tready;
    logic s_axis_b_tready;
    logic m_axis_result_tvalid;
    floating_point_add u_floating_point_add (
		.aclk                   ( clk           		),   
		// A 通道   
		.s_axis_a_tvalid        ( valid         		),   
		.s_axis_a_tready        ( s_axis_a_tready      	),   
		.s_axis_a_tdata         ( a             		),   
		// B 通道   
		.s_axis_b_tvalid        ( valid         		),   
		.s_axis_b_tready        ( s_axis_b_tready      	),   
		.s_axis_b_tdata         ( b             		),   
		// 输出结果通道
		.m_axis_result_tvalid   ( m_axis_result_tvalid  ),   
		.m_axis_result_tready   ( 1'b1          		),   
		.m_axis_result_tdata    ( result         		)    
	);

    assign finish = m_axis_result_tvalid & s_axis_a_tready & s_axis_b_tready;

endmodule