`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/05/07 10:05:45
// Design Name: 
// Module Name: fp_suber
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



module fp_suber(
    input  logic clk,
    input  logic [64-1:0] a, b,
    input  logic valid,
    output logic finish,
    output logic [64-1:0] result
);
    logic s_axis_a_tready;
    logic s_axis_b_tready;
    logic m_axis_result_tvalid;
    floating_point_sub u_floating_point_sub(
    .aclk                 ( clk                  ),
    .s_axis_a_tvalid      ( valid                ),
    .s_axis_a_tready      ( s_axis_a_tready      ),
    .s_axis_a_tdata       ( a                    ),
    .s_axis_b_tvalid      ( valid                ),
    .s_axis_b_tready      ( s_axis_b_tready      ),
    .s_axis_b_tdata       ( b                    ),
    .m_axis_result_tvalid ( m_axis_result_tvalid ),
    .m_axis_result_tready ( 1'b1                 ),
    .m_axis_result_tdata  ( result               )
);

assign finish = m_axis_result_tvalid & s_axis_a_tready & s_axis_b_tready;
endmodule