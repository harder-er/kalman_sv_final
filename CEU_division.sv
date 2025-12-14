`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/25 17:48:01
// Design Name: 
// Module Name: CEU_division
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
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: CEU_division
// Description:
//   对外提供一个除法接口，将组合信号 {b,e,y} 作为分子，alpha 作为分母，
//   输出它们的浮点商 inv_alpha = (b,e,y) / alpha。
// Dependencies:
//   已生成的 Xilinx/AMD Floating-Point Divider IP 核 (64-bit IEEE-754)
//
//////////////////////////////////////////////////////////////////////////////////

module CEU_division #(
    parameter DBL_WIDTH = 64           // IEEE-754 double precision
)(
    input  logic                   clk          ,  // 时钟信号
    input  logic                   valid        ,
    output logic                   finish       ,  // 浮点除法完成信号
    input  logic [DBL_WIDTH-1:0]   numerator    ,    // 分子 {b,e,y} 组合结果
    input  logic [DBL_WIDTH-1:0]   denominator  ,  // 分母 alpha（行列式值）
    output logic [DBL_WIDTH-1:0]   quotient      // 输出 1/α 或分子／分母
);

    // --- 浮点除法 IP 核 信号声明 ---
    // 以下信号名称对应 Core Generator 生成的 AXI-Stream 接口
    // ds335.pdf 中的示例通常为：
    //   s_axis_dividend_tdata, s_axis_dividend_tvalid
    //   s_axis_divisor_tdata,  s_axis_divisor_tvalid
    //   m_axis_dout_tdata,     m_axis_dout_tvalid
    //
    logic                     div_dividend_tvalid = valid;
    logic                     div_divisor_tvalid  = valid;
    logic                     div_dividend_tready;
    logic                     div_divisor_tready;
    logic                     div_dout_tvalid;
    logic                     div_dout_tready  = 1'b1;


    floating_point_div u_floating_point_div(
        .aclk                 ( clk                     ),
        .s_axis_a_tdata       ( numerator               ),
        .s_axis_a_tvalid      ( div_dividend_tvalid     ),
        .s_axis_a_tready      ( div_dividend_tready     ),

        .s_axis_b_tdata       ( denominator             ),
        .s_axis_b_tvalid      ( div_divisor_tvalid      ),
        .s_axis_b_tready      ( div_divisor_tready      ),
        
        .m_axis_result_tdata  ( quotient                ),
        .m_axis_result_tvalid ( div_dout_tvalid         ),
        .m_axis_result_tready ( div_dout_tready         )
    );

    assign finish = div_dout_tvalid & div_dividend_tready & div_divisor_tready;

endmodule

