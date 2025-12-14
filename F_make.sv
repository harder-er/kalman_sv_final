`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/05/12 11:32:40
// Design Name: 
// Module Name: F_make
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


module F_make (
    input           clk     ,   
    input           rst_n   ,   
    output          finish  ,   
    input  [63:0]   deltat  ,   
    output reg [63:0] F [0:11][0:11] // 12x12双精度浮点矩阵
);

// 浮点运算IP核声明
    logic [63:0] deltat_sq, deltat_cu, deltat_sq_div2, deltat_cu_div6, deltat_sq_div6;

    logic dt2_finish, dt3_finish;

    fp_multiplier u_fp_mult_dt2 (
        .clk        (   clk         ),
        .a          (   deltat      ),
        .b          (   deltat      ),
        .valid      (   1'b1        ),
        .finish     (   dt2_finish  ),
        .result     (   deltat_sq   )
    );

    fp_multiplier u_fp_mult_dt3 (
        .clk    (   clk         ),
        .valid  (   dt2_finish  ),
        .finish (   dt3_finish  ),
        .a      (   deltat_sq   ),
        .b      (   deltat      ),
        .result (   deltat_cu   )
    );


    logic   div_dividend_tvalid = dt3_finish    ;
    logic   div_divisor_tvalid  = dt3_finish    ;
    logic   div_dividend_tready                 ;
    logic   div_divisor_tready                  ;
    logic   div_dout_tvalid                     ;    
    logic   div_dout_tready     = dt3_finish    ;

    // --- 顶层实例化 Floating-Point Divider IP ---


// 正确实例化 floating_point_div 模块
    floating_point_div u_floating_point_div2 (
        .aclk                   (   clk                    ), 
        // 被除数输入 (A 通道)  
        .s_axis_a_tvalid        (   div_dividend_tvalid    ), 
        .s_axis_a_tready        (   div_divisor_tready     ), 
        .s_axis_a_tdata         (   deltat_sq              ), 
        // 除数输入 (B 通道)    
        .s_axis_b_tvalid        (   div_divisor_tvalid     ), 
        .s_axis_b_tready        (   s_axis_divisor_tready  ), 
        .s_axis_b_tdata         (   64'h4000000000000000   ), 
        // 结果输出 
        .m_axis_result_tvalid   (   div_dout_tvalid        ), 
        .m_axis_result_tready   (   div_dout_tready        ), 
        .m_axis_result_tdata    (   deltat_sq_div2         )  
    );
    

    logic   div_dividend_tvalid6 = dt3_finish ;
    logic   div_divisor_tvalid6  = dt3_finish ;
    logic   div_dividend_tready6              ;
    logic   div_divisor_tready6               ;
    logic   div_dout_tvalid6                  ;
    logic   div_dout_tready6     = dt3_finish ;
    floating_point_div u_floating_point_div6 (
        .aclk                   ( clk                   ),   
        // 被除数输入 (A 通道) 
        .s_axis_a_tvalid        ( div_dividend_tvalid6  ),   
        .s_axis_a_tready        ( div_dividend_tready6  ),   
        .s_axis_a_tdata         ( deltat_cu             ),   
        // 除数输入 (B 通道) 
        .s_axis_b_tvalid        ( div_divisor_tvalid6   ),   
        .s_axis_b_tready        ( div_divisor_tready6   ),   
        .s_axis_b_tdata         ( 64'h4018000000000000  ),   
        // 结果输出 
        .m_axis_result_tvalid   ( div_dout_tvalid6      ),   
        .m_axis_result_tready   ( div_dout_tready6      ),   
        .m_axis_result_tdata    ( deltat_cu_div6)            
    );

    logic   div_dividend_tvalidsq6 = dt3_finish   ;
    logic   div_divisor_tvalidsq6  = dt3_finish   ;
    logic   div_dividend_treadysq6                ;
    logic   div_divisor_treadysq6                 ;
    logic   div_dout_tvalidsq6                    ;
    logic   div_dout_treadysq6    = dt3_finish    ;


    floating_point_div u_floating_point_divsq6 (
        .aclk                   (   clk                     ),    
        // 被除数输入 (A 通道)  
        .s_axis_a_tvalid        (   div_dividend_tvalidsq6  ),    
        .s_axis_a_tready        (   div_dividend_treadysq6  ),    
        .s_axis_a_tdata         (   deltat_sq               ),    
        // 除数输入 (B 通道)    
        .s_axis_b_tvalid        (   div_divisor_tvalidsq6   ),    
        .s_axis_b_tready        (   div_divisor_treadysq6   ),    
        .s_axis_b_tdata         (   64'h4018000000000000    ),    
        // 结果输出 
        .m_axis_result_tvalid   (   div_dout_tvalidsq6      ),    
        .m_axis_result_tready   (   div_dout_treadysq6      ),    
        .m_axis_result_tdata    (   deltat_sq_div6          )     
    );


assign finish = div_dividend_treadysq6 & div_divisor_treadysq6 & div_dout_tvalidsq6 
                & div_dividend_tready6 & div_divisor_tready6   & div_dout_tvalid6
                & div_dividend_tready  & div_divisor_tready    & div_dout_tvalid    ;

always_comb begin
    // Initialize all elements of F to 0
    for (int i = 0; i < 12; i ++) begin
        for (int j = 0; j < 12; j ++) begin
            F[i][j] = 64'h0000000000000000; // Assuming F is a 64-bit wide signal
        end
    end
    
    // 保持主对角线为1.0
    for (int i = 0; i < 12; i ++) F[i][i] = 64'h3FF0000000000000;
    
    // 动态更新非对角元素
    // 一阶项（Δt）
    F[0][3]   = deltat;
    F[1][4]   = deltat;
    F[2][5]   = deltat;
    F[3][6]   = deltat;
    F[4][7]   = deltat;
    F[5][8]   = deltat;
    F[6][9]   = deltat;
    F[7][10]  = deltat;
    F[8][11]  = deltat;
    
    // 二阶项（1/2Δt²）
    F[0][6]   = deltat_sq_div2;
    F[1][7]   = deltat_sq_div2;
    F[2][8]   = deltat_sq_div2;
    F[3][9]   = deltat_sq_div2;
    F[4][10]  = deltat_sq_div2;
    F[5][11]  = deltat_sq_div2;

    // 三阶项（1/6Δt³）
    F[0][9]   = deltat_cu_div6;
    F[1][10]  = deltat_cu_div6;
    F[2][11]  = deltat_cu_div6;
end

endmodule
