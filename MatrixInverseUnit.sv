`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/03/10 17:01:06
// Design Name: 
// Module Name: MatrixInverseUnit  // 矩阵求逆单元模块
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 实现3x3矩阵求逆运算，支持浮点数据格式的流水线处理
// 
// Dependencies: 依赖CEU系列浮点运算单元（CEU_a, CEU_d, CEU_division等）
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments: 采用三级流水线架构，支持矩阵元素的并行计算
// 
//////////////////////////////////////////////////////////////////////////////////


module MatrixInverseUnit #(
    parameter DWIDTH = 64  // 数据宽度（浮点精度位宽）
)(
    input  logic                clk                     ,      
    input  logic                rst_n                   ,      
    input  logic                valid                   ,      
    input  logic [DWIDTH-1:0]   P_k1k1 [0:12-1][0:12-1] ,   //p_k-1,k-1
    input  logic [DWIDTH-1:0]   Q_k [0:12-1][0:12-1]    ,  
    input  logic [DWIDTH-1:0]   R_k [0:5][0:5]          ,  
    output logic                finish                  ,   
    output logic [DWIDTH-1:0]   inv_matrix [0:5][0:5]
);


// ==== 中间结果寄存器（对应图中Intermediate Result Reg）
//-----------------------------------------------------------------
logic [DWIDTH-1:0] a, b, c, d, e, f, x, y, z;  // 矩阵运算中间变量

// ==== CEU模块实例化（按流程图处理顺序）
CEU_a #(
    .DBL_WIDTH(64)
) u_CEU_a (
    .clk          (clk),
    .rst_n        (rst_n),
    // 动态输入：替换为 P_k1k1[i][j] 形式
    // .Theta_1_1    (P_k1k1[0][0]),
    .Theta_4_1    (P_k1k1[3][0]),
    .Theta_7_1    (P_k1k1[6][0]),
    .Theta_4_4    (P_k1k1[3][3]),
    .Theta_10_1   (P_k1k1[9][0]),
    .Theta_7_4    (P_k1k1[6][3]),
    .Theta_10_4   (P_k1k1[9][3]),
    .Theta_7_7    (P_k1k1[6][6]),
    .Theta_10_7   (P_k1k1[9][6]),
    .Theta_10_10  (P_k1k1[9][9]),
    .Q_1_1        (Q_k[0][0]),
    .R_1_1        (R_k[0][0]),
    // 固定参数
    .dt_1   (delta_t2           ),
    .dt_2   (delta_t_sq         ),
    .dt_3   (delta_t_cu         ),
    .dt_4   (delta_t_qu         ),
    .dt_5   (delta_t_qi         ),
    .dt_6   (delta_t_sx         ),
    // 输出 
    .a_out        (a            ),
    .valid_out    (valid_out    )
);

CEU_a #(
    .DBL_WIDTH(64)
) u_CEU_b (
    .clk          (clk),
    .rst_n        (rst_n),
    // 动态输入：替换为 P_k1k1[i][j] 形式
    .Theta_4_1    (P_k1k1[4][1]),
    .Theta_7_1    (P_k1k1[7][1]),
    .Theta_4_4    (P_k1k1[4][4]),
    .Theta_10_1   (P_k1k1[10][1]),
    .Theta_7_4    (P_k1k1[7][4]),
    .Theta_10_4   (P_k1k1[10][4]),
    .Theta_7_7    (P_k1k1[7][7]),
    .Theta_10_7   (P_k1k1[10][7]),
    .Theta_10_10  (P_k1k1[10][10]),
    .Q_1_1        (Q_k[1][1]),
    .R_1_1        (R_k[1][1]),
    // 固定参数
    .dt_1     (delta_t2),
    .dt_2   (delta_t_sq),
    .dt_3   (delta_t_cu),
    .dt_4   (delta_t_qu),
    .dt_5   (delta_t_qi),
    .dt_6   (delta_t_sx),
    // 输出
    .a_out        (b),
    .valid_out    (valid_out)
);
    

CEU_a #(
    .DBL_WIDTH(64)
) u_CEU_c (
    .clk          (clk),
    .rst_n        (rst_n),
    // 动态输入：替换为 P_k1k1[i][j] 形式
    // .Theta_1_1    (P_k1k1[2][2]),
    .Theta_4_1    (P_k1k1[5][2]),
    .Theta_7_1    (P_k1k1[8][2]),
    .Theta_4_4    (P_k1k1[5][5]),
    .Theta_10_1   (P_k1k1[11][2]),
    .Theta_7_4    (P_k1k1[8][5]),
    .Theta_10_4   (P_k1k1[11][5]),
    .Theta_7_7    (P_k1k1[8][8]),
    .Theta_10_7   (P_k1k1[11][8]),
    .Theta_10_10  (P_k1k1[11][11]),
    .Q_1_1        (Q_k[2][2]),
    .R_1_1        (R_k[2][2]),
    // 固定参数
    .dt_1     (delta_t2),
    .dt_2   (delta_t_sq),
    .dt_3   (delta_t_cu),
    .dt_4   (delta_t_qu),
    .dt_5   (delta_t_qi),
    .dt_6   (delta_t_sx),
    // 输出
    .a_out        (c),
    .valid_out    (valid_out)
);

// ---------------------------------------------------------
// 在上层模块中例化 CEU_d（"d" 通道核心计算）
// ---------------------------------------------------------

// 把所有 (i,j) 改为 (i-1, j-1)
CEU_d #(
    .DBL_WIDTH(64)
) u_CEU_d (
    .clk         (clk),
    .rst_n       (rst_n),
    // 动态输入：索引均减一
    .Theta_10_7  (P_k1k1[9][6]),    // 10→9, 7→6
    .Theta_7_4   (P_k1k1[6][3]),    // 7→6, 4→3
    .Theta_10_4  (P_k1k1[9][3]),    // 10→9, 4→3
    .Theta_4_7   (P_k1k1[3][6]),    // 4→3, 7→6
    .Theta_10_10 (P_k1k1[9][9]),    // 10→9, 10→9
    .Theta_4_4   (P_k1k1[3][3]),    // 4→3, 4→3
    .Q_4_4       (Q_k[3][3]),        // 4→3, 4→3
    .R_4_4       (R_k[3][3]),        // 4→3, 4→3
    // 固定时间参数
    .delta_t2    (delta_t2),
    .delta_t_sq  (delta_t_sq),
    .delta_t_hcu (delta_t_hcu),
    .delta_t_qr  (delta_t_qr),
    // 输出
    .d           (d),
    .valid_out   (valid_minus)
);


CEU_d #(
    .DBL_WIDTH(64)
) u_CEU_e (
    .clk         (clk),
    .rst_n       (rst_n),
    // 动态输入：按 "d" 通道对应的 Θ
    .Theta_10_7  (P_k1k1[10][7]),    // 或者你自己的信号名
    .Theta_7_4   (P_k1k1[7][4]),
    .Theta_10_4  (P_k1k1[10][4]),
    .Theta_4_7   (P_k1k1[4][7]),
    .Theta_10_10 (P_k1k1[10][10]),
    .Theta_4_4   (P_k1k1[4][4]),
    .Q_4_4       (Q_k[4][4]),
    .R_4_4       (R_k[4][4]),
    // 固定时间参数（上层预先计算或定义）
    .delta_t2    (delta_t2),
    .delta_t_sq  (delta_t_sq),
    .delta_t_hcu (delta_t_hcu),
    .delta_t_qr  (delta_t_qr),
    // 输出
    .d           (e),
    .valid_out   (d_valid)          // 或者 valid_d，根据你的命名
);

// 把所有 (i,j) 改为 (i+1, j+1)
CEU_d #(
    .DBL_WIDTH(64)
) u_CEU_f (
    .clk         (clk),
    .rst_n       (rst_n),
    // 动态输入：索引均加一
    .Theta_10_7  (P_k1k1[11][8]),   // 10→11, 7→8
    .Theta_7_4   (P_k1k1[8][5]),    // 7→8, 4→5
    .Theta_10_4  (P_k1k1[11][5]),   // 10→11, 4→5
    .Theta_4_7   (P_k1k1[5][8]),    // 4→5, 7→8
    .Theta_10_10 (P_k1k1[11][11]),  // 10→11, 10→11
    .Theta_4_4   (P_k1k1[5][5]),    // 4→5, 4→5
    .Q_4_4       (Q_k[5][5]),        // 4→5, 4→5
    .R_4_4       (R_k[5][5]),        // 4→5, 4→5
    // 固定时间参数
    .delta_t2    (delta_t2),
    .delta_t_sq  (delta_t_sq),
    .delta_t_hcu (delta_t_hcu),
    .delta_t_qr  (delta_t_qr),
    // 输出
    .d           (f),
    .valid_out   (valid_plus)
);

// ---------------------------------------------------------
// CEU_x 模块例化
// ---------------------------------------------------------

CEU_x #(
    .DBL_WIDTH(64)
) u_CEU_x (
    .clk        (clk),
    .rst_n      (rst_n),

    .Theta_1_7  (P_k1k1[0][6]),   // 1→0, 7→6
    .Theta_4_4  (P_k1k1[3][3]),   // 4→3, 4→3

    .Theta_7_4  (P_k1k1[6][3]),   // 7→6, 4→3
    .Theta_10_4 (P_k1k1[9][3]),   // 10→9, 4→3
    .Theta_7_7  (P_k1k1[6][6]),   // 7→6, 7→6

    .Theta_10_1 (P_k1k1[9][0]),   // 10→9, 1→0

    .Theta_10_7 (P_k1k1[9][6]),   // 10→9, 7→6
    .Theta_10_10(P_k1k1[9][9]),   // 10→9, 10→9
    .Theta_1_4  (P_k1k1[0][3]),   // 1→0, 4→3

    .Q_1_4      (Q_k[0][3]),       // 1→0, 4→3
    .R_1_4      (R_k[0][3]),       // 1→0, 4→3

    .delta_t    (delta_t),
    .half_dt2   (delta_t2),
    .sixth_dt3 (delta_t_sq),
    .five12_dt4 (delta_t_cu),
    .one12_dt5 (delta_t_qu),

    .x          (x),
    .valid_out  (x_valid_minus1)
);


CEU_x #(
    .DBL_WIDTH(64)
) u_CEU_y (
    .clk        (clk),
    .rst_n      (rst_n),

    .Theta_1_7  (P_k1k1[1][7]),   
    .Theta_4_4  (P_k1k1[4][4]),   

    .Theta_7_4  (P_k1k1[7][4]),   
    .Theta_10_4 (P_k1k1[10][4]),   
    .Theta_7_7  (P_k1k1[7][7]),   

    .Theta_10_1 (P_k1k1[10][1]),  

    .Theta_10_7 (P_k1k1[10][7]),    
    .Theta_10_10(P_k1k1[10][10]),
    .Theta_1_4  (P_k1k1[1][4]),  

    // 噪声项 Q/R
    .Q_1_4      (Q_k[1][4]),       // Q[1,7]
    .R_1_4      (R_k[1][4]),       // R[1,7]

    // 固定时间参数
    .delta_t    (delta_t),
    .half_dt2   (delta_t2),
    .sixth_dt3 (delta_t_sq),
    .five12_dt4 (delta_t_cu),
    .one12_dt5 (delta_t_qu),

    // 输出
    .x          (y),
    .valid_out  (x_valid)
);

CEU_x #(
    .DBL_WIDTH(64)
) u_CEU_z (
    .clk        (clk),
    .rst_n      (rst_n),

    .Theta_1_7  (P_k1k1[2][8]),   // 1→2, 7→8
    .Theta_4_4  (P_k1k1[5][5]),   // 4→5, 4→5

    .Theta_7_4  (P_k1k1[8][5]),   // 7→8, 4→5
    .Theta_10_4 (P_k1k1[11][5]),  // 10→11, 4→5
    .Theta_7_7  (P_k1k1[8][8]),   // 7→8, 7→8

    .Theta_10_1 (P_k1k1[11][2]),  // 10→11, 1→2

    .Theta_10_7 (P_k1k1[11][8]),  // 10→11, 7→8
    .Theta_10_10(P_k1k1[11][11]), // 10→11, 10→11)
    .Theta_1_4  (P_k1k1[2][5]),   // 1→2, 4→5

    .Q_1_4      (Q_k[2][5]),       // 1→2, 4→5
    .R_1_4      (R_k[2][5]),       // 1→2, 4→5

    .delta_t    (delta_t),
    .half_dt2   (delta_t2),
    .sixth_dt3  (delta_t_sq),
    .five12_dt4 (delta_t_cu),
    .one12_dt5  (delta_t_qu),

    .x          (z),
    .valid_out  (x_valid_plus1)
);

// ================== 第二计算阶段：行列式计算 ==================
// 计算α = a*d - x^2?（行列式核心项）
logic [DWIDTH-1:0] alpha1, alpha2, alpha3;
logic [DWIDTH-1:0] inv_alpha11, inv_alpha12, inv_alpha13;
logic [DWIDTH-1:0] inv_alpha21, inv_alpha22, inv_alpha23;
logic [DWIDTH-1:0] inv_alpha31, inv_alpha32, inv_alpha33;
logic [DWIDTH-1:0] _inv_alpha13, _inv_alpha23, _inv_alpha33;
logic valid_out1, valid_out2, valid_out3;

CEU_alpha u_CEU_alpha1 (
    .clk        (clk            ),
    .rst_n      (rst_n          ),
    .in1        (a              ),             // 第一行第一列乘积项
    .in2        (d              ),             // 第二行第二列乘积项
    .in3        (x              ),             // 交叉项平方
    .out        (alpha1         ),          // 输出行列式值α
    .valid_out  (valid_out1     )
);
CEU_alpha u_CEU_alpha2 (
    .clk        (clk            ),
    .rst_n      (rst_n          ),
    .in1        (b              ),             // 第一行第一列乘积项
    .in2        (e              ),             // 第二行第二列乘积项
    .in3        (y              ),             // 交叉项平方
    .out        (alpha2         ),          // 输出行列式值α
    .valid_out   (valid_out2    )
);

CEU_alpha u_CEU_alpha3 (
    .clk        (clk        ),
    .rst_n      (rst_n      ),
    .in1        (c          ),             // 第一行第一列乘积项
    .in2        (f          ),             // 第二行第二列乘积项
    .in3        (z          ),             // 交叉项平方
    .out        (alpha3     ),          // 输出行列式值α
    .valid_out  (valid_out3 )
);
logic valid_div;
assign valid_div = valid & valid_out1 & valid_out2 & valid_out3; 
// ================== 第三计算阶段：逆矩阵元素计算 ==================
// 计算1/α（行列式倒数）——单个除法IP串行复用 9 次
typedef enum logic [1:0] {DIV_IDLE, DIV_BUSY} div_state_e;
div_state_e div_state;
logic [3:0] div_idx;
logic div_go, div_finish;
logic [DWIDTH-1:0] div_num, div_den, div_q;

CEU_division u_CEU_div_shared (
    .clk        (clk),
    .valid      (div_go),
    .finish     (div_finish),
    .numerator  (div_num),
    .denominator(div_den),
    .quotient   (div_q)
);

// 结果收集标志
logic all_div_done;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        div_state    <= DIV_IDLE;
        div_idx      <= '0;
        div_go       <= 1'b0;
        all_div_done <= 1'b0;
        {inv_alpha11, inv_alpha12, inv_alpha13,
         inv_alpha21, inv_alpha22, inv_alpha23,
         inv_alpha31, inv_alpha32, inv_alpha33} <= '{default:'0};
    end else begin
        div_go       <= 1'b0;
        all_div_done <= 1'b0;

        case (div_state)
            DIV_IDLE: begin
                div_idx <= '0;
                if (valid_div) begin
                    div_num <= a;      div_den <= alpha1; div_go <= 1'b1; div_state <= DIV_BUSY;
                end
            end

            DIV_BUSY: begin
                if (div_finish) begin
                    case (div_idx)
                        4'd0: inv_alpha11 <= div_q;
                        4'd1: inv_alpha12 <= div_q;
                        4'd2: inv_alpha13 <= div_q;
                        4'd3: inv_alpha21 <= div_q;
                        4'd4: inv_alpha22 <= div_q;
                        4'd5: inv_alpha23 <= div_q;
                        4'd6: inv_alpha31 <= div_q;
                        4'd7: inv_alpha32 <= div_q;
                        4'd8: inv_alpha33 <= div_q;
                        default: ;
                    endcase

                    if (div_idx == 4'd8) begin
                        div_state    <= DIV_IDLE;
                        all_div_done <= 1'b1;
                    end else begin
                        div_idx <= div_idx + 1'b1;
                        case (div_idx + 1'b1)
                            4'd1: begin div_num <= d; div_den <= alpha1; end
                            4'd2: begin div_num <= x; div_den <= alpha1; end
                            4'd3: begin div_num <= b; div_den <= alpha2; end
                            4'd4: begin div_num <= e; div_den <= alpha2; end
                            4'd5: begin div_num <= y; div_den <= alpha2; end
                            4'd6: begin div_num <= c; div_den <= alpha3; end
                            4'd7: begin div_num <= f; div_den <= alpha3; end
                            4'd8: begin div_num <= z; div_den <= alpha3; end
                            default: begin div_num <= '0; div_den <= '0; end
                        endcase
                        div_go <= 1'b1;
                    end
                end
            end
        endcase
    end
end

// 三个 -x/-y/-z 的符号翻转在除法完成后统一触发
logic fpsub13_finish,fpsub23_finish,fpsub33_finish;
assign fpsub13_valid = all_div_done;
assign fpsub23_valid = all_div_done;
assign fpsub33_valid = all_div_done;

fp_suber u_fp_suber_x (
    .clk        (clk            ),
    .valid      (fpsub13_valid  ),
    .finish     (fpsub13_finish ),
    .a          (64'h0          ),
    .b          (inv_alpha13    ),
    .result     (_inv_alpha13   )
);

fp_suber u_fp_suber_y (
    .clk        (clk            ),
    .valid      (fpsub23_valid  ),
    .finish     (fpsub23_finish ),
    .a          (64'h0          ),
    .b          (inv_alpha23    ),
    .result     (_inv_alpha23   )
);

fp_suber u_fp_suber_z (
    .clk        (clk            ),
    .valid      (fpsub33_valid  ),
    .finish     (fpsub33_finish ),
    .a          (64'h0          ),
    .b          (inv_alpha33    ),
    .result     (_inv_alpha33   )
);
// ================== 输出阶段：逆矩阵元素合成 ==================
// 计算逆矩阵第一行第一列元素：(d*e - y^2?)/α
    // 声明：6×6 寄存器缓存
    logic [DWIDTH-1:0] result_reg [0:5][0:5];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // 复位时全部清零
            for (int i = 0; i < 6; i++) begin
                for (int j = 0; j < 6; j++) begin
                    result_reg[i][j] <= '0;
                end
            end
        end else begin
            // Row 0
            result_reg[0][0] <= inv_alpha12;               // d / (a*d - x^2)
            result_reg[0][1] <= 64'h0;                           // 矩阵第0行第1列为 0
            result_reg[0][2] <= 64'h0;                           // 第0行第2列为 0
            result_reg[0][3] <= _inv_alpha13 ;               // -x/(a*d - x^2)
            result_reg[0][4] <= 64'h0;
            result_reg[0][5] <= 64'h0;

            // Row 1
            result_reg[1][0] <= 64'h0;
            result_reg[1][1] <= inv_alpha22;                // e / (b*e - y^2)
            result_reg[1][2] <= 64'h0;
            result_reg[1][3] <= 64'h0;
            result_reg[1][4] <= _inv_alpha23;                // -y/(b*e - y^2)
            result_reg[1][5] <= 64'h0;

            // Row 2
            result_reg[2][0] <= 64'h0;
            result_reg[2][1] <= 64'h0;
            result_reg[2][2] <= inv_alpha32;               // f / (c*f - z^2)
            result_reg[2][3] <= 64'h0;
            result_reg[2][4] <= 64'h0;
            result_reg[2][5] <= _inv_alpha33;               // -z/(c*f - z^2)

            // Row 3
            result_reg[3][0] <= _inv_alpha13;               // -x/(a*d - x^2)
            result_reg[3][1] <= 64'h0;
            result_reg[3][2] <= 64'h0;
            result_reg[3][3] <= inv_alpha11;               // a/(a*d - x^2)
            result_reg[3][4] <= 64'h0;
            result_reg[3][5] <= 64'h0;

            // Row 4
            result_reg[4][0] <= 64'h0;
            result_reg[4][1] <= _inv_alpha23;                // -y/(b*e - y^2)
            result_reg[4][2] <= 64'h0;
            result_reg[4][3] <= 64'h0;
            result_reg[4][4] <= inv_alpha21;                // b/(b*e - y^2)
            result_reg[4][5] <= 64'h0;

            // Row 5
            result_reg[5][0] <= 64'h0;
            result_reg[5][1] <= 64'h0;
            result_reg[5][2] <= _inv_alpha33;               // -z/(c*f - z^2)
            result_reg[5][3] <= 64'h0;
            result_reg[5][4] <= 64'h0;
            result_reg[5][5] <= inv_alpha31;               // c/(c*f - z^2)
        end
    end


assign inv_matrix = result_reg;        
assign finish = fpsub13_finish & fpsub23_finish & fpsub33_finish; 

endmodule
