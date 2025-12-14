`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/24 15:40:54
// Design Name: 
// Module Name: SystolicArray
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


module SystolicArray #(
    parameter DWIDTH   = 64,
    parameter N        = 12,
    parameter LATENCY  = 12
)(
    input  logic                clk,
    input  logic                rst_n,
    input  logic                load_en,
    input  logic [DWIDTH-1:0]   a_row  [0:N-1][0:N-1],
    input  logic [DWIDTH-1:0]   b_col  [0:N-1][0:N-1],
    input  logic                enb_1,      // 使能第1列
    input  logic                enb_2_6,    // 使能第2–6列
    input  logic                enb_7_12,   // 使能第7–12列
    output logic [DWIDTH-1:0]   c_out  [0:N-1][0:N-1],
    output logic                cal_finish
);

    // 行列移位寄存器，额外多一行/列存边界数据
    logic [DWIDTH-1:0] a_reg [0:N-1][0:N];
    logic [DWIDTH-1:0] b_reg [0:N][0:N-1];
    // 累加和寄存
    logic [DWIDTH-1:0] sum_reg [0:N-1][0:N-1];
    // 数据就绪
    logic               ready   [0:N-1][0:N-1];

    // 时序计数器，从 0 ~ 2N-2
    logic [$clog2(2*N-1)-1:0] cnt;

    // step1：启动后计数
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)          cnt <= 0;
        else if (load_en && cnt < 2*N-2) cnt <= cnt + 1;
        else if (!load_en)    cnt <= 0;
    end

    // step2：边界注入 A 和 B
    always_ff @(posedge clk) begin
        if (load_en) begin
            for (int i = 0; i < N; i++) begin
                a_reg[i][0] <= a_row[i][cnt];     // A[i][k]
                b_reg[0][i] <= b_col[cnt][i];     // B[k][i]
            end
        end
    end

    // step3：实例化 PE 阵列
    generate
      for (genvar i = 0; i < N; i++) begin
        for (genvar j = 0; j < N; j++) begin

          // 列使能解码
          logic pe_en;
          assign pe_en = (j==0 && enb_1)
                      || (j>=1 && j<=5 && enb_2_6)
                      || (j>=6 && j<=11&& enb_7_12);

          ProcessingElement #(
              .DWIDTH(DWIDTH),
              .ADD_PIPE_STAGES(LATENCY)
          ) u_pe (
              .clk       (clk               ),
              .rst_n     (rst_n             ),
              .en        (load_en & pe_en   ),
              .a_in      (a_reg[i][j]       ),
              .b_in      (b_reg[i][j]       ),
              .sum_down  (sum_reg[i][j]     ),
              .a_out     (a_reg[i][j+1]     ),
              .b_out     (b_reg[i+1][j]     ),
              .sum_right (sum_reg[i][j]     ),
              .data_ready(ready[i][j]       )
          );

        end
      end
    endgenerate

    // step4：计算完成与输出
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cal_finish <= 1'b0;
            for (int i = 0; i < N; i++)
              for (int j = 0; j < N; j++)
                c_out[i][j] <= 0;
        end else if (cnt == 2*N-2) begin
            cal_finish <= 1'b1;
            // 结果已驻留在 sum_reg 中
            for (int i = 0; i < N; i++)
              for (int j = 0; j < N; j++)
                c_out[i][j] <= sum_reg[i][j];
        end
    end

endmodule
