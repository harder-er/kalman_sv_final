`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/15 17:33:29
// Design Name: 
// Module Name: SystolicArrayCore
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

module SystolicArrayCore #(
    parameter DWIDTH   = 64,
    parameter N        = 12,
    parameter LATENCY  = 12
)(
    input  logic                clk,
    input  logic                rst_n,
    input  logic                load_en,

    input  logic [DWIDTH-1:0]   a_row  [0:N-1][0:N-1],
    input  logic [DWIDTH-1:0]   b_col  [0:N-1][0:N-1],

    input  logic                enb_1,
    input  logic                enb_2_6,
    input  logic                enb_7_12,

    output logic [DWIDTH-1:0]   c_out  [0:N-1][0:N-1],
    output logic                cal_finish
);

    logic [DWIDTH-1:0] a_reg [0:N-1][0:N];
    logic [DWIDTH-1:0] b_reg [0:N][0:N-1];
    logic [DWIDTH-1:0] sum_reg [0:N-1][0:N-1];
    logic              ready   [0:N-1][0:N-1];

    logic [$clog2(2*N-1)-1:0] cnt;

    // cnt: 0..2N-2 while load_en=1
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)               cnt <= '0;
        else if (!load_en)        cnt <= '0;
        else if (cnt < 2*N-2)     cnt <= cnt + 1'b1;
    end

    // 边界注入：cnt>=N 时注入 0（padding），避免越界访问
    always_ff @(posedge clk) begin
        if (load_en) begin
            for (int i = 0; i < N; i++) begin
                a_reg[i][0] <= (cnt < N) ? a_row[i][cnt] : '0;
                b_reg[0][i] <= (cnt < N) ? b_col[cnt][i] : '0;
            end
        end
    end

    generate
      for (genvar i = 0; i < N; i++) begin : GEN_ROW
        for (genvar j = 0; j < N; j++) begin : GEN_COL
          logic pe_en;
          assign pe_en = (j==0 && enb_1)
                      || (j>=1 && j<=5 && enb_2_6)
                      || (j>=6 && j<=11 && enb_7_12);

          ProcessingElement #(
              .DWIDTH(DWIDTH),
              .ADD_PIPE_STAGES(LATENCY)
          ) u_pe (
              .clk        ( clk              ),
              .rst_n      ( rst_n            ),
              .en         ( load_en & pe_en  ),
              .a_in       ( a_reg[i][j]      ),
              .b_in       ( b_reg[i][j]      ),
              .sum_down   ( sum_reg[i][j]    ),
              .a_out      ( a_reg[i][j+1]    ),
              .b_out      ( b_reg[i+1][j]    ),
              .sum_right  ( sum_reg[i][j]    ),
              .data_ready ( ready[i][j]      )
          );
        end
      end
    endgenerate

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cal_finish <= 1'b0;
            for (int i = 0; i < N; i++)
              for (int j = 0; j < N; j++)
                c_out[i][j] <= '0;
        end else if (!load_en) begin
            cal_finish <= 1'b0;
        end else if (cnt == 2*N-2) begin
            cal_finish <= 1'b1;
            for (int i = 0; i < N; i++)
              for (int j = 0; j < N; j++)
                c_out[i][j] <= sum_reg[i][j];
        end
    end

endmodule
