`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/03/13 09:58:38
// Design Name: 
// Module Name: StatePredictor
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

module StatePredictor #(
    parameter VEC_WIDTH = 64,
    parameter MAT_DIM = 12
)( 
    input   logic             clk                           ,
    input   logic             rst_n                         ,

    input   logic             Init_Valid                    ,     // 初始化有效信�?    
    output  logic             SP_DONE                       ,     // 状态预测完成信�?  

    input   logic [VEC_WIDTH-1:0] X_kk1  [MAT_DIM-1:0]      ,     // 上一时刻状态估?    
    input   logic [VEC_WIDTH-1:0] K_k    [12-1:0][6-1:0]   ,     // Kalman增益
    input   logic [VEC_WIDTH-1:0] Z_k    [6-1:0]            ,     // 观测?
    output  logic [VEC_WIDTH-1:0] X_kk   [MAT_DIM-1:0]           // 下一时刻状态估?
    );
    // fp_* ready wires
    logic sub_ready [0:5];

    logic [VEC_WIDTH-1:0] HX [6-1:0];
    logic [VEC_WIDTH-1:0] Z_HX [6-1:0]; // 上一时刻状态协方差矩阵
    logic finish[6-1:0];
    logic sub_go;
    logic sub_pending;
    logic sub_inflight;
    logic [5:0] sub_done;
    logic sub_all_done_d;
    logic init_valid_d;
    wire  sub_all_done = &sub_done;
    wire  sub_ready_all = sub_ready[0] & sub_ready[1] & sub_ready[2] & sub_ready[3] & sub_ready[4] & sub_ready[5];
    wire  start_pulse = Init_Valid & ~init_valid_d;
    generate
        for (genvar i = 0; i < 6; i++) begin : gen_HX
            assign HX[i] = X_kk1[i];

            fp_suber u_fp_suber (.clk(clk), 
                .rst_n(rst_n), 
            // .rst_n(rst_n),
                .valid      (   sub_go      ),
                .ready      (   sub_ready[i] ),
        .finish     (   finish[i]   ),
                .a      	(   Z_k[i]      ),
                .b      	(   HX[i]       ),
                .result 	(   Z_HX[i]     ) 
            );

        end

    endgenerate

    // suber handshake control
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_valid_d <= 1'b0;
            sub_pending  <= 1'b0;
            sub_inflight <= 1'b0;
            sub_done     <= '0;
            sub_go       <= 1'b0;
            sub_all_done_d <= 1'b0;
        end else begin
            init_valid_d <= Init_Valid;
            sub_go <= 1'b0;

            if (start_pulse) begin
                sub_pending  <= 1'b1;
                sub_inflight <= 1'b0;
                sub_done     <= '0;
                sub_all_done_d <= 1'b0;
            end

            if (sub_pending && sub_ready_all) begin
                sub_pending  <= 1'b0;
                sub_inflight <= 1'b1;
                sub_go       <= 1'b1;
            end

            if (sub_inflight) begin
                for (int i = 0; i < 6; i++) begin
                    if (!sub_done[i] && finish[i]) sub_done[i] <= 1'b1;
                end
                if (sub_all_done) sub_inflight <= 1'b0;
            end

            if (start_pulse) sub_all_done_d <= 1'b0;
            else sub_all_done_d <= sub_all_done;
        end
    end


    logic [64-1:0] KKmatrix [0:12-1][0:12-1];
    logic [64-1:0] ZHXmatrix [0:12-1][0:12-1];
    logic [64-1:0] Xkkmatrix [0:12-1][0:12-1];
    generate//填充�?2x12矩阵
        for (genvar i = 0; i < 12; i++) begin : row_gen
            for (genvar j = 0; j < 12; j++) begin : col_gen
                always_ff @(posedge clk or negedge rst_n) begin
                    if (!rst_n) begin
                        KKmatrix[i][j] <= 64'h0; // 复位时清�?                        
                        ZHXmatrix[i][j] <= 64'h0; // 复位时清�?                    
                    end else  begin
                        KKmatrix[i][j] <= (j < 6) ? K_k[i][j] : 64'h0;
                        ZHXmatrix[i][j] <= (j < 6&&i == 0) ? Z_HX[i] : 64'h0;
                    end
                end
            end
        end
    endgenerate


    logic sys_load_en;
    logic systolic_done;
    assign sys_load_en = sub_all_done & ~sub_all_done_d;
    SystolicArray #(
        .DWIDTH(64),
        .N(12),
        .LATENCY(12)
    ) u_systolic (
        .clk        (   clk             ),
        .rst_n      (   rst_n           ),
        .a_row      (   KKmatrix        ),
        .b_col      (   ZHXmatrix       ),
        .load_en    (   sys_load_en     ),
        .enb_1      (   1'b1            ),
        .enb_2_6    (   1'b1            ),
        .enb_7_12   (   1'b0            ),
        .c_out      (   Xkkmatrix       ),
        .cal_finish (   systolic_done   )
    );

    generate
        for (genvar i = 0; i < 12; i++) begin 
           assign X_kk[i] = Xkkmatrix[i][0];
        end
    endgenerate

    assign SP_DONE = systolic_done;

endmodule


