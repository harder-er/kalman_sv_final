`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/24 15:55:52
// Design Name: 
// Module Name: ProcessingElement
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


module ProcessingElement #(
    parameter DWIDTH = 64,
    parameter ADD_PIPE_STAGES = 2
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             en,
    input  logic [DWIDTH-1:0] a_in,
    input  logic [DWIDTH-1:0] b_in,
    input  logic [DWIDTH-1:0] sum_down,
    
    output logic [DWIDTH-1:0] a_out,
    output logic [DWIDTH-1:0] b_out,
    output logic [DWIDTH-1:0] sum_right,
    output logic             data_ready
);

// ████�?状态机定义（符合状态转移图�?
typedef enum logic [2:0] {
    IDLE, INIT, MUL, ADD, SEND_DATA, DATA_THROUGH, END2
} fsm_state;

fsm_state current_state, next_state;

// ████�?数据寄存�?
logic [DWIDTH-1:0] a_reg, b_reg;
logic [DWIDTH-1:0] partial_sum;
logic [DWIDTH-1:0] partial_sum_reg;
logic [DWIDTH-1:0] sum_temp;
// output declaration of module fp_multiplier




// ████�?控制信号
logic mul_start,  add_start     ;
logic mul_finish, add_finish    ;
logic mul_ready, add_ready;
logic mul_pending, add_pending;

logic data_through_finish;
always_ff @(posedge clk) begin
    if(!rst_n) begin
        data_through_finish <= 1'b0;
    end else begin
        data_through_finish <= en;
    end
end



fp_multiplier u_fp_multiplier (.clk(clk),
    .rst_n   (rst_n),
    .valid   (mul_start      ),
    .ready   (mul_ready),
    .a       (a_reg          ),
    .b       (b_reg          ),
    .finish  (mul_finish      ),
    .result  (partial_sum    )
);


fp_adder u_fp_adder_st (.clk(clk), 
    .rst_n(rst_n),
    .valid   (add_start          ),
    .ready   (add_ready),
    .a       (partial_sum_reg    ),
    .b       (sum_down           ),
    .finish  (add_finish          ),
    .result  (sum_temp)
);


// ████ 状态转移逻辑（对应状态图�?
always_ff @(posedge clk) begin
    if(!rst_n) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end

// launch one request per state entry, hold until ready
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mul_pending <= 1'b0;
        add_pending <= 1'b0;
    end else begin
        if (current_state != MUL && next_state == MUL) begin
            mul_pending <= 1'b1;
        end else if (mul_pending && mul_ready) begin
            mul_pending <= 1'b0;
        end

        if (current_state != ADD && next_state == ADD) begin
            add_pending <= 1'b1;
        end else if (add_pending && add_ready) begin
            add_pending <= 1'b0;
        end
    end
end

always_comb begin
    next_state = current_state;
    case(current_state)
        IDLE: 
            if(en) next_state = INIT;
            else if(!en) next_state = DATA_THROUGH;
        INIT: 
            next_state = MUL;
        
        MUL: 
            if(mul_finish) next_state = ADD;
        
        ADD: 
            if(add_finish) next_state = SEND_DATA;
        
        SEND_DATA: 
            if(data_ready) next_state = END;
        
        DATA_THROUGH: 
            if(data_through_finish) next_state = END2;
        
        END2: 
            if(en) next_state = INIT;
            else next_state = IDLE;
    endcase
end

// ████�?数据通道控制（对应架构图�?
always_ff @(posedge clk) begin
    if(!rst_n) begin
        a_reg            <= '0;
        b_reg            <= '0;
        partial_sum_reg  <= '0;
        sum_right        <= '0;
        data_ready       <= '0;
        a_out            <= '0;
        b_out            <= '0;
    end else begin
        case(current_state)
            INIT: begin
                a_reg <= a_in;
                b_reg <= b_in;
            end
            MUL: if(mul_finish) partial_sum_reg <= partial_sum;
            ADD: if(add_finish) sum_right <= sum_temp;
            SEND_DATA: begin
                data_ready <= 1'b1;
                a_out <= a_reg;
                b_out <= b_reg;
            end
            DATA_THROUGH: begin
                a_out <= a_in;
                b_out <= b_in;
                sum_right <= sum_down;
            end
            default: data_ready <= 1'b0;
        endcase
    end
end


// ████�?控制信号生成（精确时序控制）
assign mul_start = (current_state == MUL) && mul_pending;
assign add_start = (current_state == ADD) && add_pending;


endmodule






