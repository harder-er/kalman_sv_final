`timescale 1ns / 1ps

module StateCovarainceOutput #(
    parameter int STATE_DIM   = 12,
    parameter int MEASURE_DIM = 6,
    parameter int DATA_WIDTH  = 64,
    parameter int MAX_ITER    = 50
)(
    input  logic clk,
    input  logic rst_n,
    
    // Control & Data In
    input  logic en_sco,                              // Enable from FSM (en_sco in S_SCO state)
    input  logic [DATA_WIDTH-1:0] P_kk [STATE_DIM-1:0][STATE_DIM-1:0],  // Covariance matrix
    input  logic [DATA_WIDTH-1:0] X_k1k [STATE_DIM-1:0],                // State vector
    
    // AXI Write Interface (512-bit bus = 8×64-bit)
    output logic [31:0]   axi_awaddr,
    output logic [7:0]    axi_awlen,
    output logic [2:0]    axi_awsize,
    output logic [1:0]    axi_awburst,
    output logic          axi_awvalid,
    input  logic          axi_awready,
    
    output logic [511:0]  axi_wdata,
    output logic [63:0]   axi_wstrb,
    output logic          axi_wvalid,
    input  logic          axi_wready,
    output logic          axi_wlast,
    
    input  logic [1:0]    axi_bresp,
    input  logic          axi_bvalid,
    output logic          axi_bready,
    
    // Control Output
    output logic          sco_done,      // Write transaction complete
    output logic [7:0]    iteration_out, // Current iteration (0-49)
    output logic          all_done       // All 50 iterations complete
);

    // ====================================================================
    // State Machine: Data Sequencing for P_kk (144 elements) + X_k1k (12 elements)
    // ====================================================================
    
    typedef enum logic [3:0] {
        S_IDLE    = 4'd0,    // Waiting for en_sco
        S_P_WRITE = 4'd1,    // Writing P_kk (144 elements �?need ceil(144/8)=18 beats)
        S_X_WRITE = 4'd2,    // Writing X_k1k (12 elements �?need ceil(12/8)=2 beats)
        S_B_WAIT  = 4'd3,    // Waiting for AXI write response
        S_DONE    = 4'd4     // Iteration done, ready for next
    } state_t;
    
    state_t current_state, next_state;
    
    // ====================================================================
    // Iteration Counter (0-49)
    // ====================================================================
    logic [7:0] iteration_cnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            iteration_cnt <= 8'd0;
        end else begin
            if (next_state == S_IDLE && current_state == S_DONE) begin
                if (iteration_cnt == 8'd49) begin
                    iteration_cnt <= 8'd0;  // Reset for next batch (or wrap)
                end else begin
                    iteration_cnt <= iteration_cnt + 1'b1;
                end
            end
        end
    end
    
    assign iteration_out = iteration_cnt;
    assign all_done = (iteration_cnt == 8'd49) && (next_state == S_IDLE && current_state == S_DONE);
    
    // ====================================================================
    // Data Element Counter & Beat Counter
    // ====================================================================
    // Total elements: 144 (P_kk) + 12 (X_k1k) = 156 elements
    // With 512-bit bus = 8 elements per beat
    // P_kk needs: ceil(144/8) = 18 beats
    // X_k1k needs: ceil(12/8) = 2 beats
    // Total: 20 beats
    
    localparam int P_BEATS = (STATE_DIM * STATE_DIM + 7) / 8;  // 18 beats
    localparam int X_BEATS = (STATE_DIM + 7) / 8;              // 2 beats
    localparam int TOTAL_BEATS = P_BEATS + X_BEATS;             // 20 beats
    
    logic [7:0] beat_cnt;
    logic [3:0] elem_in_beat;  // Element index within current beat (0-7)
    
    // ====================================================================
    // Data muxing: P_kk then X_k1k
    // ====================================================================
    logic [DATA_WIDTH-1:0] data_to_write [7:0];
    
    always_comb begin
        for (int b = 0; b < 8; b++) begin
            if (beat_cnt < P_BEATS) begin
                // P_kk phase
                automatic int elem_idx = beat_cnt * 8 + b;
                automatic int row = elem_idx / STATE_DIM;
                automatic int col = elem_idx % STATE_DIM;
                data_to_write[b] = (elem_idx < STATE_DIM * STATE_DIM) ? P_kk[row][col] : 64'h0;
            end else begin
                // X_k1k phase
                automatic int elem_idx = (beat_cnt - P_BEATS) * 8 + b;
                data_to_write[b] = (elem_idx < STATE_DIM) ? X_k1k[elem_idx] : 64'h0;
            end
        end
    end
    
    // Concatenate 8×64-bit values into 512-bit output
    always_comb begin
        axi_wdata = {data_to_write[7], data_to_write[6], data_to_write[5], data_to_write[4],
                     data_to_write[3], data_to_write[2], data_to_write[1], data_to_write[0]};
    end
    
    // All bytes valid (full 512-bit writes)
    assign axi_wstrb = 64'hFFFFFFFFFFFFFFFF;
    
    // ====================================================================
    // AXI Address Generation
    // Base address: 0x0050_0000
    // Offset per iteration: 156 elements × 8 bytes / 8 elements per beat = 20 beats × 64 bytes = 1280 bytes
    // ====================================================================
    localparam int BYTES_PER_ITER = TOTAL_BEATS * 64;  // 1280 bytes
    
    logic [31:0] iter_base_addr;
    
    always_comb begin
        // Base address for this iteration
        iter_base_addr = 32'h0050_0000 + (iteration_cnt * BYTES_PER_ITER);
        
        // Current beat address (each beat = 64 bytes)
        axi_awaddr = iter_base_addr + (beat_cnt << 6);  // beat_cnt * 64
    end
    
    // ====================================================================
    // AXI Write Address & Data Handshake
    // ====================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= S_IDLE;
            beat_cnt      <= 8'd0;
        end else begin
            current_state <= next_state;
            
            // Increment beat counter on AXI write data handshake
            if (axi_wvalid && axi_wready) begin
                if (beat_cnt == TOTAL_BEATS - 1) begin
                    beat_cnt <= 8'd0;
                end else begin
                    beat_cnt <= beat_cnt + 1'b1;
                end
            end
        end
    end
    
    // ====================================================================
    // FSM: State Transitions
    // ====================================================================
    always_comb begin
        next_state = current_state;
        
        unique case (current_state)
            S_IDLE: begin
                if (en_sco)
                    next_state = S_P_WRITE;
            end
            
            S_P_WRITE, S_X_WRITE: begin
                // Stay in write state until all beats transferred
                if (axi_wvalid && axi_wready && beat_cnt == TOTAL_BEATS - 1) begin
                    next_state = S_B_WAIT;
                end
            end
            
            S_B_WAIT: begin
                // Wait for AXI write response (B channel)
                if (axi_bvalid)
                    next_state = S_DONE;
            end
            
            S_DONE: begin
                // Return to IDLE, iteration counter increments
                next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // ====================================================================
    // AXI Output Control
    // ====================================================================
    always_comb begin
        // Default: no transactions
        axi_awvalid = 1'b0;
        axi_wvalid  = 1'b0;
        axi_wlast   = 1'b0;
        axi_bready  = 1'b0;
        sco_done    = 1'b0;
        
        case (current_state)
            S_P_WRITE, S_X_WRITE: begin
                // Start write address transaction on first beat
                if (beat_cnt == 8'd0)
                    axi_awvalid = 1'b1;
                
                // Write data always available
                axi_wvalid = 1'b1;
                axi_wlast  = (beat_cnt == TOTAL_BEATS - 1) ? 1'b1 : 1'b0;
            end
            
            S_B_WAIT: begin
                // Accept write response
                axi_bready = 1'b1;
            end
            
            S_DONE: begin
                // Signal completion
                sco_done = 1'b1;
            end
        endcase
    end

endmodule
