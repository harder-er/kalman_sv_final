`timescale 1ns / 1ps

module axi4_mem_slave #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 512  // 默认 512 bit (64 Bytes)
)(
    input  logic                      aclk,
    input  logic                      aresetn,

    // ---------------------------------------------------------
    // AXI4 Write Address Channel
    // ---------------------------------------------------------
    input  logic [ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  logic [7:0]                s_axi_awlen,
    input  logic                      s_axi_awvalid,
    output logic                      s_axi_awready,

    // ---------------------------------------------------------
    // AXI4 Write Data Channel
    // ---------------------------------------------------------
    input  logic [DATA_WIDTH-1:0]     s_axi_wdata,
    input  logic [DATA_WIDTH/8-1:0]   s_axi_wstrb,
    input  logic                      s_axi_wlast,
    input  logic                      s_axi_wvalid,
    output logic                      s_axi_wready,

    // ---------------------------------------------------------
    // AXI4 Write Response Channel
    // ---------------------------------------------------------
    output logic [1:0]                s_axi_bresp,
    output logic                      s_axi_bvalid,
    input  logic                      s_axi_bready,

    // ---------------------------------------------------------
    // AXI4 Read Address Channel
    // ---------------------------------------------------------
    input  logic [ADDR_WIDTH-1:0]     s_axi_araddr,
    input  logic [7:0]                s_axi_arlen,
    input  logic                      s_axi_arvalid,
    output logic                      s_axi_arready,

    // ---------------------------------------------------------
    // AXI4 Read Data Channel
    // ---------------------------------------------------------
    output logic [DATA_WIDTH-1:0]     s_axi_rdata,
    output logic [1:0]                s_axi_rresp,
    output logic                      s_axi_rlast,
    output logic                      s_axi_rvalid,
    input  logic                      s_axi_rready
);

    // -------------------------------------------------------------------------
    // 1) 稀疏存储器定义：关联数组模拟 DDR（Byte 寻址）
    // -------------------------------------------------------------------------
    logic [7:0] mem_store[logic [ADDR_WIDTH-1:0]];

    // -------------------------------------------------------------------------
    // 2) 写通道逻辑 (Write Channel)
    //   改进点：用 AWLEN 计数判定 burst 结束，避免 WLAST 出错导致死锁
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {W_IDLE, W_BURST, W_RESP} w_state_t;
    w_state_t                w_state;

    logic [ADDR_WIDTH-1:0]   w_addr_latch;
    logic [7:0]              w_len_latch;   // latch awlen
    logic [7:0]              w_cnt;         // beat counter

    // “按 AWLEN 推导的最后一拍”
    logic w_last_by_len;
    assign w_last_by_len = (w_cnt == w_len_latch);

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00; // OKAY

            w_state       <= W_IDLE;
            w_addr_latch  <= '0;
            w_len_latch   <= '0;
            w_cnt         <= '0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    s_axi_awready <= 1'b1;
                    s_axi_wready  <= 1'b0;
                    s_axi_bvalid  <= 1'b0;

                    if (s_axi_awvalid && s_axi_awready) begin
                        w_addr_latch  <= s_axi_awaddr;
                        w_len_latch   <= s_axi_awlen;   // 记住 burst len (beats-1)
                        w_cnt         <= 8'd0;

                        s_axi_awready <= 1'b0;
                        s_axi_wready  <= 1'b1;
                        w_state       <= W_BURST;
                    end
                end

                W_BURST: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        // 根据 WSTRB 按字节写
                        for (int i = 0; i < (DATA_WIDTH/8); i++) begin
                            if (s_axi_wstrb[i]) begin
                                mem_store[w_addr_latch + i[ADDR_WIDTH-1:0]]
                                    = s_axi_wdata[8*i +: 8];
                            end
                        end

                        // 如果 master 的 WLAST 与 AWLEN 推导不一致，给 warning（不终止仿真）
                        if ((s_axi_wlast === 1'b0 || s_axi_wlast === 1'b1) &&
                            (s_axi_wlast !== w_last_by_len)) begin
                            $display("[%0t] WARNING: AXI WLAST(%0b) != expected_last_by_AWLEN(%0b). awlen=%0d cnt=%0d",
                                     $time, s_axi_wlast, w_last_by_len, w_len_latch, w_cnt);
                        end

                        // 结束条件：
                        // 1) 正常情况：按 AWLEN 到头
                        // 2) 宽容情况：WLAST==1 也允许提前结束（但只认确定的 1）
                        if (w_last_by_len || (s_axi_wlast === 1'b1)) begin
                            s_axi_wready <= 1'b0;
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= 2'b00; // OKAY
                            w_state      <= W_RESP;
                        end else begin
                            // 下一拍
                            w_cnt        <= w_cnt + 1'b1;
                            w_addr_latch <= w_addr_latch + (DATA_WIDTH/8);
                        end
                    end
                end

                W_RESP: begin
                    if (s_axi_bvalid && s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        w_state      <= W_IDLE;
                    end
                end

                default: w_state <= W_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 3) 读通道逻辑 (Read Channel)
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {R_IDLE, R_BURST} r_state_t;
    r_state_t                r_state;

    logic [ADDR_WIDTH-1:0]   r_addr_latch;
    logic [7:0]              r_len_latch;
    logic [7:0]              r_cnt;

    assign s_axi_rresp = 2'b00; // 固定 OKAY

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rlast   <= 1'b0;

            r_state       <= R_IDLE;
            r_addr_latch  <= '0;
            r_len_latch   <= '0;
            r_cnt         <= '0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    s_axi_arready <= 1'b1;
                    s_axi_rvalid  <= 1'b0;
                    s_axi_rlast   <= 1'b0;

                    if (s_axi_arvalid && s_axi_arready) begin
                        r_addr_latch  <= s_axi_araddr;
                        r_len_latch   <= s_axi_arlen;
                        r_cnt         <= 8'd0;

                        s_axi_arready <= 1'b0;
                        r_state       <= R_BURST;
                    end
                end

                R_BURST: begin
                    if (!s_axi_rvalid) begin
                        s_axi_rvalid <= 1'b1;
                        s_axi_rlast  <= (r_cnt == r_len_latch);
                    end else if (s_axi_rvalid && s_axi_rready) begin
                        if (r_cnt == r_len_latch) begin
                            s_axi_rvalid <= 1'b0;
                            s_axi_rlast  <= 1'b0;
                            r_state      <= R_IDLE;
                        end else begin
                            r_cnt        <= r_cnt + 1'b1;
                            r_addr_latch <= r_addr_latch + (DATA_WIDTH/8);
                            s_axi_rlast  <= ((r_cnt + 1'b1) == r_len_latch);
                        end
                    end
                end

                default: r_state <= R_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 4) 读数据组合逻辑
    // -------------------------------------------------------------------------
    always_comb begin
        s_axi_rdata = '0;

        if (r_state == R_BURST) begin
            for (int i = 0; i < (DATA_WIDTH/8); i++) begin
                logic [ADDR_WIDTH-1:0] byte_addr;
                byte_addr = r_addr_latch + i[ADDR_WIDTH-1:0];

                if (mem_store.exists(byte_addr))
                    s_axi_rdata[8*i +: 8] = mem_store[byte_addr];
                else
                    s_axi_rdata[8*i +: 8] = 8'h00;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 5) Backdoor 任务
    // -------------------------------------------------------------------------
    task backdoor_write64(
        input logic [ADDR_WIDTH-1:0] addr,
        input logic [63:0]           data
    );
        for (int i = 0; i < 8; i++) begin
            mem_store[addr + i[ADDR_WIDTH-1:0]] = data[8*i +: 8];
        end
    endtask

    task backdoor_read64(
        input  logic [ADDR_WIDTH-1:0] addr,
        output logic [63:0]           data
    );
        for (int i = 0; i < 8; i++) begin
            logic [ADDR_WIDTH-1:0] byte_addr;
            byte_addr = addr + i[ADDR_WIDTH-1:0];

            if (mem_store.exists(byte_addr))
                data[8*i +: 8] = mem_store[byte_addr];
            else
                data[8*i +: 8] = 8'h00;
        end
    endtask

endmodule
