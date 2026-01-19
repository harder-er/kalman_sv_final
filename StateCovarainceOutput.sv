`timescale 1ns / 1ps

module StateCovarainceOutput #(
    parameter int  STATE_DIM   = 12,
    parameter int  MEASURE_DIM = 6,    // 目前未使用，保留接口一致性
    parameter int  DATA_WIDTH  = 64,
    parameter int  MAX_ITER    = 50,

    // 输出写回内存基地址
    parameter logic [31:0] BASE_ADDR = 32'h0050_0000,

    // 每次迭代在内存中的起始地址步长（字节）
    // 0 => 默认紧凑存储 = ITER_BYTES (= TOTAL_BEATS*64 = 1280B)
    parameter int  ITER_STRIDE_BYTES = 0
)(
    input  logic clk,
    input  logic rst_n,

    // Control & Data In
    input  logic en_sco,
    input  logic [DATA_WIDTH-1:0] P_kk  [STATE_DIM-1:0][STATE_DIM-1:0],
    input  logic [DATA_WIDTH-1:0] X_k1k [STATE_DIM-1:0],

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
    output logic          sco_done,
    output logic [7:0]    iteration_out,
    output logic          all_done
);

    // ------------------------------------------------------------
    // 常量/参数派生
    // ------------------------------------------------------------
    localparam int BUS_BYTES      = 64;  // 512-bit
    localparam int ELEMS_PER_BEAT = 8;   // 8×64-bit
    localparam int P_ELEMS        = STATE_DIM * STATE_DIM; // 144
    localparam int X_ELEMS        = STATE_DIM;             // 12
    localparam int P_BEATS        = (P_ELEMS + ELEMS_PER_BEAT - 1) / ELEMS_PER_BEAT; // 18
    localparam int X_BEATS        = (X_ELEMS + ELEMS_PER_BEAT - 1) / ELEMS_PER_BEAT; // 2
    localparam int TOTAL_BEATS    = P_BEATS + X_BEATS; // 20
    localparam int ITER_BYTES     = TOTAL_BEATS * BUS_BYTES; // 1280

    localparam int STRIDE_BYTES   = (ITER_STRIDE_BYTES == 0) ? ITER_BYTES : ITER_STRIDE_BYTES;
    localparam int MAX_ITER_M1    = (MAX_ITER <= 0) ? 0 : (MAX_ITER - 1);

    // AXI 固定字段
    assign axi_awsize  = 3'b110; // 64 bytes/beat
    assign axi_awburst = 2'b01;  // INCR burst

    // 默认全写（包含 padding 的 0 也写进去，便于软件读取一致）
    assign axi_wstrb = 64'hFFFF_FFFF_FFFF_FFFF;

    // ------------------------------------------------------------
    // en_sco 上升沿作为启动脉冲，避免电平重复触发
    // ------------------------------------------------------------
    logic en_sco_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) en_sco_d <= 1'b0;
        else        en_sco_d <= en_sco;
    end
    wire start_pulse = en_sco & ~en_sco_d;

    // ------------------------------------------------------------
    // FSM：支持最多 2 个 burst（用于避免跨 4KB）
    // ------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE   = 2'd0,
        S_WRITE  = 2'd1,
        S_B_WAIT = 2'd2,
        S_DONE   = 2'd3
    } state_t;

    state_t st, st_n;

    // ------------------------------------------------------------
    // 迭代计数（0..MAX_ITER-1）
    // ------------------------------------------------------------
    logic [7:0] iteration_cnt;
    assign iteration_out = iteration_cnt;
    assign all_done      = (st == S_DONE) && (iteration_cnt == MAX_ITER_M1);

    // ------------------------------------------------------------
    // 当前迭代基地址（锁存），burst 拆分信息（锁存）
    // ------------------------------------------------------------
    logic [31:0] iter_base_addr_r;

    logic        has_burst1_r;         // 是否需要第二个 burst
    logic [7:0]  burst0_beats_r;       // 第一个 burst beats 数
    logic [7:0]  burst1_beats_r;       // 第二个 burst beats 数
    logic        burst_sel_r;          // 0: burst0, 1: burst1

    // burst 内 beat 计数
    logic [7:0]  beat_in_burst_r;
    logic        aw_sent_r;

    // 当前 burst beats / addr
    logic [7:0]  cur_burst_beats;
    logic [31:0] cur_burst_addr;
    logic [7:0]  global_beat; // 用于数据 mux（跨 burst 连续）

    always_comb begin
        cur_burst_beats = (burst_sel_r) ? burst1_beats_r : burst0_beats_r;
        cur_burst_addr  = iter_base_addr_r + (burst_sel_r ? ( {burst0_beats_r, 6'b0} ) : 32'd0);
        global_beat     = (burst_sel_r ? burst0_beats_r : 8'd0) + beat_in_burst_r;
    end

    // AW 信号（每个 burst 发送一次）
    always_comb begin
        axi_awaddr = cur_burst_addr;
        axi_awlen  = (cur_burst_beats == 0) ? 8'd0 : (cur_burst_beats - 1); // AxLEN+1 = beats
    end

    // ------------------------------------------------------------
    // 数据 mux：先 P_kk (144) 再 X_k1k (12)，按 global_beat 顺序打包
    // ------------------------------------------------------------
    logic [DATA_WIDTH-1:0] data_to_write [0:7];

    always_comb begin
        for (int b = 0; b < 8; b++) begin
            int elem_idx;
            int row, col;

            if (global_beat < P_BEATS) begin
                // P_kk
                elem_idx = global_beat * 8 + b; // 0..143
                row      = elem_idx / STATE_DIM;
                col      = elem_idx % STATE_DIM;
                data_to_write[b] = (elem_idx < P_ELEMS) ? P_kk[row][col] : '0;
            end else begin
                // X_k1k
                elem_idx = (global_beat - P_BEATS) * 8 + b; // 0..15
                data_to_write[b] = (elem_idx < X_ELEMS) ? X_k1k[elem_idx] : '0;
            end
        end
    end

    always_comb begin
        axi_wdata = {data_to_write[7], data_to_write[6], data_to_write[5], data_to_write[4],
                     data_to_write[3], data_to_write[2], data_to_write[1], data_to_write[0]};
    end

    // ------------------------------------------------------------
    // FSM next-state
    // ------------------------------------------------------------
    always_comb begin
        st_n = st;

        unique case (st)
            S_IDLE: begin
                if (start_pulse) st_n = S_WRITE;
            end

            S_WRITE: begin
                // 发送完当前 burst 的最后一个 beat（W 握手）=> 等 B
                if (aw_sent_r && axi_wvalid && axi_wready && (beat_in_burst_r == (cur_burst_beats - 1))) begin
                    st_n = S_B_WAIT;
                end
            end

            S_B_WAIT: begin
                if (axi_bvalid) begin
                    if (has_burst1_r && !burst_sel_r)
                        st_n = S_WRITE; // 进入 burst1
                    else
                        st_n = S_DONE;
                end
            end

            S_DONE: begin
                st_n = S_IDLE;
            end

            default: st_n = S_IDLE;
        endcase
    end

    // ------------------------------------------------------------
    // 顺序逻辑：状态/计数/拆分计算
    // ------------------------------------------------------------
    logic [31:0] iter_base_addr_calc;

    always_comb begin
        // 当前 iteration 的起始地址（紧凑布局时 STRIDE_BYTES=1280）
        iter_base_addr_calc = BASE_ADDR + (iteration_cnt * STRIDE_BYTES);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st              <= S_IDLE;
            iteration_cnt   <= 8'd0;

            iter_base_addr_r<= BASE_ADDR;

            has_burst1_r    <= 1'b0;
            burst0_beats_r  <= 8'd0;
            burst1_beats_r  <= 8'd0;
            burst_sel_r     <= 1'b0;

            beat_in_burst_r <= 8'd0;
            aw_sent_r       <= 1'b0;

        end else begin
            st <= st_n;

            // 默认不改
            // --------------------------------------------------------
            // 进入一次新的输出：锁存 base addr，并计算是否跨 4KB
            // --------------------------------------------------------
            if (st == S_IDLE && st_n == S_WRITE) begin
                int unsigned offset_4k;
                int unsigned bytes_to_4k;
                int unsigned b0_beats;

                iter_base_addr_r <= iter_base_addr_calc;

                offset_4k  = iter_base_addr_calc[11:0];        // within 4KB page
                // bytes_to_4k = 12'd4096 - offset_4k;
                bytes_to_4k = 32'd4096 - offset_4k;

                // 默认：不拆分
                has_burst1_r   <= 1'b0;
                burst0_beats_r <= TOTAL_BEATS[7:0];
                burst1_beats_r <= 8'd0;

                // 若本次 1280B burst 会跨 4KB，则拆成两段
                if ((offset_4k + ITER_BYTES) > 4096) begin
                    b0_beats = (bytes_to_4k + BUS_BYTES - 1) / BUS_BYTES;        // 第一段 beat 数
                    has_burst1_r   <= 1'b1;
                    burst0_beats_r <= b0_beats[7:0];
                    burst1_beats_r <= (TOTAL_BEATS[7:0] - b0_beats[7:0]);
                end

                burst_sel_r     <= 1'b0;
                beat_in_burst_r <= 8'd0;
                aw_sent_r       <= 1'b0;
            end

            // --------------------------------------------------------
            // AW 握手（每个 burst 1 次）
            // --------------------------------------------------------
            if (st == S_WRITE && !aw_sent_r) begin
                if (axi_awvalid && axi_awready) begin
                    aw_sent_r <= 1'b1;
                end
            end

            // --------------------------------------------------------
            // W 握手（每 beat）
            // --------------------------------------------------------
            if (st == S_WRITE && aw_sent_r) begin
                if (axi_wvalid && axi_wready) begin
                    if (beat_in_burst_r == (cur_burst_beats - 1)) begin
                        // burst 结束，等待 B
                        beat_in_burst_r <= beat_in_burst_r; // 保持（也可清零）
                    end else begin
                        beat_in_burst_r <= beat_in_burst_r + 1'b1;
                    end
                end
            end

            // --------------------------------------------------------
            // B 握手：若需要 burst1，则切换 burst_sel 并重置 burst 内计数/aw_sent
            // --------------------------------------------------------
            if (st == S_B_WAIT) begin
                if (axi_bvalid && axi_bready) begin
                    if (has_burst1_r && !burst_sel_r) begin
                        burst_sel_r     <= 1'b1;   // 切到 burst1
                        beat_in_burst_r <= 8'd0;
                        aw_sent_r       <= 1'b0;   // burst1 重新发 AW
                    end
                end
            end

            // --------------------------------------------------------
            // DONE：一次迭代完成，iteration_cnt 自增（或回卷）
            // --------------------------------------------------------
            if (st == S_DONE) begin
                if (iteration_cnt == MAX_ITER_M1[7:0])
                    iteration_cnt <= 8'd0;
                else
                    iteration_cnt <= iteration_cnt + 1'b1;

                // 下次迭代重新从 burst0 开始
                burst_sel_r <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------
    // AXI 输出控制
    // ------------------------------------------------------------
    always_comb begin
        axi_awvalid = 1'b0;
        axi_wvalid  = 1'b0;
        axi_wlast   = 1'b0;
        axi_bready  = 1'b0;

        sco_done    = 1'b0;

        unique case (st)
            S_WRITE: begin
                axi_awvalid = ~aw_sent_r;
                axi_wvalid  = aw_sent_r;

                if (aw_sent_r) begin
                    axi_wlast = (beat_in_burst_r == (cur_burst_beats - 1));
                end
            end

            S_B_WAIT: begin
                axi_bready = 1'b1;
            end

            S_DONE: begin
                sco_done = 1'b1;
            end

            default: ;
        endcase
    end

endmodule
