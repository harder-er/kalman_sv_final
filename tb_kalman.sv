`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2025/12/30 20:58:07
// Design Name:
// Module Name: tb_kalman_bd
// Project Name:
// Target Devices:
// Tool Versions:
// Description: Testbench for KalmanFilterTop_bd
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module tb_kalman_bd;

    // --------------------------------------------------------
    // 1) Parameters
    // --------------------------------------------------------
    parameter int CLK_PERIOD  = 10; // 100MHz

    // DDR memory base addresses
    parameter logic [31:0] ADDR_X00_BASE   = 32'h0030_0000;
    parameter logic [31:0] ADDR_P00_BASE   = 32'h0040_0000;
    parameter logic [31:0] ADDR_X_RES_BASE = 32'h0050_0000;
    parameter logic [31:0] ADDR_ZK_BASE    = 32'h0070_0000;

    // --------------------------------------------------------
    // 2) Clock / Reset / Control
    // --------------------------------------------------------
    logic clk, rst_n, start;
    wire  filter_done;
    logic filter_done_d;
    wire  filter_done_pulse = filter_done & ~filter_done_d;

    // --------------------------------------------------------
    // 3) AXI Interface Signals for KalmanFilterTop_bd
    // --------------------------------------------------------
    // m0 read
    wire [31:0]  m0_axi_araddr;
    wire [7:0]   m0_axi_arlen;
    wire [2:0]   m0_axi_arsize;
    wire [1:0]   m0_axi_arburst;
    wire         m0_axi_arvalid;
    wire         m0_axi_arready;
    wire [511:0] m0_axi_rdata;
    wire         m0_axi_rvalid;
    wire         m0_axi_rready;

    // m1 read
    wire [31:0]  m1_axi_araddr;
    wire [7:0]   m1_axi_arlen;
    wire [2:0]   m1_axi_arsize;
    wire [1:0]   m1_axi_arburst;
    wire         m1_axi_arvalid;
    wire         m1_axi_arready;
    wire [511:0] m1_axi_rdata;
    wire         m1_axi_rvalid;
    wire         m1_axi_rready;

    // m2 write
    wire [31:0]  m2_axi_awaddr;
    wire [7:0]   m2_axi_awlen;
    wire [2:0]   m2_axi_awsize;
    wire [1:0]   m2_axi_awburst;
    wire         m2_axi_awvalid;
    wire         m2_axi_awready;
    wire [511:0] m2_axi_wdata;
    wire [63:0]  m2_axi_wstrb;
    wire         m2_axi_wvalid;
    wire         m2_axi_wready;
    wire         m2_axi_wlast;
    wire [1:0]   m2_axi_bresp;
    wire         m2_axi_bvalid;
    wire         m2_axi_bready;

    // --------------------------------------------------------
    // 4) DUT: KalmanFilterTop_bd instance
    // --------------------------------------------------------
    KalmanFilterTop_bd dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .filter_done(filter_done),

        // m0 read
        .m0_axi_araddr (m0_axi_araddr),
        .m0_axi_arlen  (m0_axi_arlen),
        .m0_axi_arsize (m0_axi_arsize),
        .m0_axi_arburst(m0_axi_arburst),
        .m0_axi_arvalid(m0_axi_arvalid),
        .m0_axi_arready(m0_axi_arready),
        .m0_axi_rdata  (m0_axi_rdata),
        .m0_axi_rvalid (m0_axi_rvalid),
        .m0_axi_rready (m0_axi_rready),

        // m1 read
        .m1_axi_araddr (m1_axi_araddr),
        .m1_axi_arlen  (m1_axi_arlen),
        .m1_axi_arsize (m1_axi_arsize),
        .m1_axi_arburst(m1_axi_arburst),
        .m1_axi_arvalid(m1_axi_arvalid),
        .m1_axi_arready(m1_axi_arready),
        .m1_axi_rdata  (m1_axi_rdata),
        .m1_axi_rvalid (m1_axi_rvalid),
        .m1_axi_rready (m1_axi_rready),

        // m2 write
        .m2_axi_awaddr (m2_axi_awaddr),
        .m2_axi_awlen  (m2_axi_awlen),
        .m2_axi_awsize (m2_axi_awsize),
        .m2_axi_awburst(m2_axi_awburst),
        .m2_axi_awvalid(m2_axi_awvalid),
        .m2_axi_awready(m2_axi_awready),
        .m2_axi_wdata  (m2_axi_wdata),
        .m2_axi_wstrb  (m2_axi_wstrb),
        .m2_axi_wvalid (m2_axi_wvalid),
        .m2_axi_wready (m2_axi_wready),
        .m2_axi_wlast  (m2_axi_wlast),
        .m2_axi_bresp  (m2_axi_bresp),
        .m2_axi_bvalid (m2_axi_bvalid),
        .m2_axi_bready (m2_axi_bready)
    );

    // --------------------------------------------------------
    // 5) AXI4 Memory Models: m0/m1 read-only; m2 write-only
    // --------------------------------------------------------

    // m0 read-only memory model
    axi4_mem_slave #(.ADDR_WIDTH(32), .DATA_WIDTH(512)) ddr_m0 (
        .aclk(clk),
        .aresetn(rst_n),

        // write unused
        .s_axi_awaddr ('0),
        .s_axi_awlen  ('0),
        .s_axi_awvalid(1'b0),
        .s_axi_awready(),
        .s_axi_wdata  ('0),
        .s_axi_wstrb  ('0),
        .s_axi_wlast  (1'b0),
        .s_axi_wvalid (1'b0),
        .s_axi_wready (),
        .s_axi_bresp  (),
        .s_axi_bvalid (),
        .s_axi_bready (1'b0),

        // read
        .s_axi_araddr (m0_axi_araddr),
        .s_axi_arlen  (m0_axi_arlen),
        .s_axi_arvalid(m0_axi_arvalid),
        .s_axi_arready(m0_axi_arready),
        .s_axi_rdata  (m0_axi_rdata),
        .s_axi_rresp  (),
        .s_axi_rlast  (),
        .s_axi_rvalid (m0_axi_rvalid),
        .s_axi_rready (m0_axi_rready)
    );

    // m1 read-only memory model
    axi4_mem_slave #(.ADDR_WIDTH(32), .DATA_WIDTH(512)) ddr_m1 (
        .aclk(clk),
        .aresetn(rst_n),

        // write unused
        .s_axi_awaddr ('0),
        .s_axi_awlen  ('0),
        .s_axi_awvalid(1'b0),
        .s_axi_awready(),
        .s_axi_wdata  ('0),
        .s_axi_wstrb  ('0),
        .s_axi_wlast  (1'b0),
        .s_axi_wvalid (1'b0),
        .s_axi_wready (),
        .s_axi_bresp  (),
        .s_axi_bvalid (),
        .s_axi_bready (1'b0),

        // read
        .s_axi_araddr (m1_axi_araddr),
        .s_axi_arlen  (m1_axi_arlen),
        .s_axi_arvalid(m1_axi_arvalid),
        .s_axi_arready(m1_axi_arready),
        .s_axi_rdata  (m1_axi_rdata),
        .s_axi_rresp  (),
        .s_axi_rlast  (),
        .s_axi_rvalid (m1_axi_rvalid),
        .s_axi_rready (m1_axi_rready)
    );

    // m2 write-only memory model
    axi4_mem_slave #(.ADDR_WIDTH(32), .DATA_WIDTH(512)) ddr_m2 (
        .aclk(clk),
        .aresetn(rst_n),

        // write
        .s_axi_awaddr (m2_axi_awaddr),
        .s_axi_awlen  (m2_axi_awlen),
        .s_axi_awvalid(m2_axi_awvalid),
        .s_axi_awready(m2_axi_awready),
        .s_axi_wdata  (m2_axi_wdata),
        .s_axi_wstrb  (m2_axi_wstrb),
        .s_axi_wlast  (m2_axi_wlast),
        .s_axi_wvalid (m2_axi_wvalid),
        .s_axi_wready (m2_axi_wready),
        .s_axi_bresp  (m2_axi_bresp),
        .s_axi_bvalid (m2_axi_bvalid),
        .s_axi_bready (m2_axi_bready),

        // read unused
        .s_axi_araddr ('0),
        .s_axi_arlen  ('0),
        .s_axi_arvalid(1'b0),
        .s_axi_arready(),
        .s_axi_rdata  (),
        .s_axi_rresp  (),
        .s_axi_rlast  (),
        .s_axi_rvalid (),
        .s_axi_rready (1'b0)
    );

    // --------------------------------------------------------
    // 6) Clock Generation
    // --------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // --------------------------------------------------------
    // 7) Test Sequence
    // --------------------------------------------------------
    initial begin
        // IMPORTANT: Declarations must appear before any procedural statements
        // to avoid tool parsing errors.  (SystemVerilog rule)
        string       x_init_file, p_init_file, z_meas_file, out_file;
        string       golden_file;
        string       tb_dir;
        int unsigned timeout_cycles;
        int unsigned start_delay_cycles;
        int unsigned b_wait_cycles;
        int          plus_ok; // for capturing $value$plusargs return value

        // defaults
        tb_dir             = dirname(`__FILE__);
        x_init_file        = {tb_dir, "/x_init.hex"};
        p_init_file        = {tb_dir, "/p_init.hex"};
        z_meas_file        = {tb_dir, "/z_meas.hex"};
        out_file           = {tb_dir, "/fpga_output_x.hex"};
        golden_file        = {tb_dir, "/x_golden.hex"};
        timeout_cycles     = 2_000_000_00;
		//timeout_cycles     = 2_000;
        start_delay_cycles = 100;
        b_wait_cycles      = 100000;

        // Read plusargs (accept user overrides)
        plus_ok = $value$plusargs("X_INIT=%s",      x_init_file);
        plus_ok = $value$plusargs("P_INIT=%s",      p_init_file);
        plus_ok = $value$plusargs("Z_MEAS=%s",      z_meas_file);
        plus_ok = $value$plusargs("OUT=%s",         out_file);
        plus_ok = $value$plusargs("TIMEOUT=%d",     timeout_cycles);
        plus_ok = $value$plusargs("START_DELAY=%d", start_delay_cycles);
        plus_ok = $value$plusargs("GOLDEN=%s",      golden_file);
        plus_ok = $value$plusargs("BWAIT=%d",       b_wait_cycles);

        rst_n = 1'b0;
        start = 1'b0;

        // reset
        repeat (50) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        // Load DDR via backdoor
        $display("[%0t] Loading Memory...", $time);
        load_file_to_mem(x_init_file, ADDR_X00_BASE);
        load_file_to_mem(p_init_file, ADDR_P00_BASE);
        load_file_to_mem(z_meas_file, ADDR_ZK_BASE);

        // Start filtering after delay
        $display("[%0t] Wait %0d cycles before start ...", $time, start_delay_cycles);
        repeat (start_delay_cycles) @(posedge clk);
        start = 1'b1;
        @(posedge clk);
        start = 1'b0;
        $display("[%0t] Filter Started...", $time);

        // Wait done / timeout
        fork
            begin
                wait (filter_done === 1'b1);
                $display("[%0t] Filter Done!", $time);

                wait_m2_write_resp(b_wait_cycles);
                report_metrics_to_console();
                compare_x_rmse_file(golden_file, ADDR_X_RES_BASE, 12);
            end
            begin
                repeat (timeout_cycles) @(posedge clk);
                $display("[%0t] ERROR: Simulation Timeout!", $time);
					$finish;
            end
        join_any
        disable fork;

        // Dump results
        repeat (50) @(posedge clk);
        $display("[%0t] Dumping results to %s", $time, out_file);
        dump_mem_to_file(out_file, ADDR_X_RES_BASE, 100);

        $display("[%0t] Testbench Completed.", $time);
        $finish;
    end

    // --------------------------------------------------------
    // 8) Backdoor Dispatch
    // --------------------------------------------------------
    task automatic backdoor_write64_dispatch(input logic [31:0] addr, input logic [63:0] val);
        if (addr >= ADDR_X_RES_BASE && addr < ADDR_ZK_BASE) begin
            ddr_m2.backdoor_write64(addr, val);
        end else if (addr >= ADDR_ZK_BASE) begin
            ddr_m1.backdoor_write64(addr, val);
        end else begin
            ddr_m0.backdoor_write64(addr, val);
        end
    endtask

    task automatic backdoor_read64_dispatch(input logic [31:0] addr, output logic [63:0] val);
        if (addr >= ADDR_X_RES_BASE && addr < ADDR_ZK_BASE) begin
            ddr_m2.backdoor_read64(addr, val);
        end else if (addr >= ADDR_ZK_BASE) begin
            ddr_m1.backdoor_read64(addr, val);
        end else begin
            ddr_m0.backdoor_read64(addr, val);
        end
    endtask

    // --------------------------------------------------------
    // 9) File I/O Tasks
    // --------------------------------------------------------
    task automatic load_file_to_mem(input string filename, input logic [31:0] base_addr);
        integer fd, code;
        logic [63:0] val;
        logic [31:0] curr_addr;

        fd = $fopen(filename, "r");
        if (fd == 0) begin
            $display("[%0t] WARNING: Could not open %s (skip).", $time, filename);
        end else begin
            curr_addr = base_addr;
            while (!$feof(fd)) begin
                code = $fscanf(fd, "%h", val);
                if (code == 1) begin
                    backdoor_write64_dispatch(curr_addr, val);
                    curr_addr += 32'd8;
                end
            end
            $fclose(fd);
            $display("[%0t] Loaded %s to base 0x%08h", $time, filename, base_addr);
        end
    endtask

    task automatic dump_mem_to_file(input string filename, input logic [31:0] base_addr, input int num_words);
        integer fd;
        logic [63:0] val;
        logic [31:0] curr_addr;

        fd = $fopen(filename, "w");
        if (fd == 0) begin
            $display("[%0t] ERROR: Could not open output file %s", $time, filename);
            $stop;
        end

        curr_addr = base_addr;
        for (int i = 0; i < num_words; i++) begin
            backdoor_read64_dispatch(curr_addr, val);
            $fwrite(fd, "%h\n", val);
            curr_addr += 32'd8;
        end

        $fclose(fd);
        $display("[%0t] Dumped %0d words from 0x%08h to %s", $time, num_words, base_addr, filename);
    endtask

    // ============================================================
    // 10) Metrics: latency / AXI throughput / RMSE (golden compare)
    // ============================================================

    // --------------------
    // Cycle counter
    // --------------------
    longint unsigned cycle_cnt;
    always_ff @(posedge clk) begin
        if (!rst_n) cycle_cnt <= 0;
        else        cycle_cnt <= cycle_cnt + 1;
    end

    // --------------------
    // AXI handshake helpers
    // --------------------
    function automatic bit hs(input bit v, input bit r);
        return v && r;
    endfunction

    function automatic string dirname(input string path);
        int last;
        last = -1;
        for (int i = 0; i < path.len(); i++) begin
            if ((path[i] == 8'h2f) || (path[i] == 8'h5c))
                last = i;
        end
        if (last > 0) return path.substr(0, last-1);
        if (last == 0) return path.substr(0, 0);
        return ".";
    endfunction

    // --------------------
    // AXI counters
    // --------------------
    longint unsigned m0_ar_cnt, m0_rbeat_cnt;
    longint unsigned m1_ar_cnt, m1_rbeat_cnt;
    longint unsigned m2_aw_cnt, m2_wbeat_cnt, m2_b_cnt;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m0_ar_cnt    <= 0;  m0_rbeat_cnt <= 0;
            m1_ar_cnt    <= 0;  m1_rbeat_cnt <= 0;
            m2_aw_cnt    <= 0;  m2_wbeat_cnt <= 0;  m2_b_cnt <= 0;
        end else begin
            if (hs(m0_axi_arvalid, m0_axi_arready)) m0_ar_cnt++;
            if (hs(m0_axi_rvalid,  m0_axi_rready))  m0_rbeat_cnt++;
            if (hs(m1_axi_arvalid, m1_axi_arready)) m1_ar_cnt++;
            if (hs(m1_axi_rvalid,  m1_axi_rready))  m1_rbeat_cnt++;
            if (hs(m2_axi_awvalid, m2_axi_awready)) m2_aw_cnt++;
            if (hs(m2_axi_wvalid,  m2_axi_wready))  m2_wbeat_cnt++;
            if (hs(m2_axi_bvalid,  m2_axi_bready))  m2_b_cnt++;
        end
    end

    // 512-bit data bus => 64 bytes per beat
    localparam int AXI_BEAT_BYTES = 64;

    // -----------------------------------------------------------
    // Latency measurement
    // -----------------------------------------------------------
    longint unsigned start_cycle, done_cycle;
    bit started, done_seen;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            started     <= 1'b0;
            done_seen   <= 1'b0;
            start_cycle <= 0;
            done_cycle  <= 0;
            filter_done_d <= 1'b0;
        end else begin
            filter_done_d <= filter_done;
            if (!started && start) begin
                started     <= 1'b1;
                start_cycle <= cycle_cnt;
            end
            if (!done_seen && filter_done_pulse) begin
                done_seen  <= 1'b1;
                done_cycle <= cycle_cnt;
            end
        end
    end

    // -----------------------------------------------------------
    // RMSE compare (hex IEEE754 double per line)
    // -----------------------------------------------------------
    function automatic real bits64_to_real(input logic [63:0] bits);
        return $bitstoreal(bits);
    endfunction

    task automatic compare_x_rmse_file(
        input string golden_file,
        input logic [31:0] pred_base,
        input int n_words
    );
        integer fd, code;
        logic [63:0] gold_bits, pred_bits;
        real gold_r, pred_r, err, sse;
        int i;

        sse = 0.0;
        fd = $fopen(golden_file, "r");
        if (fd == 0) begin
            $display("[RMSE] ERROR: cannot open GOLDEN file: %s", golden_file);
            return;
        end

        for (i = 0; i < n_words; i++) begin
            code = $fscanf(fd, "%h", gold_bits);
            if (code != 1) begin
                $display("[RMSE] WARNING: GOLDEN file ended early at i=%0d", i);
                break;
            end

            // Read predicted result from DDR write-back region (m2)
            ddr_m2.backdoor_read64(pred_base + i*8, pred_bits);

            gold_r = bits64_to_real(gold_bits);
            pred_r = bits64_to_real(pred_bits);

            err = pred_r - gold_r;
            sse = sse + err * err;
        end

        $fclose(fd);

        if (i > 0) begin
            real rmse;
            rmse = $sqrt(sse / i);
            $display("[RMSE] words=%0d  SSE=%e  RMSE=%e  (golden=%s)",
                     i, sse, rmse, golden_file);
        end else begin
            $display("[RMSE] ERROR: no samples compared.");
        end
    endtask

    task automatic wait_m2_write_resp(input int max_cycles);
        int cnt;
        cnt = 0;
        while (cnt < max_cycles && (m2_aw_cnt != m2_b_cnt)) begin
            @(posedge clk);
            cnt++;
        end
        if (m2_aw_cnt != m2_b_cnt) begin
            $display("[WARN] m2 write responses pending: AW=%0d B=%0d after %0d cycles",
                     m2_aw_cnt, m2_b_cnt, max_cycles);
        end
    endtask

    // -----------------------------------------------------------
    // Report metrics task
    // -----------------------------------------------------------
    task automatic report_metrics_to_console();
        longint unsigned latency_cycles;
        longint unsigned rd_bytes, wr_bytes;

        latency_cycles = (done_seen) ? (done_cycle - start_cycle) : 0;

        rd_bytes = (m0_rbeat_cnt + m1_rbeat_cnt) * AXI_BEAT_BYTES;
        wr_bytes = (m2_wbeat_cnt) * AXI_BEAT_BYTES;

        $display("==================================================");
        $display("[METRICS] start_cycle=%0d done_cycle=%0d latency=%0d cycles",
                 start_cycle, done_cycle, latency_cycles);
        $display("[METRICS] m0: AR=%0d  Rbeats=%0d  (~%0d bytes)",
                 m0_ar_cnt, m0_rbeat_cnt, m0_rbeat_cnt*AXI_BEAT_BYTES);
        $display("[METRICS] m1: AR=%0d  Rbeats=%0d  (~%0d bytes)",
                 m1_ar_cnt, m1_rbeat_cnt, m1_rbeat_cnt*AXI_BEAT_BYTES);
        $display("[METRICS] m2: AW=%0d  Wbeats=%0d (~%0d bytes)  B=%0d",
                 m2_aw_cnt, m2_wbeat_cnt, wr_bytes, m2_b_cnt);
        $display("[METRICS] Total RD=%0d bytes, WR=%0d bytes", rd_bytes, wr_bytes);
        $display("==================================================");
    endtask

endmodule
