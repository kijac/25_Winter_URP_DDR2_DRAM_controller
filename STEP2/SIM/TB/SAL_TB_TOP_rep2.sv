`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

module SAL_TB_TOP;

    // =========================================================================
    // 1. Environment Setup
    // =========================================================================
    logic                       clk;
    logic                       rst_n;

    // Semaphores for Channel Serialization
    semaphore wr_sem = new(1);
    semaphore ar_sem = new(1);

    // Clock Generation
    initial begin
        clk = 1'b0;
        forever #(`CLK_PERIOD/2) clk = ~clk;
    end

    // Reset Generation
    initial begin
        rst_n = 1'b0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
    end

    // Interfaces
    APB_IF                          apb_if      (.clk(clk), .rst_n(rst_n));
    AXI_A_IF                        axi_ar_if   (.clk(clk), .rst_n(rst_n));
    AXI_R_IF                        axi_r_if    (.clk(clk), .rst_n(rst_n));
    AXI_A_IF                        axi_aw_if   (.clk(clk), .rst_n(rst_n));
    AXI_W_IF                        axi_w_if    (.clk(clk), .rst_n(rst_n));
    AXI_B_IF                        axi_b_if    (.clk(clk), .rst_n(rst_n));

    DFI_CTRL_IF                     dfi_ctrl_if (.clk(clk), .rst_n(rst_n));
    DFI_WR_IF                       dfi_wr_if   (.clk(clk), .rst_n(rst_n));
    DFI_RD_IF                       dfi_rd_if   (.clk(clk), .rst_n(rst_n));

    DDR_IF                          ddr_if      ();

    // DUT Instantiation
    SAL_DDR_CTRL                    u_dram_ctrl
    (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .apb_if                     (apb_if),
        .axi_ar_if                  (axi_ar_if),
        .axi_aw_if                  (axi_aw_if),
        .axi_w_if                   (axi_w_if),
        .axi_b_if                   (axi_b_if),
        .axi_r_if                   (axi_r_if),
        .dfi_ctrl_if                (dfi_ctrl_if),
        .dfi_wr_if                  (dfi_wr_if),
        .dfi_rd_if                  (dfi_rd_if)
    );

    DDRPHY                          u_ddrphy
    (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .dfi_ctrl_if                (dfi_ctrl_if),
        .dfi_wr_if                  (dfi_wr_if),
        .dfi_rd_if                  (dfi_rd_if),
        .ddr_if                     (ddr_if)
    );

    ddr2_dimm u_rank0 (.ddr_if(ddr_if), .cs_n(ddr_if.cs_n[0]));
    // ddr2_dimm u_rank1 (.ddr_if(ddr_if), .cs_n(ddr_if.cs_n[1]));

    // =========================================================================
    // 2. Stress Test Tasks
    // =========================================================================
    
    task init();
        axi_aw_if.init();
        axi_w_if.init();
        axi_b_if.init();
        axi_ar_if.init();
        axi_r_if.init();
        @(posedge rst_n);
        repeat (250) @(posedge clk); 
    endtask

    // --- Non-Blocking Issues (Serialized) ---
    task automatic fire_write(input axi_id_t id, input axi_addr_t addr);
        fork
            begin
                wr_sem.get();
                axi_aw_if.send(id, addr, 'd1, `AXI_SIZE_128, `AXI_BURST_INCR);
                axi_w_if.send(id, {16{8'hAA}}, 16'hFFFF, 1'b0);
                axi_w_if.send(id, {16{8'hBB}}, 16'hFFFF, 1'b1);
                wr_sem.put();
            end
        join_none
    endtask

    task automatic fire_read(input axi_id_t id, input axi_addr_t addr);
        fork 
            begin
                ar_sem.get();
                axi_ar_if.send(id, addr, 'd1, `AXI_SIZE_128, `AXI_BURST_INCR);
                ar_sem.put();
            end
        join_none
    endtask

    task automatic wait_write_resp(input int count);
        axi_id_t rid; axi_resp_t rresp;
        repeat(count) axi_b_if.recv(rid, rresp);
    endtask

    task automatic wait_read_resp(input int count);
        axi_id_t rid; axi_resp_t rresp; logic rlast; logic [0:127] d;
        repeat(count) begin
            axi_r_if.recv(rid, d, rresp, rlast); 
            axi_r_if.recv(rid, d, rresp, rlast); 
        end
    endtask

    // =========================================================================
    // 3. Main Test Sequence (Updated for Scheduler Performance)
    // =========================================================================
    real start_time, end_time;

    initial begin
        init();

        // ---------------------------------------------------------------------
        // 0. Pre-load Memory to prevent 'XX' Data on Reads
        // ---------------------------------------------------------------------
        $display("[Init] Pre-loading memory to avoid uninitialized reads...");
        
        // Addresses for Test D (Thread A Reads)
        fire_write(0, 'h0000); 
        fire_write(0, 'h8000); // Updated to match Row 1 access
        fire_write(0, 'h0010); 
        fire_write(0, 'h0020);

        // Addresses for Test E (Reads that don't match Writes)
        fire_write(0, 'h2000); 
        fire_write(0, 'h6000); 
        fire_write(0, 'hA000); 
        fire_write(0, 'hE000);

        // Wait for pre-load to finish and controller to drain
        wait_write_resp(8); 
        repeat(100) @(posedge clk); 
        $display("[Init] Memory Pre-load Complete.\n");


        $display("============================================================");
        $display("   DDR2 Stress Test v2 - Random & Conflict Scenarios        ");
        $display("============================================================");
        
        // ---------------------------------------------------------------------
        // SCENARIO D: Random Row & Bank Conflict
        // Target: Force Scheduler to reorder Row Hits vs Row Misses
        // ---------------------------------------------------------------------
        $display("\n[Test D] Random Traffic (Row Miss vs Row Hit)");
        repeat(20) @(posedge clk);
        start_time = $realtime;

        // Queue Pattern designed to stress the Scheduler Reordering Logic
        // We will send requests intentionally "Out of Order" relative to optimal execution
        // Expectation: Smart Scheduler should execute HITs first, regardless of arrival order.
        
        fork
            // Thread A: Sends 'MISS' followed by 'HITs' to Bank 0
            begin
                 // 1. Row 0 (Hit) -> 2. Row 5 (Miss) -> 3. Row 0 (Hit) -> 4. Row 0 (Hit)
                 // If FIFO: Hit -> Miss (Wait) -> Hit -> Hit (Inefficient)
                 // If Sched: Hit -> Hit -> Hit -> Miss (Efficient)
                 fire_read(0, 'h0000);   // Bank 0, Row 0 (Hit)
                 fire_read(1, 'h8000);   // Bank 0, Row 1 (Miss - Conflict)
                 fire_read(2, 'h0010);   // Bank 0, Row 0 (Hit - return)
                 fire_read(3, 'h0020);   // Bank 0, Row 0 (Hit)
            end

            // Thread B: Noise Traffic to other Banks 
            begin
                fire_write(4, 'h2000); // Bank 1
                fire_write(5, 'h4000); // Bank 2
                fire_write(6, 'h6000); // Bank 3
                fire_write(7, 'h10000); // Bank 0 (Row 2) - Noise
            end
        join

        fork
            wait_read_resp(4);
            wait_write_resp(4);
        join
        
        end_time = $realtime;
        $display("[Result D] Duration: %0f ps", end_time - start_time);


        // ---------------------------------------------------------------------
        // SCENARIO E: Heavy Random Saturation
        // Target: Fill all queues with random addresses. Pure Chaos.
        // ---------------------------------------------------------------------
        $display("\n[Test E] Heavy Saturation (32 Ops)");
        repeat(20) @(posedge clk);
        start_time = $realtime;

        fork
            begin
                repeat(4) begin
                    // Mix of Reads and Writes to random-ish addresses
                    fire_write(0, 'h0000); fire_read(1, 'h2000);
                    fire_write(2, 'h4000); fire_read(3, 'h6000);
                    fire_write(4, 'h8000); fire_read(5, 'hA000);
                    fire_write(6, 'hC000); fire_read(7, 'hE000);
                end
            end
        join

        fork
            wait_write_resp(16);
            wait_read_resp(16);
        join

        end_time = $realtime;
        $display("[Result E] Duration: %0f ps", end_time - start_time);


        $display("============================================================");
        $display("   Test Completed.                                          ");
        $display("============================================================");
        $finish;
    end

endmodule
