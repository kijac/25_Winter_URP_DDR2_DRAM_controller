`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

// Force Sync Update 2
`define DRAM_BK_CNT2 4

module SAL_SCHED
#(
    parameter   bk_cnt = 4
)
(
    // clock & reset
    input                       clk,
    input                       rst_n,

    TIMING_IF.MON               timing_if,

    // requests from bank controllers
    BK_CTRL_IF.SCHED            bk_if,
    
    SCHED_IF.SCHED              sched_if
);

    //Our code
    // 1. Parameters & Localparams
    localparam STARVATION_LIMIT = 8'd16; // Fixed at 16 per user requirement

    // 2. Global Timing Counters
    
    // 2.1 tRRD
    wire is_zero_trrd;
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RRD_WIDTH)) u_cnt_trrd (
        .clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.act_gnt), .reset_value_i(timing_if.t_rrd_m1), 
        .is_zero_o(is_zero_trrd));

    // 2.2 tCCD
    wire is_zero_tccd;
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_CCD_WIDTH)) u_cnt_tccd (
        .clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.rd_gnt | sched_if.wr_gnt), .reset_value_i(timing_if.t_ccd_m1), .is_zero_o(is_zero_tccd));

    // 2.3 tWTR
    wire is_zero_twtr;
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_WTR_WIDTH)) u_cnt_twtr (
        .clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.wr_gnt), .reset_value_i(timing_if.t_wtr_m1), 
        .is_zero_o(is_zero_twtr));

    // 2.4 tRTW
    wire is_zero_trtw;
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RTW_WIDTH)) u_cnt_trtw (
        .clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.rd_gnt), .reset_value_i(timing_if.t_rtw_m1), 
        .is_zero_o(is_zero_trtw));

    // 2.5 tRFC
    wire is_zero_trfc;
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RFC_WIDTH)) u_cnt_trfc (
        .clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.ref_gnt), .reset_value_i(timing_if.t_rfc_m1), .is_zero_o(is_zero_trfc));


    // 3. Optimization Logic
    // 3.1 Last Operation Direction
    
    logic last_op_was_write;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_op_was_write <= 1'b0;
        end else begin
            if (sched_if.wr_gnt) begin
                last_op_was_write <= 1'b1;
            end else if (sched_if.rd_gnt) begin
                last_op_was_write <= 1'b0;
            end
            // Keep state on NOP or ACT/PRE
        end
    end

    // 3.2 Starvation Protection (Safety Net)
    // Counts how long a valid request has been waiting without a grant.
    logic [7:0] wait_timer [bk_cnt];
    logic [bk_cnt-1:0] is_starved;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for(int i=0; i<bk_cnt; i++) wait_timer[i] <= '0;
        end else begin
            for(int i=0; i<bk_cnt; i++) begin
                // If request exists...
                if (bk_if.reqs[i].rd_req || bk_if.reqs[i].wr_req || bk_if.reqs[i].act_req) begin
                    // ...and NOT granted yet
                    if (!bk_if.gnts[i].rd_gnt && !bk_if.gnts[i].wr_gnt && !bk_if.gnts[i].act_gnt) begin
                        // This prevents banks from being blocked indefinitely by a stream of R/W in other banks.
                        if (wait_timer[i] < STARVATION_LIMIT) 
                            wait_timer[i] <= wait_timer[i] + 1;
                    end else begin
                        wait_timer[i] <= '0; // Granted! Reset timer.
                    end
                end else begin
                    wait_timer[i] <= '0; // No request. Reset timer.
                end
            end
        end
    end

    assign is_starved[0] = (wait_timer[0] == STARVATION_LIMIT);
    assign is_starved[1] = (wait_timer[1] == STARVATION_LIMIT);
    assign is_starved[2] = (wait_timer[2] == STARVATION_LIMIT);
    assign is_starved[3] = (wait_timer[3] == STARVATION_LIMIT);


    // 4. Arbitration Logic

    // Request Vectors
    logic [bk_cnt-1:0] req_acts, req_rds, req_wrs, req_pres;
    genvar i;
    generate
        for(i=0; i<bk_cnt; i++) begin
            assign req_acts[i] = bk_if.reqs[i].act_req;
            assign req_rds[i]  = bk_if.reqs[i].rd_req;
            assign req_wrs[i]  = bk_if.reqs[i].wr_req;
            assign req_pres[i] = bk_if.reqs[i].pre_req;
        end
    endgenerate

    logic [bk_cnt-1:0] cand_valid, cand_timing_ok, base_set;
    logic [bk_cnt-1:0] cand_emergency, cand_prio_dir, cand_row_hit;
    logic [bk_cnt-1:0] final_candidates;
    logic [bk_cnt-1:0] winner_oh;

    always_comb begin
        // A. Validity Check
        cand_valid = req_acts | req_rds | req_wrs | req_pres;

        // B. Timing Check
        cand_timing_ok = '0;
        if (is_zero_trfc) begin
            for(int k=0; k<bk_cnt; k++) begin
                if (req_acts[k])        cand_timing_ok[k] = is_zero_trrd;
                else if (req_rds[k])    cand_timing_ok[k] = is_zero_tccd & is_zero_twtr;
                else if (req_wrs[k])    cand_timing_ok[k] = is_zero_tccd & is_zero_trtw;
                else if (req_pres[k])   cand_timing_ok[k] = 1'b1;
            end
        end
    end
    
    // ID-Specific Ordering Check
    logic [bk_cnt-1:0] id_ordering_safe;
    
    // Check if any other bank has the SAME ID but a SMALLER seq_num (Older request)
    always_comb begin
        id_ordering_safe = '1; // Default to safe (allow all)
        
        for(int k=0; k<bk_cnt; k++) begin
            // For a candidate bank K
             if (bk_if.reqs[k].act_req || bk_if.reqs[k].rd_req || bk_if.reqs[k].wr_req || bk_if.reqs[k].pre_req) begin
                
                for(int j=0; j<bk_cnt; j++) begin
                    if (k != j) begin
                        // If bank J also has a request
                         if (bk_if.reqs[j].act_req || bk_if.reqs[j].rd_req || bk_if.reqs[j].wr_req || bk_if.reqs[j].pre_req) begin
                             // And IDs match
                             if (bk_if.reqs[k].id == bk_if.reqs[j].id) begin
                                 // Wraparound-safe Comparison:
                                 // Check if J < K in a circular buffer sense.
                                 logic [7:0] diff;
                                 diff = bk_if.reqs[k].seq_num - bk_if.reqs[j].seq_num;
                                 if (diff < 8'd128 && diff != 0) begin
                                     id_ordering_safe[k] = 1'b0; // K must wait for J
                                 end
                             end
                         end
                    end
                end

             end
        end
    end

    always_comb begin
        // Base Set: Valid + Timing OK + ID Safe
        base_set = cand_valid & cand_timing_ok & id_ordering_safe;

        // C. Intelligent Filtering (Priority System)
        
        // Priority 0: Starvation (Emergency)
        // If anyone has waited too long, they bypass optimization rules.
        cand_emergency = base_set & is_starved;

        // Priority 1: Same Direction (Maximize Bus Utilization)
        // Try to keep doing what we were doing (Read or Write) to avoid tWTR/tRTW bubbles.
        if (last_op_was_write)
            cand_prio_dir = base_set & req_wrs;
        else
            cand_prio_dir = base_set & req_rds;

        // Priority 2: Row Hits (Data Transfer > ACT/PRE)
        // If we can't maintain direction, at least try to do SOME data transfer.
        cand_row_hit = base_set & (req_rds | req_wrs);

        // Priority 4: Wait more if Precharge is issued
        // To avoid PRE -> ACT/RD/WR violation immediately

        // Final Selection Hierarchy
        if (|cand_emergency)        final_candidates = cand_emergency; // Emergency First!
        else if (|cand_prio_dir)    final_candidates = cand_prio_dir;  // Same Direction Second
        else if (|cand_row_hit)     final_candidates = cand_row_hit;   // Then Any Hit (Data Bus Open)
        else                        final_candidates = base_set & ~req_rds & ~req_wrs; // Avoid RD/WR if not hit to prevent blind access
    end


    // D. Global Sequence Number Based Grant (Oldest First)
    // Instead of Round Robin, we pick the candidate with the 'smallest' seq_num.
    // Smallest means oldest in a circular buffer sense.
    
    always_comb begin
        winner_oh = '0;
        
        // Only if there are candidates
        if (|final_candidates) begin
            logic [7:0] min_seq;
            int winner_idx;
            logic first_found;
            
            winner_idx = 0;
            first_found = 1'b0;
            min_seq = '0; // default (not actually used since first_found logic overwrites it)

            for (int k=0; k<bk_cnt; k++) begin
                if (final_candidates[k]) begin
                    if (!first_found) begin
                        // First valid candidate we see becomes the temporary winner
                        winner_idx = k;
                        // Use a dummy variable or specific logic to get min_seq
                        min_seq = bk_if.reqs[k].seq_num;
                        first_found = 1'b1;
                    end else begin
                        // We compare current k's seq (seq_k) vs min_seq
                        // We want to check: Is seq_k < min_seq (older)?
                        // Logic: wrapped diff (seq_k - min_seq)
                        // If result is small pos -> seq_k > min_seq (newer)
                        // If result is large pos (neg) -> seq_k < min_seq (older)
                        
                        logic [7:0] seq_k;
                        logic [7:0] diff_k_min;
                        
                        seq_k = bk_if.reqs[k].seq_num;
                        diff_k_min = seq_k - min_seq;
                        
                        // If diff_k_min >= 128, it means seq_k is effectively smaller (older) due to wrap-around
                        // or just simple smaller number.
                        // Example: seq_k=255, min=0. diff = 255. 255 >= 128. True. k is older.
                        // Example: seq_k=0, min=255. diff = 1. False. k is newer.
                        
                        if (diff_k_min >= 8'd128) begin 
                            winner_idx = k;
                            min_seq = seq_k;
                        end
                    end
                end
            end
            
            winner_oh[winner_idx] = 1'b1;
        end
    end

    // 5. Output Grant Generation
    logic [bk_cnt-1:0] all_ref_reqs;
    genvar k;
    generate
        for(k=0; k<bk_cnt; k++) begin
            assign bk_if.gnts[k].act_gnt = winner_oh[k] && req_acts[k];
            assign bk_if.gnts[k].rd_gnt  = winner_oh[k] && req_rds[k];
            assign bk_if.gnts[k].wr_gnt  = winner_oh[k] && req_wrs[k];
            assign bk_if.gnts[k].pre_gnt = winner_oh[k] && req_pres[k];

            assign all_ref_reqs[k]       = bk_if.reqs[k].ref_req;
            // Grant refresh to all banks when the scheduler decides to refresh
            assign bk_if.gnts[k].ref_gnt = sched_if.ref_gnt; 
        end
    endgenerate

    assign sched_if.act_gnt = |(winner_oh & req_acts);
    assign sched_if.rd_gnt  = |(winner_oh & req_rds);
    assign sched_if.wr_gnt  = |(winner_oh & req_wrs);
    assign sched_if.ref_gnt = (!is_zero_trfc) ? 1'b0 : (&all_ref_reqs); 
    assign sched_if.pre_gnt = |(winner_oh & req_pres);

    // 6. Data Muxing
    always_comb begin
        sched_if.ba      = '0;
        sched_if.ra      = '0;
        sched_if.ca      = '0;
        sched_if.id      = '0;
        sched_if.len     = '0;
        
        for(int k=0; k<bk_cnt; k++) begin
            if (winner_oh[k]) begin
                sched_if.ba      = bk_if.reqs[k].ba;
                sched_if.ra      = bk_if.reqs[k].ra;
                sched_if.ca      = bk_if.reqs[k].ca;
                sched_if.id      = bk_if.reqs[k].id;
                sched_if.len     = bk_if.reqs[k].len;
            end
        end
    end

endmodule
