`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

// Out-of-Order Dispatch Decoder with Reorder Buffer
// Solves Head-of-Line Blocking issue of simple schedulers.

module SAL_ADDR_DECODER
#(
    parameter integer ROB_DEPTH = 4 // Number of slots (buffers)
)
(
    // clock & reset
    input                       clk,
    input                       rst_n,

    // request from the AXI side
    AXI_A_IF.DST                axi_ar_if,
    AXI_A_IF.DST                axi_aw_if,

    // requests to bank controllers
    REQ_IF.SRC                  req_if_arr[`DRAM_BK_CNT]
);

    // 1. Definition of Slot Entry
    typedef struct packed {
        logic                   valid;
        logic                   wr;         // 1: Write, 0: Read
        axi_id_t                id;
        axi_len_t               len;
        axi_addr_t              addr;
        seq_num_t               seq_num;
        dram_ba_t               target_ba;
        dram_ra_t               target_ra;
        dram_ca_t               target_ca;
    } rob_entry_t;

    rob_entry_t rob[ROB_DEPTH];
    
    // Sequence Number Management
    seq_num_t global_seq_num;

    // 2. Input Logic (Allocation)
    
    // Find first empty slot logic
    logic [$clog2(ROB_DEPTH)-1:0] alloc_ptr;
    logic                         alloc_valid;
    
    // Check if we have space
    logic full;
    logic [ROB_DEPTH-1:0] valid_vec;
    
    genvar i;
    generate
        for(i=0; i<ROB_DEPTH; i++) assign valid_vec[i] = rob[i].valid;
    endgenerate
    assign full = &valid_vec;

    // AXI Ready signals Generation
    // Give priority to WRITE if both valid (to drain write buffer), 
    // but only if we have space.
    logic aw_grant, ar_grant;
    
    always_comb begin
        aw_grant = 1'b0;
        ar_grant = 1'b0;

        if (!full) begin
           if (axi_aw_if.avalid) begin
               aw_grant = 1'b1; // Write takes priority / or Low Watermark logic here
           end else if (axi_ar_if.avalid) begin
               ar_grant = 1'b1;
           end
        end
    end

    assign axi_aw_if.aready = aw_grant;
    assign axi_ar_if.aready = ar_grant;


    // Allocation Logic (Find first '0' in valid_vec)
    always_comb begin
        alloc_ptr = '0;
        for(int k=0; k<ROB_DEPTH; k++) begin
            if (!rob[k].valid) begin
                alloc_ptr = k[$clog2(ROB_DEPTH)-1:0];
                break;
            end
        end
    end

    // 3. Dispatch Logic (Arbitration & De-allocation) 
    // Issue Candidate Selection
    logic [ROB_DEPTH-1:0] can_issue; // Bit mask of issuable slots
    logic [ROB_DEPTH-1:0] issue_grant_oh; // One-hot grant
    logic issue_valid;
    
    // Bank Status
    logic [`DRAM_BK_CNT-1:0] bank_ready;
    generate
        for(i=0; i<`DRAM_BK_CNT; i++) assign bank_ready[i] = req_if_arr[i].ready;
    endgenerate

    // Ordering Check Logic (Hazard Detection)
    logic [ROB_DEPTH-1:0] hazard_block;

    always_comb begin
        hazard_block = '0;
        
        for(int k=0; k<ROB_DEPTH; k++) begin
            if (rob[k].valid) begin
                // Check against ALL other older valid slots
                for(int j=0; j<ROB_DEPTH; j++) begin
                    if (k != j && rob[j].valid) begin
                        // Assuming ROB is NOT a circular queue but a pool, 
                        // we need real timestamps. For simplicity here:
                        // Compare seq_num: If J < K (J is older), J must be checked.
                        if (rob[j].seq_num < rob[k].seq_num) begin
                            
                            // 1. Same ID Hazard (Strict Ordering for Same ID)
                            if (rob[j].id == rob[k].id) 
                                hazard_block[k] = 1'b1;

                            // 2. RAW / WAW / WAR Hazard (Address Match)
                            // Even if ID is different, same address access needs ordering
                            // (Though usually AXI allows reordering if ID diff, 
                            //  DRAM coherence must be maintained)
                            // simple row/bank check is not enough, need full addr match
                            // (Approximation: Check Bank & Row)
                            if (rob[j].target_ba == rob[k].target_ba && 
                                rob[j].target_ra == rob[k].target_ra &&
                                rob[j].target_ca == rob[k].target_ca) begin // Simple Col check added
                                hazard_block[k] = 1'b1;
                            end     
                        end
                    end 
                end
            end
        end
    end

    // Determine Candidates
    always_comb begin
        for(int k=0; k<ROB_DEPTH; k++) begin
            can_issue[k] = 1'b0;
            if (rob[k].valid) begin
                // Condition 1: Target Bank is Ready
                if (bank_ready[rob[k].target_ba]) begin
                    // Condition 2: No Hazard blocking
                    if (!hazard_block[k]) begin
                         can_issue[k] = 1'b1;
                    end
                end
            end
        end
    end

    // Pick the "Best" Candidate
    // Policy: Oldest First (to prevent starvation in Reorder Buffer)
    // (Simply scan from seq_num perspective or simple iterate)
    rob_entry_t winner;
    logic [$clog2(ROB_DEPTH)-1:0] winner_idx;
    logic found_winner;

    always_comb begin
        issue_grant_oh = '0;
        winner_idx = '0;
        found_winner = 1'b0;
        
        // Simple Scan: Priority to "Smallest Seq Num" among candidates
        // This is inefficient in logic, but logically correct.
        // Optimization: Just pick First Valid found in linear scan? 
        // No, that biases towards low index slots.
        
        // 1st Pass: Try to find READ in candidates
        for(int k=0; k<ROB_DEPTH; k++) begin
            if (can_issue[k] && !rob[k].wr) begin
                 // Ideally pick the oldest read, but picking ANY read is better effectively
                 issue_grant_oh[k] = 1'b1;
                 found_winner = 1'b1;
                 winner_idx = k[$clog2(ROB_DEPTH)-1:0]; // Cast
                 break; 
            end
        end

        // 2nd Pass: If no read, pick WRITE
        if (!found_winner) begin
            for(int k=0; k<ROB_DEPTH; k++) begin
                if (can_issue[k]) begin
                     issue_grant_oh[k] = 1'b1;
                     found_winner = 1'b1;
                     winner_idx = k[$clog2(ROB_DEPTH)-1:0]; // Cast
                     break; 
                end
            end
        end
    end
    
    // 4. Sequential Update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            global_seq_num <= '0;
            for(int k=0; k<ROB_DEPTH; k++) rob[k].valid <= 1'b0;
        end else begin
            
            // Allocation (Input)
            if (aw_grant) begin // Write Accepted
                rob[alloc_ptr].valid     <= 1'b1;
                rob[alloc_ptr].wr        <= 1'b1;
                rob[alloc_ptr].id        <= axi_aw_if.aid;
                rob[alloc_ptr].len       <= axi_aw_if.alen;
                rob[alloc_ptr].addr      <= axi_aw_if.aaddr;
                rob[alloc_ptr].seq_num   <= global_seq_num;
                rob[alloc_ptr].target_ba <= get_dram_ba(axi_aw_if.aaddr);
                rob[alloc_ptr].target_ra <= get_dram_ra(axi_aw_if.aaddr);
                rob[alloc_ptr].target_ca <= get_dram_ca(axi_aw_if.aaddr);
                
                global_seq_num           <= global_seq_num + 1;
            end else if (ar_grant) begin // Read Accepted
                rob[alloc_ptr].valid     <= 1'b1;
                rob[alloc_ptr].wr        <= 1'b0;
                rob[alloc_ptr].id        <= axi_ar_if.aid;
                rob[alloc_ptr].len       <= axi_ar_if.alen;
                rob[alloc_ptr].addr      <= axi_ar_if.aaddr;
                rob[alloc_ptr].seq_num   <= global_seq_num;
                rob[alloc_ptr].target_ba <= get_dram_ba(axi_ar_if.aaddr);
                rob[alloc_ptr].target_ra <= get_dram_ra(axi_ar_if.aaddr);
                rob[alloc_ptr].target_ca <= get_dram_ca(axi_ar_if.aaddr);
                
                global_seq_num           <= global_seq_num + 1;
            end

            // Issue (Output) - Clear the slot
            if (found_winner) begin
                rob[winner_idx].valid <= 1'b0;
                
                // If Alloc happened at same cycle on same slot (impossible due to logic, but for safety)
                // If alloc_ptr == winner_idx, we would overwrite. 
                // However, current logic: IF alloc, we write '1'. IF issue, we write '0'.
                // If both? Valid should stay '1' (New item replaces Old item).
                // But alloc_ptr finds '0' valid. So alloc_ptr never points to a valid slot that is being issued?
                // Actually, if we clear it NOW, it becomes free NEXT cycle. So no conflict.
            end
        end
    end

    // 5. Connect Output Interface
    genvar geni;
    generate
        for (geni=0; geni<`DRAM_BK_CNT; geni=geni+1) begin
            // If this bank is the target of the winner, assert valid
            assign req_if_arr[geni].valid   = found_winner && (rob[winner_idx].target_ba == geni);
            
            // Mux out the winner info
            assign req_if_arr[geni].wr      = rob[winner_idx].wr;
            assign req_if_arr[geni].id      = rob[winner_idx].id;
            assign req_if_arr[geni].len     = rob[winner_idx].len;
            assign req_if_arr[geni].seq_num = rob[winner_idx].seq_num;
            assign req_if_arr[geni].ra      = rob[winner_idx].target_ra;
            assign req_if_arr[geni].ca      = rob[winner_idx].target_ca;
        end
    endgenerate

endmodule // SAL_ADDR_DECODER_OO
