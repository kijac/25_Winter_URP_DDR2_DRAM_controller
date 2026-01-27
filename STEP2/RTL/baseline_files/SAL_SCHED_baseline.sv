`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

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

    logic   [bk_cnt-1:0]        granted_bank_oh;
    logic                       gnt_done;
    reg    [$clog2(bk_cnt)-1:0] last_bank;

    logic   [bk_cnt-1:0]        ref_req_bus;
    logic                       all_banks_ready_for_ref;

    wire is_t_rrd_met, is_t_ccd_met, is_t_rtw_met, is_t_wtr_met;
    
    SAL_TIMING_CNTR #(`T_RRD_WIDTH) t_rrd_cntr (.clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.act_gnt), .reset_value_i(timing_if.t_rrd_m1), .is_zero_o(is_t_rrd_met));
    SAL_TIMING_CNTR #(`T_CCD_WIDTH) t_ccd_cntr (.clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.rd_gnt | sched_if.wr_gnt), .reset_value_i(timing_if.t_ccd_m1), .is_zero_o(is_t_ccd_met));
    SAL_TIMING_CNTR #(`T_WTR_WIDTH) t_wtr_cntr (.clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.wr_gnt), .reset_value_i(timing_if.t_wtr_m1), .is_zero_o(is_t_wtr_met));
    SAL_TIMING_CNTR #(`T_RTW_WIDTH) t_rtw_cntr (.clk(clk), .rst_n(rst_n), .reset_cmd_i(sched_if.rd_gnt), .reset_value_i(timing_if.t_rtw_m1), .is_zero_o(is_t_rtw_met));


    /*
    * FILL YOUR CODES HERE
    */

    always_comb begin
        for(int i = 0; i < bk_cnt; i++) begin
            ref_req_bus[i] = bk_if.reqs[i].ref_req;
        end
    end

    assign all_banks_ready_for_ref = &ref_req_bus;

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            last_bank <= '0;
        end else begin
            if(gnt_done && !sched_if.ref_gnt) begin
                for(int i = 0; i <bk_cnt; i++) begin
                    if(granted_bank_oh[i]) begin
                        last_bank <= i[1:0] + 1'b1;
                    end
                end
            end
        end
    end

    always_comb begin
        gnt_done = 1'b0;
        granted_bank_oh = '0;

        sched_if.act_gnt    = 1'b0;
        sched_if.rd_gnt     = 1'b0;
        sched_if.wr_gnt     = 1'b0;
        sched_if.pre_gnt    = 1'b0;
        sched_if.ref_gnt    = 1'b0;
        sched_if.ba         = 'x;
        sched_if.ra         = 'x;
        sched_if.ca         = 'x;
        sched_if.id         = 'x;
        sched_if.len        = 'x;

        for(int i = 0; i < bk_cnt; i++) begin
            bk_if.gnts[i] = 1'b0;
        end

        //Refresh
        if(all_banks_ready_for_ref) begin
            gnt_done = 1'b1;
            sched_if.ref_gnt = 1'b1;

            for(int k = 0; k < bk_cnt; k++) begin
                bk_if.gnts[k].ref_gnt = 1'b1;
            end

        end

        if (!gnt_done) begin
            for (int i = 0; i < bk_cnt; i++) begin
                if(gnt_done) break;
                if (!gnt_done) begin
                    if (bk_if.reqs[(last_bank + i) % 4].rd_req && is_t_ccd_met && is_t_wtr_met) begin
                        gnt_done = 1'b1;
                        granted_bank_oh[(last_bank + i) % 4] = 1'b1;
                        bk_if.gnts[(last_bank + i) % 4].rd_gnt = 1'b1;
                        sched_if.rd_gnt = 1'b1;
                    end else if (bk_if.reqs[(last_bank + i) % 4].wr_req && is_t_ccd_met && is_t_rtw_met) begin
                        gnt_done = 1'b1;
                        granted_bank_oh[(last_bank + i) % 4] = 1'b1;
                        bk_if.gnts[(last_bank + i) % 4].wr_gnt = 1'b1;
                        sched_if.wr_gnt = 1'b1;
                    end
                end
            end
        end

        // 2. ACTIVATE request
        if (!gnt_done) begin
            for (int i = 0; i < bk_cnt; i++) begin
                if(gnt_done) break;
                if (!gnt_done) begin
                    if (bk_if.reqs[(last_bank + i) % 4].act_req && is_t_rrd_met) begin
                        gnt_done = 1'b1;
                        granted_bank_oh[(last_bank + i) % 4] = 1'b1;
                        bk_if.gnts[(last_bank + i) % 4].act_gnt = 1'b1;
                        sched_if.act_gnt = 1'b1;
                    end
                end
            end
        end

        // 3. PRECHARGE request
        if (!gnt_done) begin
            for (int i = 0; i < bk_cnt; i++) begin
                if(gnt_done) break;
                if (!gnt_done) begin
                    if (bk_if.reqs[(last_bank + i) % 4].pre_req) begin
                        gnt_done = 1'b1;
                        granted_bank_oh[(last_bank + i) % 4] = 1'b1;
                        bk_if.gnts[(last_bank + i) % 4].pre_gnt = 1'b1;
                        sched_if.pre_gnt = 1'b1;
                    end
                end
            end
        end

        // Pass winning bank's info to other modules via sched_if
        for (int i = 0; i < bk_cnt; i++) begin
            if (granted_bank_oh[(last_bank + i) % 4]) begin
                sched_if.ba     = bk_if.reqs[(last_bank + i) % 4].ba;
                sched_if.ra     = bk_if.reqs[(last_bank + i) % 4].ra;
                sched_if.ca     = bk_if.reqs[(last_bank + i) % 4].ca;
                sched_if.id     = bk_if.reqs[(last_bank + i) % 4].id;
                sched_if.len    = bk_if.reqs[(last_bank + i) % 4].len;
            end
        end
    end
    

endmodule // SAL_SCHED
