`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

module SAL_BK_CTRL
#(
    parameter BK_ID = 0
)
(
    input  wire                 clk,
    input  wire                 rst_n,
    TIMING_IF.MON               timing_if,
    REQ_IF.DST                  req_if,
    output bk_req_t             bk_reqs,
    input  bk_gnt_t             bk_gnts,
    input  wire                 ref_req_i,
    output wire                 ref_gnt_o
);

    // 0. Constant Generation (Logic 0 & Logic 1)
    wire logic_0, logic_1;
    AND2 u_tie_low (.A1(1'b0), .A2(1'b0), .Y(logic_0));
    INV  u_tie_high (.A(logic_0), .Y(logic_1));

    // 1. Internal Wires Definition
    wire [6:0] state;       // Current State (Q)
    wire [6:0] state_n;     // Next State (D)

    // Data Path Registers
    wire req_buf_valid;
    wire req_buf_valid_n;
    wire req_buf_wr;
    wire [`AXI_ID_WIDTH-1:0]    req_buf_id;
    wire [`AXI_LEN_WIDTH-1:0]   req_buf_len;
    wire [`DRAM_RA_WIDTH-1:0]   req_buf_ra;
    wire [`DRAM_RA_WIDTH-1:0]   cur_ra;
    wire [`DRAM_CA_WIDTH-1:0]   req_buf_ca;
    wire [2:0]                  req_buf_seq_num;

    // Timing Status Signals
    wire is_t_rc_met, is_t_rcd_met, is_t_rp_met, is_t_ras_met, is_t_rfc_met;
    wire is_t_rtp_met, is_t_wtp_met, is_row_open_met, is_t_wtr_met, is_t_rtw_met;

    // Logic Flags
    wire ref_req_pending;
    wire latch_enable;
    
    // 2. Timing Counters Instantiation

    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RC_WIDTH))       u_rc_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.act_gnt),.reset_value_i(timing_if.t_rc_m1),.is_zero_o(is_t_rc_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RCD_WIDTH))     u_rcd_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.act_gnt),.reset_value_i(timing_if.t_rcd_m2),.is_zero_o(is_t_rcd_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RP_WIDTH))       u_rp_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.pre_gnt),.reset_value_i(timing_if.t_rp_m2),.is_zero_o(is_t_rp_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RAS_WIDTH))     u_ras_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.act_gnt),.reset_value_i(timing_if.t_ras_m1),.is_zero_o(is_t_ras_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RTP_WIDTH))     u_rtp_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.rd_gnt),.reset_value_i(timing_if.t_rtp_m1),.is_zero_o(is_t_rtp_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_WTP_WIDTH))     u_wtp_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.wr_gnt),.reset_value_i(timing_if.t_wtp_m1),.is_zero_o(is_t_wtp_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`ROW_OPEN_WIDTH)) u_row_open_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.act_gnt),.reset_value_i(timing_if.row_open_cnt),.is_zero_o(is_row_open_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RFC_WIDTH))     u_rfc_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.ref_gnt),.reset_value_i(timing_if.t_rfc_m2),.is_zero_o(is_t_rfc_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_WTR_WIDTH))     u_wtr_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.wr_gnt),.reset_value_i(timing_if.t_wtr_m1),.is_zero_o(is_t_wtr_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RTW_WIDTH))     u_rtw_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(bk_gnts.rd_gnt),.reset_value_i(timing_if.t_rtw_m1),.is_zero_o(is_t_rtw_met));

    // 3. Periodic Refresh Logic
    wire n_clear_cond, set_or_hold, next_pending;
    NAND2 u_nand_clear (.A1(bk_reqs.ref_req), .A2(bk_gnts.ref_gnt), .Y(n_clear_cond));
    OR2   u_or_set_hold (.A1(ref_req_i), .A2(ref_req_pending), .Y(set_or_hold));
    AND2  u_and_next (.A1(n_clear_cond), .A2(set_or_hold), .Y(next_pending));
    DFF   u_dff_pending (.D(next_pending), .RST_n(rst_n), .CLK(clk), .Q(ref_req_pending), .QN());

    // 4. Data Path Registers & Control
    
    // 4.1 Latch Enable & Ready Signal
    wire n_req_buf_valid;
    INV u_inv_valid (.A(req_buf_valid), .Y(n_req_buf_valid));
    INV u_drive_rdy (.A(req_buf_valid), .Y(req_if.ready));
    AND2 u_latch_en (.A1(req_if.valid), .A2(n_req_buf_valid), .Y(latch_enable));

    // 4.2 Payload Registers (Using Generate + MUX + DFF)
    genvar i;
    generate
        wire wr_next;
        MUX21 u_mux_wr (.A1(req_if.wr), .A2(req_buf_wr), .S0(latch_enable), .Y(wr_next));
        DFF u_dff_wr (.D(wr_next), .RST_n(rst_n), .CLK(clk), .Q(req_buf_wr), .QN());

        for(i=0; i<`AXI_ID_WIDTH; i=i+1) begin : gen_buf_id
            wire id_next;
            MUX21 u_mux_id (.A1(req_if.id[i]), .A2(req_buf_id[i]), .S0(latch_enable), .Y(id_next));
            DFF u_dff_id (.D(id_next), .RST_n(rst_n), .CLK(clk), .Q(req_buf_id[i]), .QN());
        end

        for(i=0; i<`AXI_LEN_WIDTH; i=i+1) begin : gen_buf_len
            wire len_next;
            MUX21 u_mux_len (.A1(req_if.len[i]), .A2(req_buf_len[i]), .S0(latch_enable), .Y(len_next));
            DFF u_dff_len (.D(len_next), .RST_n(rst_n), .CLK(clk), .Q(req_buf_len[i]), .QN());
        end

        for(i=0; i<`DRAM_RA_WIDTH; i=i+1) begin : gen_buf_ra
            wire ra_next;
            MUX21 u_mux_ra (.A1(req_if.ra[i]), .A2(req_buf_ra[i]), .S0(latch_enable), .Y(ra_next));
            DFF u_dff_ra (.D(ra_next), .RST_n(rst_n), .CLK(clk), .Q(req_buf_ra[i]), .QN());
        end

        for(i=0; i<`DRAM_CA_WIDTH; i=i+1) begin : gen_buf_ca
            wire ca_next;
            MUX21 u_mux_ca (.A1(req_if.ca[i]), .A2(req_buf_ca[i]), .S0(latch_enable), .Y(ca_next));
            DFF u_dff_ca (.D(ca_next), .RST_n(rst_n), .CLK(clk), .Q(req_buf_ca[i]), .QN());
        end

        for(i=0; i<3; i=i+1) begin : gen_buf_seq
            wire seq_next;
            MUX21 u_mux_seq (.A1(req_if.seq_num[i]), .A2(req_buf_seq_num[i]), .S0(latch_enable), .Y(seq_next));
            DFF u_dff_seq (.D(seq_next), .RST_n(rst_n), .CLK(clk), .Q(req_buf_seq_num[i]), .QN());
        end

        for(i=0; i<`DRAM_RA_WIDTH; i=i+1) begin : gen_cur_ra
            wire cur_next_val;
            MUX21 u_mux_cur (.A1(req_buf_ra[i]), .A2(cur_ra[i]), .S0(bk_gnts.act_gnt), .Y(cur_next_val));
            DFF u_dff_cur_ra (.D(cur_next_val), .RST_n(rst_n), .CLK(clk), .Q(cur_ra[i]), .QN());
        end
    endgenerate

    // 4.3 Request Buffer Valid Register Logic
    wire valid_set, valid_clear, n_valid_clear, valid_hold;
    
    AND2 u_v_set (.A1(req_if.valid), .A2(n_req_buf_valid), .Y(valid_set));
    OR2  u_v_clr (.A1(bk_gnts.wr_gnt), .A2(bk_gnts.rd_gnt), .Y(valid_clear));
    INV  u_inv_v_clr (.A(valid_clear), .Y(n_valid_clear));
    AND2 u_v_hold (.A1(req_buf_valid), .A2(n_valid_clear), .Y(valid_hold));
    OR2  u_v_next (.A1(valid_hold), .A2(valid_set), .Y(req_buf_valid_n));
    
    DFF  u_dff_req_valid (.D(req_buf_valid_n), .RST_n(rst_n), .CLK(clk), .Q(req_buf_valid), .QN());

    // 5. Combinational Logic

    // 5.1 Row Miss Detector
    wire [`DRAM_RA_WIDTH-1:0] ra_diff_bits;
    generate
        for(i=0; i<`DRAM_RA_WIDTH; i=i+1) begin : ra_comp
            XOR2 u_xor_ra (.A1(req_buf_ra[i]), .A2(cur_ra[i]), .Y(ra_diff_bits[i]));
        end
    endgenerate

    wire [`DRAM_RA_WIDTH-1:0] chain_w; 
    wire row_miss;

    generate
        AND2 u_base_inv (.A1(ra_diff_bits[0]), .A2(logic_1), .Y(chain_w[0]));
        genvar m;
        for(m=1; m<`DRAM_RA_WIDTH; m=m+1) begin : gen_or_chain
            OR2 u_miss_or (.A1(chain_w[m-1]), .A2(ra_diff_bits[m]), .Y(chain_w[m]));
        end
        AND2 u_out_inv (.A1(chain_w[`DRAM_RA_WIDTH-1]), .A2(logic_1), .Y(row_miss));
    endgenerate

    // 5.2 Priority Logic Signals
    
    // 1. Refresh Priority Logic
    wire cond_ref_needed, timer_ref_ready;
    AND3 u_tm_ref (.A1(is_t_wtp_met), .A2(is_t_rtp_met), .A3(is_t_ras_met), .Y(timer_ref_ready));
    AND2 u_cond_ref (.A1(ref_req_pending), .A2(timer_ref_ready), .Y(cond_ref_needed));

    // 2. Row Miss Logic (Timing Aware)
    wire n_cond_ref, miss_timers, raw_miss_req, cond_miss_needed;
    INV  u_inv_ref (.A(cond_ref_needed), .Y(n_cond_ref));
    AND3 u_tm_miss (.A1(is_t_ras_met), .A2(is_t_wtp_met), .A3(is_t_rtp_met), .Y(miss_timers));
    AND3 u_raw_miss (.A1(req_buf_valid), .A2(row_miss), .A3(miss_timers), .Y(raw_miss_req));
    AND2 u_prio_miss (.A1(raw_miss_req), .A2(n_cond_ref), .Y(cond_miss_needed));
    wire n_pure_row_miss; 
    INV u_inv_pure_miss (.A(row_miss), .Y(n_pure_row_miss));

    // 3. Write Priority Logic
    wire n_cond_miss_NOT_USED, wr_timers, raw_wr_req, cond_wr_needed;
    INV  u_inv_miss (.A(cond_miss_needed), .Y(n_cond_miss_NOT_USED)); 
    AND2 u_tm_wr (.A1(is_t_rcd_met), .A2(is_t_rtw_met), .Y(wr_timers));
    AND3 u_raw_wr (.A1(req_buf_valid), .A2(req_buf_wr), .A3(wr_timers), .Y(raw_wr_req));
    AND3 u_prio_wr (.A1(raw_wr_req), .A2(n_cond_ref), .A3(n_pure_row_miss), .Y(cond_wr_needed));

    // 4. Read Priority Logic
    wire n_cond_wr, n_req_buf_wr, rd_timers, raw_rd_req, cond_rd_needed;
    INV  u_inv_wr_prio (.A(cond_wr_needed), .Y(n_cond_wr));
    INV  u_inv_wr_flag (.A(req_buf_wr), .Y(n_req_buf_wr));
    AND2 u_tm_rd (.A1(is_t_rcd_met), .A2(is_t_wtr_met), .Y(rd_timers));
    AND3 u_raw_rd (.A1(req_buf_valid), .A2(n_req_buf_wr), .A3(rd_timers), .Y(raw_rd_req));
    AND3 u_prio_rd (.A1(raw_rd_req), .A2(n_pure_row_miss), .A3(n_cond_ref), .Y(cond_rd_needed));
    
    // 5.3 Output Driving
    
    // (a) Drive bk_reqs.ba
    genvar j;
    generate
        for(j=0; j<2; j=j+1) begin : gen_ba_mux
            
            MUX21 u_mux_ba (
                .A1(logic_1),
                .A2(logic_0),
                .S0(BK_ID[j]),
                .Y(bk_reqs.ba[j])
            );
        end
    endgenerate

    // (b) Drive bk_reqs.ref_req
    wire idle_timer_ref, idle_ref_req;
    AND2 u_id_tm (.A1(is_t_rp_met), .A2(is_t_rc_met), .Y(idle_timer_ref));
    AND3 u_out_ref_idle (.A1(state[0]), .A2(ref_req_pending), .A3(idle_timer_ref), .Y(idle_ref_req));
    AND2 u_buf_idle (.A1(idle_ref_req), .A2(logic_1), .Y(bk_reqs.ref_req));

    // (c) Drive bk_reqs.act_req
    wire n_idle_ref_req, idle_timer_act, raw_act_req, act_req_internal;
    INV  u_inv_id_ref (.A(idle_ref_req), .Y(n_idle_ref_req));
    AND2 u_id_tm_act (.A1(is_t_rc_met), .A2(is_t_rp_met), .Y(idle_timer_act));
    AND3 u_raw_act (.A1(state[0]), .A2(req_buf_valid), .A3(idle_timer_act), .Y(raw_act_req));
    AND2 u_out_act (.A1(raw_act_req), .A2(n_idle_ref_req), .Y(act_req_internal));
    AND2 u_buf_act (.A1(act_req_internal), .A2(logic_1), .Y(bk_reqs.act_req));

    // (d) Drive bk_reqs.pre_req
    wire active_pre_req, pre_req_internal;
    OR2  u_or_pre_cond (.A1(cond_ref_needed), .A2(cond_miss_needed), .Y(active_pre_req));
    AND2 u_out_pre (.A1(state[2]), .A2(active_pre_req), .Y(pre_req_internal));
    AND2 u_buf_pre (.A1(pre_req_internal), .A2(logic_1), .Y(bk_reqs.pre_req));

    // (e) Drive bk_reqs.wr_req
    wire wr_req_internal, n_out_wr;
    AND2 u_out_wr (.A1(state[2]), .A2(cond_wr_needed), .Y(wr_req_internal));
    AND2 u_buf_wr (.A1(wr_req_internal), .A2(logic_1), .Y(bk_reqs.wr_req));


    // (f) Drive bk_reqs.rd_req
    wire rd_req_internal;
    AND2 u_out_rd (.A1(state[2]), .A2(cond_rd_needed), .Y(rd_req_internal));
    AND2 u_buf_rd (.A1(rd_req_internal), .A2(logic_1), .Y(bk_reqs.rd_req));


    // (g) Drive ref_gnt_o
    INV u_buf_gnt2 (.A(logic_1), .Y(ref_gnt_o));

    // (h) Pass-through Data Bus
    generate
        for(i=0; i<`DRAM_RA_WIDTH; i=i+1) begin: pass_ra
            AND2 u_and (.A1(req_buf_ra[i]), .A2(logic_1), .Y(bk_reqs.ra[i]));
        end
        for(i=0; i<`DRAM_CA_WIDTH; i=i+1) begin: pass_ca
            AND2 u_and (.A1(req_buf_ca[i]), .A2(logic_1), .Y(bk_reqs.ca[i]));
        end
        for(i=0; i<`AXI_ID_WIDTH; i=i+1) begin: pass_id
            AND2 u_and (.A1(req_buf_id[i]), .A2(logic_1), .Y(bk_reqs.id[i]));
        end
        for(i=0; i<`AXI_LEN_WIDTH; i=i+1) begin: pass_len
            AND2 u_and (.A1(req_buf_len[i]), .A2(logic_1), .Y(bk_reqs.len[i]));
        end
        for(i=0; i<3; i=i+1) begin: pass_seq
            AND2 u_and (.A1(req_buf_seq_num[i]), .A2(logic_1), .Y(bk_reqs.seq_num[i]));
        end
    endgenerate

    // 5.4 Next State Logic
    
    // IDLE (state[0])
    wire trans_pre_idle;
    AND2 u_tr_pi (.A1(state[5]), .A2(is_t_rp_met), .Y(trans_pre_idle));
    wire trans_ref_idle;
    AND2 u_tr_ri (.A1(state[6]), .A2(is_t_rfc_met), .Y(trans_ref_idle));
    wire idle_gnt_ref, idle_gnt_act;
    AND2 u_ig_ref (.A1(idle_ref_req), .A2(bk_gnts.ref_gnt), .Y(idle_gnt_ref));
    AND2 u_ig_act (.A1(act_req_internal), .A2(bk_gnts.act_gnt), .Y(idle_gnt_act));
    wire idle_exit, n_idle_exit, hold_idle, next_idle_or;
    OR2  u_id_exit (.A1(idle_gnt_ref), .A2(idle_gnt_act), .Y(idle_exit));
    INV  u_inv_ie (.A(idle_exit), .Y(n_idle_exit));
    AND2 u_h_idle (.A1(state[0]), .A2(n_idle_exit), .Y(hold_idle));
    OR2  u_ni_1 (.A1(trans_pre_idle), .A2(trans_ref_idle), .Y(next_idle_or));
    OR2  u_ni_2 (.A1(next_idle_or), .A2(hold_idle), .Y(state_n[0]));

    // ACTIVATING (state[1])
    wire trans_idle_act;
    AND2 u_trans_idle_act (.A1(idle_gnt_act), .A2(logic_1), .Y(trans_idle_act));
    wire n_rcd_met, hold_act;
    INV  u_inv_rcd (.A(is_t_rcd_met), .Y(n_rcd_met));
    AND2 u_h_act (.A1(state[1]), .A2(n_rcd_met), .Y(hold_act));
    OR2  u_ns_1 (.A1(trans_idle_act), .A2(hold_act), .Y(state_n[1]));

    // BANK_ACTIVE (state[2])
    wire trans_act_active;
    AND2 u_tr_aa (.A1(state[1]), .A2(is_t_rcd_met), .Y(trans_act_active));
    wire active_gnt_pre, active_gnt_wr, active_gnt_rd;
    AND2 u_ag_pre (.A1(pre_req_internal), .A2(bk_gnts.pre_gnt), .Y(active_gnt_pre));
    AND2 u_ag_wr  (.A1(wr_req_internal),  .A2(bk_gnts.wr_gnt),  .Y(active_gnt_wr));
    AND2 u_ag_rd  (.A1(rd_req_internal),  .A2(bk_gnts.rd_gnt),  .Y(active_gnt_rd));
    wire active_exit_1, active_exit, n_active_exit, hold_active;
    OR2  u_ae_1 (.A1(active_gnt_pre), .A2(active_gnt_wr), .Y(active_exit_1));
    OR2  u_ae_2 (.A1(active_exit_1), .A2(active_gnt_rd), .Y(active_exit));
    INV  u_inv_ae (.A(active_exit), .Y(n_active_exit));
    AND2 u_h_active (.A1(state[2]), .A2(n_active_exit), .Y(hold_active));
    wire ns_2_or1, ns_2_or2;
    OR2  u_ns2_1 (.A1(trans_act_active), .A2(state[3]), .Y(ns_2_or1)); 
    OR2  u_ns2_2 (.A1(state[4]), .A2(hold_active), .Y(ns_2_or2));
    OR2  u_ns2_f (.A1(ns_2_or1), .A2(ns_2_or2), .Y(state_n[2]));

    // READING (state[3])
    AND2 u_sn3 (.A1(active_gnt_rd), .A2(logic_1), .Y(state_n[3]));

    // WRITING (state[4])
    AND2 u_sn4 (.A1(active_gnt_wr), .A2(logic_1), .Y(state_n[4]));

    // PRECHARGING (state[5])
    wire n_rp_met, hold_pre;
    INV  u_inv_rp (.A(is_t_rp_met), .Y(n_rp_met));
    AND2 u_h_pre (.A1(state[5]), .A2(n_rp_met), .Y(hold_pre));
    OR2  u_ns_5 (.A1(active_gnt_pre), .A2(hold_pre), .Y(state_n[5]));

    // REFRESH (state[6])
    wire n_rfc_met, hold_ref;
    INV  u_inv_rfc (.A(is_t_rfc_met), .Y(n_rfc_met));
    AND2 u_h_ref (.A1(state[6]), .A2(n_rfc_met), .Y(hold_ref));
    OR2  u_ns_6 (.A1(idle_gnt_ref), .A2(hold_ref), .Y(state_n[6]));

    // 6. FSM State Registers

    // 6.1 State[0] (IDLE): 리셋 시 1
    wire d_idle_inverted;
    wire q_idle_internal;

    INV u_inv_idle_in (.A(state_n[0]), .Y(d_idle_inverted));
    DFF u_dff_state_idle (
        .D(d_idle_inverted), 
        .RST_n(rst_n), 
        .CLK(clk), 
        .Q(q_idle_internal), 
        .QN()
    );
    INV u_inv_idle_out (.A(q_idle_internal), .Y(state[0]));

    // 6.2 State[6:1]: 리셋 시 0
    genvar k;
    generate
        for(k=1; k<7; k=k+1) begin : gen_state_reg_others
            DFF u_dff_state (
                .D(state_n[k]), 
                .RST_n(rst_n), 
                .CLK(clk), 
                .Q(state[k]), 
                .QN()
            );
        end
    endgenerate

endmodule