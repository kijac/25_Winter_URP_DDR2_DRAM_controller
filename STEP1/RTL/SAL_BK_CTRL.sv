`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

module SAL_BK_CTRL
(
    // clock & reset
    input                       clk,
    input                       rst_n,

    // timing parameters
    TIMING_IF.MON               timing_if,

    // request from the address decoder
    REQ_IF.DST                  req_if,
    // scheduling interface
    SCHED_IF.BK_CTRL            sched_if,

    // per-bank auto-refresh requests
    input   wire                ref_req_i,
    output  logic               ref_gnt_o
);

    localparam  S_IDLE = 3'b000, S_ACTIVATING = 3'b001, S_BANK_ACTIVE = 3'b010, S_READING = 3'b011, S_WRITING = 3'b100, 
                S_PRECHARGING = 3'b101, S_REFRESH = 3'b110;

    logic   [2:0]                   state, state_n;
    logic                           req_buf_valid, req_buf_valid_n;
    logic                           req_buf_wr;
    logic   [`AXI_ID_WIDTH-1:0]     req_buf_id;
    logic   [`AXI_LEN_WIDTH-1:0]    req_buf_len;
    logic   [`DRAM_RA_WIDTH-1:0]    req_buf_ra, cur_ra, cur_ra_n;
    logic   [`DRAM_CA_WIDTH-1:0]    req_buf_ca;

    wire    is_t_rc_met, is_t_rcd_met, is_t_rp_met, is_t_ras_met, is_t_rfc_met, is_t_ccdr_met, is_t_ccdw_met,
            is_t_rtp_met, is_t_wtp_met, is_row_open_met, is_t_wtr_met, is_t_rtw_met;

    localparam REFRESH_INTERVAL = 3000;
    logic auto_ref_req;
    reg ref_req_pending;

    reg [$clog2(REFRESH_INTERVAL)-1:0] ref_cntr;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) ref_cntr <= '0;
        else if(ref_cntr == REFRESH_INTERVAL - 1 || sched_if.ref_gnt) ref_cntr <= '0;
        else ref_cntr <= ref_cntr + 1'b1;
    end 
    assign auto_ref_req = (ref_cntr == REFRESH_INTERVAL - 1);
    
    always_ff @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            ref_req_pending <= 1'b0;
        end else begin
            if(sched_if.ref_gnt)begin
                ref_req_pending <= 1'b0;
            end
            else if(auto_ref_req)begin
                ref_req_pending <= 1'b1;
            end
        end
    end



    // Sequential logic for next state
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state           <= S_IDLE;
            cur_ra          <= '0;
            req_buf_valid   <= 1'b0;
        end else begin
            state           <= state_n;
            cur_ra          <= cur_ra_n;
            req_buf_valid   <= req_buf_valid_n;
            if(req_if.valid && req_if.ready) begin
                req_buf_wr  <= req_if.wr;
                req_buf_id  <= req_if.id;
                req_buf_len <= req_if.len;
                req_buf_ra  <= req_if.ra;
                req_buf_ca  <= req_if.ca;
            end
        end
    end


    always_comb begin
        
        cur_ra_n                    = cur_ra;
        state_n                     = state;
        req_buf_valid_n             = req_buf_valid;

        req_if.ready                = ~req_buf_valid;
        ref_gnt_o                   = 1'b0;

        sched_if.act_gnt            = 1'b0;
        sched_if.rd_gnt             = 1'b0;
        sched_if.wr_gnt             = 1'b0;
        sched_if.pre_gnt            = 1'b0;
        sched_if.ref_gnt            = 1'b0;
        sched_if.ba                 = 'h0;  // bank 0
        sched_if.ra                 = 'hx;
        sched_if.ca                 = 'hx;
        sched_if.id                 = 'hx;
        sched_if.len                = 'hx;

        if(req_if.valid && req_if.ready) begin
            req_buf_valid_n = 1'b1;
        end

        case (state)
            S_IDLE:begin
                if(ref_req_pending) begin
                    sched_if.ref_gnt    = 1'b1;
                    state_n = S_REFRESH;
                end else if (req_buf_valid && is_t_rc_met) begin
                    sched_if.act_gnt    = 1'b1;
                    sched_if.ra         = req_buf_ra;
                    state_n = S_ACTIVATING;
                    cur_ra_n = req_buf_ra;
                end
            end

            S_ACTIVATING:begin
                if(is_t_rcd_met) begin
                    state_n = S_BANK_ACTIVE;
                end
            end

            S_BANK_ACTIVE:begin
                if(ref_req_pending && is_t_wtp_met && is_t_rtp_met && is_t_ras_met) begin      // REFRESH
                    state_n = S_PRECHARGING;
                    sched_if.pre_gnt = 1'b1;
                end else if(req_buf_valid && (req_buf_ra != cur_ra))begin                      // Row Miss
                    if(is_t_ras_met && is_t_wtp_met && is_t_rtp_met)begin
                        state_n = S_PRECHARGING;
                        sched_if.pre_gnt = 1'b1;
                    end
                end else if(req_buf_valid && req_buf_wr == 1 && is_t_rcd_met && is_t_rtw_met && is_t_ccdw_met)begin           // Write                                        // Row Hit
                    state_n = S_WRITING;
                    sched_if.wr_gnt =1'b1;
                    sched_if.ca = req_buf_ca;
                    sched_if.id = req_buf_id;
                    sched_if.len = req_buf_len;
                    req_buf_valid_n = 1'b0;
                end else if(req_buf_valid && req_buf_wr == 0 && is_t_rcd_met & is_t_wtr_met && is_t_ccdr_met)begin
                    state_n = S_READING;
                    sched_if.rd_gnt =1'b1;
                    sched_if.ca = req_buf_ca;
                    sched_if.id = req_buf_id;
                    sched_if.len = req_buf_len;
                    req_buf_valid_n = 1'b0;
                end
            end

            S_READING:begin
                state_n = S_BANK_ACTIVE;
            end

            S_WRITING:begin
                state_n = S_BANK_ACTIVE;
            end

            S_PRECHARGING:begin
                if(is_t_rp_met)begin
                    state_n = S_IDLE;
                end
            end

            S_REFRESH:begin
                if(is_t_rfc_met)begin
                    state_n = S_IDLE;
                end
            end
        endcase
    end


    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RC_WIDTH)) u_rc_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.act_gnt),.reset_value_i(timing_if.t_rc_m1),.is_zero_o(is_t_rc_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RCD_WIDTH)) u_rcd_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.act_gnt),.reset_value_i(timing_if.t_rcd_m1),.is_zero_o(is_t_rcd_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RP_WIDTH)) u_rp_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.pre_gnt),.reset_value_i(timing_if.t_rp_m1),.is_zero_o(is_t_rp_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RAS_WIDTH)) u_ras_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.act_gnt),.reset_value_i(timing_if.t_ras_m1),.is_zero_o(is_t_ras_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RTP_WIDTH)) u_rtp_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.rd_gnt),.reset_value_i(timing_if.t_rtp_m1),.is_zero_o(is_t_rtp_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_WTP_WIDTH)) u_wtp_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.wr_gnt),.reset_value_i(timing_if.t_wtp_m1),.is_zero_o(is_t_wtp_met)); // tWR is managed by tWTP
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`ROW_OPEN_WIDTH)) u_row_open_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.act_gnt),.reset_value_i(timing_if.row_open_cnt),.is_zero_o(is_row_open_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RFC_WIDTH)) u_rfc_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.ref_gnt),.reset_value_i(timing_if.t_rfc_m1),.is_zero_o(is_t_rfc_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_WTR_WIDTH)) u_wtr_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.wr_gnt),.reset_value_i(timing_if.t_wtr_m1),.is_zero_o(is_t_wtr_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_RTW_WIDTH)) u_rtw_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.rd_gnt),.reset_value_i(timing_if.t_rtw_m1),.is_zero_o(is_t_rtw_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_CCD_WIDTH)) u_ccdr_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.rd_gnt),.reset_value_i(timing_if.t_ccd_m1),.is_zero_o(is_t_ccdr_met));
    SAL_TIMING_CNTR #(.CNTR_WIDTH(`T_CCD_WIDTH)) u_ccdw_cnt(.clk(clk),.rst_n(rst_n),.reset_cmd_i(sched_if.wr_gnt),.reset_value_i(timing_if.t_ccd_m1),.is_zero_o(is_t_ccdw_met));


endmodule // SAL_BK_CTRL
