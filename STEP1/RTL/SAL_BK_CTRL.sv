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


    /*
    * FILL YOUR CODES HERE
    */


    always_comb begin
        
        cur_ra_n                    = cur_ra;
        state_n                     = state;

        ref_gnt_o                   = 1'b0;
        req_if.ready                = 1'b0;

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

    end
    

endmodule // SAL_BK_CTRL
