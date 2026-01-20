`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

module SAL_BK_CTRL
#(
    parameter                   BK_ID   = 0
)
(
    // clock & reset
    input                       clk,
    input                       rst_n,

    // timing parameters
    TIMING_IF.MON               timing_if,

    // request from the address decoder
    REQ_IF.DST                  req_if,

    // request to the scheduler
    output  bk_req_t            bk_reqs,
    input   bk_gnt_t            bk_gnts,
    
    // per-bank auto-refresh requests
    input   wire                ref_req_i,
    output  logic               ref_gnt_o
);



    /*
    * FILL YOUR CODES HERE
    */




endmodule // SAL_BK_CTRL
