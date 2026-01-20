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


    /*
    * FILL YOUR CODES HERE
    */



endmodule // SAL_SCHED
