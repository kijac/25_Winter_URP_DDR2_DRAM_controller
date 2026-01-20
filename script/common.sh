#!/bin/bash

N=$1 # '1' or '2'

export ROOT_PATH="$PWD/.."
RUN_DIR="${ROOT_PATH}/script/OUTPUT_STEP${N}"

FILELIST="${ROOT_PATH}/script/filelist_step${N}.f"

COMPILE_CMD='vcs'
COMPILE_OPTIONS='-full64 -sverilog -debug_access+all -kdb +lint=PCWM -LDFLAGS -Wl,--no-as-needed'
COMPILE_INCDIR="+incdir+${ROOT_PATH}/STEP${N}/RTL+${ROOT_PATH}/STEP${N}/SIM/DDRPHY+${ROOT_PATH}/STEP${N}/SIM/DRAM+${ROOT_PATH}/STEP${N}/SIM/TB"

SIM_OPTIONS=''

#VERDI_CMD='Verdi-SX'
VERDI_CMD='Verdi'
VERDI_OPTIONS='-sverilog'

DC_CMD='dc_shell-xg-t'
DC_OPTIONS=''

CSR_CMD='/home/ScalableArchiLab/bin/csrCompileLite'
