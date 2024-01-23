#! /bin/bash

# Author Erik Dilger (uni Wuerzburg), June 2020

export TARGET_LIB=gecko  #used as name for the generated lib

export SUB_ARCH=efr32fg1p
export SUB_ARCH_DIR=EFR32FG1P
export SUB_ARCH_FLAGS="-DEFR32FG1P -DEFR32FG1P133F256GM48 -DEFR32_SERIES1_CONFIG1_MICRO"
export RAIL_LIB="rail_efr32xg1_gcc_release"


source $RODOS_VARS_DIR/efr32fg1p.sh