#! /bin/bash

# Author Erik Dilger (uni Wuerzburg), June 2020

export TARGET_LIB=gecko1M  #used as name for the generated lib

export SUB_ARCH=efr32fg12p
export SUB_ARCH_DIR=EFR32FG12P
export SUB_ARCH_FLAGS="-DEFR32FG12P -DEFR32FG12P433F1024GL125 -DEFR32_SERIES1_CONFIG2_MICRO"
export RAIL_LIB="rail_efr32xg12_gcc_release"

source $RODOS_VARS_DIR/efr32fg1p.sh