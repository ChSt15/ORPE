#! /bin/bash

export MCU=STM32L496xx
export BOARD=nucleo_l496zg
export TARGET_LIB=${BOARD}
export LINKER_SCRIPT="${RODOS_SRC}/bare-metal/stm32l4/scripts/stm32l496zg.ld"

source $RODOS_VARS_DIR/stm32l4.sh
