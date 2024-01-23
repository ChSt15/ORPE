#! /bin/bash

export MCU=STM32L432xx
export BOARD=nucleo_l432kc
export TARGET_LIB=${BOARD}
export LINKER_SCRIPT="${RODOS_SRC}/bare-metal/stm32l4/scripts/stm32l432kc.ld"

source $RODOS_VARS_DIR/stm32l4.sh
