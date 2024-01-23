set(board nucleo_l432kc)
set(mcu STM32L432xx)
set(RODOS_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
set(linker_script ${RODOS_DIR}/src/bare-metal/stm32l4/scripts/stm32l432kc.ld)

include(${CMAKE_CURRENT_LIST_DIR}/stm32l4.cmake)
