set(board nucleo_l496zg)
set(mcu STM32L496xx)
set(RODOS_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
set(linker_script ${RODOS_DIR}/src/bare-metal/stm32l4/scripts/stm32l496zg.ld)

include(${CMAKE_CURRENT_LIST_DIR}/stm32l4.cmake)
