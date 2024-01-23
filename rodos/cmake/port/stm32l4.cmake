if(NOT board)
    message(FATAL_ERROR "Do not use stm32l4 directly; instead, use a board port file.")
endif()

set(port_dir "bare-metal/stm32l4")
set(is_port_baremetal TRUE)

set(RODOS_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

if(NOT DEFINED linker_script)
    message(FATAL_ERROR "Linker script must be provided in board port file.")
endif()
if(NOT DEFINED mcu)
    message(FATAL_ERROR "MCU group name must be provided in board port file.")
endif()

set(compile_and_link_options
    -mcpu=cortex-m4
    -mfloat-abi=softfp
    -mfpu=fpv4-sp-d16)
add_compile_options(
    ${compile_and_link_options}
    -gdwarf-2
    -mthumb
)
add_link_options(
    ${compile_and_link_options}
    -T${linker_script}
    -nostartfiles
    -nodefaultlibs
    -nostdlib
    -Xlinker --gc-sections
    -fno-unwind-tables
    -fno-asynchronous-unwind-tables
)

set(sources_to_add
    ${RODOS_DIR}/src/bare-metal/stm32l4/hw/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/cortex_m/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/peripheral_ctrl/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/peripheral_ctrl/clock_management/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/peripheral_ctrl/gpio/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/hal/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/hw_hal/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/hw_hal/gpio/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/hw_hal/can/*.cpp
    ${RODOS_DIR}/src/bare-metal/stm32l4/startup/*.c
    ${RODOS_DIR}/src/bare-metal/stm32l4/startup/startup_${mcu}.s
    ${RODOS_DIR}/src/bare-metal/stm32l4/minimal_sanitizer/minimal_ubsan.S
)

set(directories_to_include
    src/bare-metal/stm32l4
    src/bare-metal/stm32l4/api_includes
    src/bare-metal/stm32l4/peripheral_ctrl/mcu_specific/${mcu}
    src/bare-metal/stm32l4/subtargets/boards/${board}
)

set(libraries_to_link
    m
)
