set(board peb1_100mhz) # clock frequency options are: 10/20/40/80/100
set(RODOS_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
set(linker_script ${RODOS_DIR}/src/bare-metal/va41620/scripts/linkerscript.ld)

set(port_dir "bare-metal/va41620")
set(is_port_baremetal TRUE)

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

set(compile_and_link_options
    -mcpu=cortex-m4
    -mfloat-abi=softfp
    -mfpu=fpv4-sp-d16
    -ffreestanding)
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
    ${RODOS_DIR}/src/bare-metal/va41620/hw/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/startup/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/cortex_m/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/peripheral_ctrl/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/peripheral_ctrl/clock_management/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/peripheral_ctrl/pin_config/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/hal/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/hw_hal/*.cpp
    ${RODOS_DIR}/src/bare-metal/va41620/hw_hal/can/*.cpp
)

set(directories_to_include
    src/bare-metal/va41620/api_includes
    src/bare-metal/va41620/subtargets/${board}
)

set(libraries_to_link
    m
    gcc
)
