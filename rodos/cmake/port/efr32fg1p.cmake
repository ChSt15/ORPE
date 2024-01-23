if(NOT board)
  message(FATAL_ERROR "Do not use efr32fg1p directly; instead, use a board port file.")
endif()

set(port_dir "bare-metal/efr32fg1p")
set(is_port_baremetal TRUE)

set(RODOS_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

add_link_options(-nostdlib)

set(compile_definitions CORTEXM3 CORTEXM3_EFR32_MICRO CORTEXM3_EFR32 ${SUB_ARCH_FLAGS})
set(compile_and_link_options -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16)
set(compile_options ${compile_and_link_options} -nostdlib -gdwarf-2 -mthumb)
set(link_options ${compile_and_link_options} -nostartfiles -nostdlib -Xlinker
    --gc-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -lm)
set(libraries_to_link ${RAIL_LIB}  -lm $<TARGET_PROPERTY:rodos,BINARY_DIR>/librodos.a)
set(directories_to_link $<BUILD_INTERFACE:${RODOS_DIR}/src/bare-metal/efr32fg1p/>)


set(sources_to_add
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/emlib/src/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/${SUB_ARCH_DIR}/Source/GCC/*.S
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/${SUB_ARCH_DIR}/Source/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/sleeptimer/src/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/common/src/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/common/bsp/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/common/drivers/used/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/rail_lib/hal/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/rail_lib/hal/efr32/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/rail-config/efr32fg12p/*.c
    ${RODOS_DIR}/src/bare-metal/efr32fg1p/emdrv/src/*.c
    )

set(directories_to_include
    /src/bare-metal/efr32fg1p/CMSIS/Include
    /src/bare-metal/efr32fg1p/${SUB_ARCH_DIR}/Include
    /src/bare-metal/efr32fg1p/common/bsp
    /src/bare-metal/efr32fg1p/common/drivers/used
    /src/bare-metal/efr32fg1p/common/inc
    /src/bare-metal/efr32fg1p/config
    /src/bare-metal/efr32fg1p/emlib/inc
    /src/bare-metal/efr32fg1p/hal-config
    /src/bare-metal/efr32fg1p/rail-config
    /src/bare-metal/efr32fg1p/rail_lib/chip/efr32/efr32xg1x
    /src/bare-metal/efr32fg1p/rail_lib/common
    /src/bare-metal/efr32fg1p/rail_lib/hal
    /src/bare-metal/efr32fg1p/rail_lib/hal/efr32
    /src/bare-metal/efr32fg1p/sleeptimer/config
    /src/bare-metal/efr32fg1p/sleeptimer/inc
    /src/bare-metal/efr32fg1p/emdrv/inc
    )
