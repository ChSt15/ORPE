#!/usr/bin/env bash

export ARCH=va41620         # used to select compile directories
export BOARD=peb1_100mhz    # clock frequency options are: 10/20/40/80/100 MHz
export TARGET_LIB=${ARCH}
export LINKER_SCRIPT="${RODOS_SRC}/bare-metal/va41620/scripts/linkerscript.ld"

SRCS[1]="${RODOS_SRC}/bare-metal-generic"
SRCS[2]="${RODOS_SRC}/bare-metal/${ARCH}/hw"
SRCS[3]="${RODOS_SRC}/bare-metal/${ARCH}/startup"
SRCS[4]="${RODOS_SRC}/bare-metal/${ARCH}/cortex_m"
SRCS[5]="${RODOS_SRC}/bare-metal/${ARCH}/peripheral_ctrl"
SRCS[6]="${RODOS_SRC}/bare-metal/${ARCH}/peripheral_ctrl/clock_management"
SRCS[7]="${RODOS_SRC}/bare-metal/${ARCH}/peripheral_ctrl/pin_config"
SRCS[8]="${RODOS_SRC}/bare-metal/${ARCH}/hal"
SRCS[9]="${RODOS_SRC}/bare-metal/${ARCH}/hw_hal"
SRCS[10]="${RODOS_SRC}/bare-metal/${ARCH}/hw_hal/can"

export INCLUDES=${INCLUDES}" -I ${RODOS_SRC}/bare-metal/${ARCH} \
    -I ${RODOS_SRC}/bare-metal/${ARCH}/api_includes \
    -I ${RODOS_SRC}/bare-metal/${ARCH}/subtargets/${BOARD} "

export INCLUDES_TO_BUILD_LIB=" -I ${RODOS_SRC}/bare-metal-generic"

export CFLAGS_BASICS=" -g3 -gdwarf-2"
export HWCFLAGS=" -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -ffreestanding"
export LINKFLAGS=" -T${LINKER_SCRIPT} -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -L${RODOS_LIBS}/${TARGET_LIB} -fno-unwind-tables -fno-asynchronous-unwind-tables -ffreestanding -lrodos -lm -lgcc"
export CFLAGS=${CFLAGS}" ${CFLAGS_BASICS} ${HWCFLAGS} "
export CPPFLAGS=${CPPFLAGS}" --std=c++20 -ffreestanding"

export ARM_TOOLS=""  # specify path to gcc arm toolchain to use version not in PATH
export CPP_COMP="${CXX:-${ARM_TOOLS}arm-none-eabi-g++} "
export AR="${AR:-${ARM_TOOLS}arm-none-eabi-ar} "
