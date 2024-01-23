#!/usr/bin/env bash

if [ -z $TARGET_LIB ]; then
  cat << EOT1
  ****************
  *** Do not use this architecture directly, please use
  *** one of its derivates:
  ***    nucleo_l432kc
  ***    nucleo_l496zg
  *********************
EOT1
exit
fi

export ARCH=stm32l4         #used to select compile directories
export ST_LINK="v2_1"

#---------------------------------------------- from here the same for all sub architectures

SRCS[1]="${RODOS_SRC}/bare-metal-generic"
SRCS[2]="${RODOS_SRC}/bare-metal/${ARCH}/hw"
SRCS[3]="${RODOS_SRC}/bare-metal/${ARCH}/cortex_m"
SRCS[4]="${RODOS_SRC}/bare-metal/${ARCH}/peripheral_ctrl"
SRCS[5]="${RODOS_SRC}/bare-metal/${ARCH}/peripheral_ctrl/clock_management"
SRCS[6]="${RODOS_SRC}/bare-metal/${ARCH}/peripheral_ctrl/gpio"
SRCS[7]="${RODOS_SRC}/bare-metal/${ARCH}/hal"
SRCS[8]="${RODOS_SRC}/bare-metal/${ARCH}/hw_hal"
SRCS[9]="${RODOS_SRC}/bare-metal/${ARCH}/hw_hal/gpio"
SRCS[10]="${RODOS_SRC}/bare-metal/${ARCH}/hw_hal/can"
SRCS[11]="${RODOS_SRC}/bare-metal/${ARCH}/minimal_sanitizer/"

SRC_FILES[1]="${RODOS_SRC}/bare-metal/${ARCH}/startup/call_constructors.c"
SRC_FILES[2]="${RODOS_SRC}/bare-metal/${ARCH}/startup/weak_irq_handlers.c"
SRC_FILES[3]="${RODOS_SRC}/bare-metal/${ARCH}/startup/startup_${MCU}.s"

export INCLUDES=${INCLUDES}" -I ${RODOS_SRC}/bare-metal/${ARCH} \
    -I ${RODOS_SRC}/bare-metal/${ARCH}/api_includes \
    -I ${RODOS_SRC}/bare-metal/${ARCH}/subtargets/boards/${BOARD} "

export INCLUDES_TO_BUILD_LIB=" -I ${RODOS_SRC}/bare-metal-generic  \
    -I ${RODOS_SRC}/bare-metal/${ARCH}/peripheral_ctrl/mcu_specific/${MCU} "

export CFLAGS_BASICS_COMMON=" -g3 -gdwarf-2"

export CFLAGS_BASICS="${CFLAGS_BASICS_COMMON}"
export HWCFLAGS=" -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16"
export LINKFLAGS=" -T${LINKER_SCRIPT} -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -L${RODOS_LIBS}/${TARGET_LIB} -fno-unwind-tables -fno-asynchronous-unwind-tables -lrodos -lm"
export CFLAGS=${CFLAGS}" ${CFLAGS_BASICS} ${HWCFLAGS} "
export CPPFLAGS=${CPPFLAGS}" --std=c++20"

#export ARM_TOOLS="/opt/arm-tools/bin/"
export ARM_TOOLS=""

export CPP_COMP="${CXX:-${ARM_TOOLS}arm-none-eabi-g++} "
export C_COMP="${CC:-${ARM_TOOLS}arm-none-eabi-gcc} " # only to compile BSP and Drivers from chip provider
export AR="${AR:-${ARM_TOOLS}arm-none-eabi-ar} "
