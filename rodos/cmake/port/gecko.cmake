set(board gecko)

set(SUB_ARCH efr32fg1p)
set(SUB_ARCH_DIR EFR32FG1P)
set(SUB_ARCH_FLAGS "EFR32FG1P;EFR32FG1P133F256GM48;EFR32_SERIES1_CONFIG1_MICRO")
set(RAIL_LIB "rail_efr32xg1_gcc_release")
set(linker_script ${CMAKE_CURRENT_LIST_DIR}/../../src/bare-metal/efr32fg1p/scripts/${SUB_ARCH}.ld)

include(${CMAKE_CURRENT_LIST_DIR}/efr32fg1p.cmake)
