cmake_minimum_required(VERSION 3.20)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(
    rodos
    VERSION 0.1.0
    LANGUAGES C CXX ASM
)


if (COVERAGE)
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Debug")
        message(STATUS "Coverage enabled, setting build type to Debug")
    endif ()
    include(cmake/CodeCoverage.cmake)
    append_coverage_compiler_flags()
endif ()

if (NOT DEFINED port_dir)
    message(FATAL_ERROR "port not correct (CMAKE_TOOLCHAIN_FILE, see readme)")
endif ()

#___________________________________________________________________
file(GLOB_RECURSE independent_sources
    src/independent/*.cpp
    support/support-libs/*.cpp)

add_library(rodos_rodos STATIC ${independent_sources})
add_library(rodos::rodos ALIAS rodos_rodos)
add_library(rodos ALIAS rodos_rodos)
set_target_properties(rodos_rodos PROPERTIES
    EXPORT_NAME rodos
    OUTPUT_NAME rodos
)

file(GLOB port_sources
    src/${port_dir}/*.cpp
    src/${port_dir}/*.c
    src/${port_dir}/hal/*.cpp
    src/${port_dir}/*.S)
target_sources(rodos_rodos PRIVATE ${port_sources})

target_include_directories(rodos_rodos PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/api>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/api/hal>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${port_dir}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${port_dir}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/support/support-libs>
)



target_compile_options(rodos_rodos PUBLIC -g -Wall -Wpedantic -Wextra -Wcast-qual -Wconversion
    -Wsign-conversion -Wfloat-conversion -Wdouble-promotion -Wnull-dereference -Wstrict-aliasing -Wno-long-long
    -Wno-cpp)
target_compile_features(rodos_rodos PUBLIC cxx_std_20)

option(ENABLE_RODOS_RTTI "Build RODOS with run-time type information" OFF)
if (NOT ENABLE_RODOS_RTTI)
    target_compile_options(rodos_rodos PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
endif ()


#___________________________________________________________________
if (is_port_baremetal)
    file(GLOB baremetal_generic_sources src/bare-metal-generic/*.cpp)
    target_sources(rodos_rodos PRIVATE ${baremetal_generic_sources})
    target_include_directories(rodos_rodos PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bare-metal-generic>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bare-metal-generic/ip/lwip/include>)

    option(SLEEP_WHEN_IDLE_ENABLED "Go into sleep mode when idling" OFF)
    if(SLEEP_WHEN_IDLE_ENABLED)
        add_compile_definitions(SLEEP_WHEN_IDLE_ENABLED)
    endif()
endif ()


#___________________________________________________________________
#    optional extra files which are port dependent. See cmake/port/xxx.cmake
#___________________________________________________________________
if (compile_definitions)
  target_compile_definitions(rodos_rodos PUBLIC ${compile_definitions})
endif ()

if (compile_options)
  target_compile_options(rodos_rodos PUBLIC ${compile_options})
endif ()

if (link_options)
  target_link_options(rodos_rodos PUBLIC ${link_options})
endif ()

if (sources_to_add)
    file(GLOB files ${sources_to_add})
    target_sources(rodos_rodos PRIVATE ${files})
endif ()


if (directories_to_include)
    foreach(dir ${directories_to_include})
        target_include_directories(rodos_rodos PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${dir}>)
    endforeach()
endif ()

if (directories_to_link)
  target_link_directories(rodos_rodos PUBLIC ${directories_to_link})
endif ()

if (libraries_to_link)
    target_link_libraries(rodos_rodos PUBLIC ${libraries_to_link})
endif ()

if (linker_script)
  target_link_options(rodos_rodos PUBLIC -T${linker_script})
endif()

if (EXECUTABLE)
    include(cmake/executable.cmake)
    add_subdirectory(tutorials)
endif ()


# Install rules
include(cmake/install-rules.cmake)


if (COVERAGE)
    setup_lcov_coverage(
        NAME coverage
        LCOV_ARGS "--no-external")
    message("Generate coverage report with 'make coverage_collect'")
    add_custom_command(TARGET coverage_collect POST_BUILD
        COMMAND sed -ie "s@SF:${CMAKE_CURRENT_SOURCE_DIR}/\\(.*\\)@SF:\\1@g"
        ${CMAKE_CURRENT_BINARY_DIR}/coverage.info
        COMMENT "Changing paths in coverage.info to relative paths."
        VERBATIM
        )
endif ()
enable_testing()
add_subdirectory(test-suite)

if (UNIT_TESTS)
  add_subdirectory(support/test)
endif()
