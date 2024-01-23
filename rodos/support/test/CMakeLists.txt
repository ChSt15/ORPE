enable_testing()

include(GoogleTest)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

add_executable(support-libs-test
    allocableobjects-test.cpp)

target_include_directories(support-libs-test PUBLIC
    ../../api)

target_link_libraries(support-libs-test
    gtest
    gtest_main)
