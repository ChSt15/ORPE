cmake_minimum_required(VERSION 3.0.0)

project(COMMS VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(rodos)

#file(GLOB_RECURSE SRC_FILES src/*.cpp)
#file(GLOB_RECURSE LIB_FILES lib/*.cpp)



target_include_directories(COMMS PRIVATE comms/Datalink.h ipc/ipcRODOS.h include/Datastruct.h)

add_executable(COMMS comms/Datalink.cpp ipc/ipcRODOS.cpp)
 
target_link_libraries(COMMS PUBLIC rodos)




project(ORPE VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
set (CMAKE_MODULE_PATH "/usr/local/lib/cmake/${CMAKE_MODULE_PATH}")
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE LIB_FILES lib/*.cpp)



target_include_directories(ORPE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ipc/ipcORPE.h)
target_include_directories(ORPE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})

add_executable(ORPE ${SRC_FILES} ipc/ipcORPE.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${raspicam_INCLUDE_DIRS})
target_link_libraries(ORPE PUBLIC -llccv ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
