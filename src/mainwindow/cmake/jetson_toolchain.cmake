# Shared Embedded Linux aarch64 toolchain entry point.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CROSS_COMPILE aarch64-linux-gnu- CACHE STRING "Cross compiler prefix")
find_program(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc REQUIRED)
find_program(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++ REQUIRED)

set(JETSON_ROOT "" CACHE PATH "Target aarch64 sysroot")
if(JETSON_ROOT)
  set(CMAKE_SYSROOT "${JETSON_ROOT}")
  set(CMAKE_FIND_ROOT_PATH "${JETSON_ROOT}")
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Jetson Orin NX uses Arm Cortex-A78AE (ARMv8.2-A).
add_compile_options(-march=armv8.2-a -mcpu=cortex-a78)
