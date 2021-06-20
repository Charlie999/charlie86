cmake_minimum_required(VERSION 3.17)
project(charlie86)

IF(NOT WIN32)
    message("** Not WIN32, using MinGW stdthreads")
    option(MINGW_STDTHREADS_GENERATE_STDHEADERS "" ON)
ENDIF()

set(CMAKE_CXX_STANDARD 14)

add_executable(charlie86 main.cpp types.h regs.h mr.h rm.h common.h video.cpp video.h)

IF(NOT WIN32)
    add_subdirectory(mingw_stdthreads)
    target_link_libraries(charlie86 PRIVATE mingw_stdthreads)
    target_link_libraries(charlie86 PRIVATE gdi32)
    target_link_options(charlie86 PRIVATE -static-libgcc -static-libstdc++)
ENDIF()