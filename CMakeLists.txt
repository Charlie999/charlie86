cmake_minimum_required(VERSION 3.17)
project(charlie86)

set(CMAKE_CXX_STANDARD 14)

add_executable(charlie86 main.cpp types.h regs.h mr.h rm.h common.h video.cpp video.h)