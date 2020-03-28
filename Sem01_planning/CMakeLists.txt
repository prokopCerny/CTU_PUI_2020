# cmake_minimum_required(VERSION <specify CMake version here>)
project(Sem01_planning)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DMY_DEBUG")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DTIME_EXECUTION")

set(CMAKE_C_STANDARD 99)

add_executable(Sem01_planning main.cpp problem.h problem.c)