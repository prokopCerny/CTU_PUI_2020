cmake_minimum_required(VERSION 3.10)
project(Sem01_planning)
include(CheckCXXCompilerFlag)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DMY_DEBUG")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DTIME_EXECUTION")

set(CMAKE_C_STANDARD 99)

check_cxx_compiler_flag(-Wall temp)
if(temp)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()
check_cxx_compiler_flag(-pedantic temp)
if(temp)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
endif()

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=undefined -fsanitize=integer -g3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined -fsanitize=integer -g3")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined -fsanitize=integer -g3")

add_executable(Sem01_planning problem.h problem.c lmcut.cpp)