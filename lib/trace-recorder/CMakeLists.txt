cmake_minimum_required(VERSION 3.22)
project(trace-recorder)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Trace recorder core module
add_library(trace_recorder_core STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/trace_recorder.c
)
target_include_directories(trace_recorder_core 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)