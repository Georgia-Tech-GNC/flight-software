#[[####################################################################
RocketLib/CMakeLists.txt

This file handles the RocketLib portion of the codebase.
It is NOT intended to be called independently, but is rather ran
via an "add_subdirectory" directive from the base CMakeLists.txt file.
Project settings like the C standard and the project name are 
inherited from the base CMakeLists.txt file as well.
#]]####################################################################

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE 
    utilities
    core
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    core/rocketlib.c
)