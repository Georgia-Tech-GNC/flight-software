cmake_minimum_required(VERSION 3.22)

################################## CMake Top-Level Project Settings ##################################
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(flight-software)
add_executable(${CMAKE_PROJECT_NAME} "")
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

set(WARNING_FLAGS "-Wall" "-Wextra" "-Wpedantic" "-Wshadow")
list(APPEND WARNING_FLAGS "-Wconversion" "-Wcast-align" "-Wcast-qual" "-Wdouble-promotion" "-Wsign-conversion")
list(APPEND WARNING_FLAGS "-Wformat=2" "-Wundef" "-Wstrict-prototypes" "-Wmissing-prototypes" "-Wpointer-arith")

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE ${WARNING_FLAGS})
enable_language(C ASM)

########################################## Build Submodules ##########################################
add_subdirectory(targets)
add_subdirectory(core)


# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
