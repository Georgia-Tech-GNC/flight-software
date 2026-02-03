#[[####################################################################
CMakeLists.txt

This is the base CMakeLists.txt file and handles the CMake toolchain
responsible for compiling all projects.
Compiling instructions can be found in the appropriate section in 
README.md.
#]]####################################################################

#[[##########################################################
PART 1: BASE CMAKE SETUP
#]]##########################################################
cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_PROJECT_NAME flight_software)
add_executable(${CMAKE_PROJECT_NAME})

#[[##########################################################
PART 2: GCC-ARM-EMBEDDED TOOLCHAIN AND TARGET HARDWARE SETUP
#]]##########################################################
# This needs to be "include" instead of "add_subdirectory"
# because the gcc-arm-embedded compiler settings need to 
# hold for all future target sources 
include(Targets/CMakeLists.txt)

# Enable compile command to ease indexing with e.g. clangd 
# and support for ASM and C languages
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
project(${CMAKE_PROJECT_NAME} C ASM)
enable_language(C ASM)

# Set compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -Werror")

#[[##########################################################
PART 3: SUBDIRECTORY INCLUSIONS (FreeRTOS, RocketLib, etc.)
#]]##########################################################
add_subdirectory(RocketLib)
add_subdirectory(Extensions)
add_subdirectory(FreeRTOS-Kernel)

#[[##########################################################
PART 4: Doxygen and documentation support
#]]##########################################################
find_package(Doxygen)
option(BUILD_DOXYGEN "Build doxygen documentation" ${DOXYGEN_FOUND})
if (BUILD_DOXYGEN)
    if (NOT DOXYGEN_FOUND)
        message(SEND_ERROR "Doxygen not found. Install doxygen or \
        turn off the BUILD_DOCUMENTATION flag")
    endif()
    configure_file(Doxyfile docs/Doxyfile @ONLY)
    add_custom_target(docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} docs/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM 
    )
endif()