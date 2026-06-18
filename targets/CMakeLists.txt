# Allows user to select a target board to build. 
# Note that the toolchain directory must also be set in the root CMakePresets.json

set(TARGET_BOARD "unknown" CACHE STRING "Target microcontroller architecture (see ./targets directory for implementations)")
set_property(CACHE TARGET_BOARD PROPERTY STRINGS "Nucleo-h723zg")

if (TARGET_BOARD STREQUAL "Nucleo-h723zg")
    message("Selected target board: Nucleo-h723zg")

    # Unfortunately, Much of the CubeMX-generated code triggers our compiler warnings.
    # Temporarily setting CMAKE_PROJECT_NAME is a workaround to put all of the cubemx code into a
    # set of targets instead of directly pushing it the executable, allowing us to disable warnings.
    set(REAL_CMAKE_PROJECT_NAME ${CMAKE_PROJECT_NAME})
    add_library(hardware_target)
    set(CMAKE_PROJECT_NAME hardware_target)

    add_subdirectory(Nucleo-h723zg/cmake/stm32cubemx)

    # Suppress all warnings in 3rd-party code
    target_compile_options(hardware_target PRIVATE -w)
    target_compile_options(STM32_Drivers PRIVATE -w)
    target_compile_options(FreeRTOS PRIVATE -w)
    target_compile_options(FatFs PRIVATE -w)

    # Restore CMAKE_PROJECT_NAME
    set(CMAKE_PROJECT_NAME ${REAL_CMAKE_PROJECT_NAME})

    # Mark cubemx as a system header to avoid warnings when including it
    get_target_property(CUBEMX_INCLUDES stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${CMAKE_PROJECT_NAME} SYSTEM PRIVATE ${CUBEMX_INCLUDES})

    # Link libraries
    target_link_libraries(${CMAKE_PROJECT_NAME} hardware_target)
    target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)


else()
    message(FATAL_ERROR "Unknown target board: ${TARGET_BOARD}")
endif()



