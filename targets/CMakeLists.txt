# Allows user to select a target board to build. 
# Note that the toolchain directory must also be set in the root CMakePresets.json

set(TARGET_BOARD "unknown" CACHE STRING "Target microcontroller architecture (see ./targets directory for implementations)")
set_property(CACHE TARGET_BOARD PROPERTY STRINGS "Nucleo-h723zg")

if (TARGET_BOARD STREQUAL "Nucleo-h723zg")
    message("Selected target board: Nucleo-h723zg")

    add_subdirectory(Nucleo-h723zg/cmake/stm32cubemx)

    set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "")

    add_library(freertos_config INTERFACE)
    target_include_directories(freertos_config INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Inc
    )

        
    # Mark cubemx as a system header to avoid warnings when including it
    get_target_property(CUBEMX_INCLUDES stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${CMAKE_PROJECT_NAME} SYSTEM PRIVATE ${CUBEMX_INCLUDES})
    target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx freertos_kernel)

    # Disable warnings for items in targets
    get_target_property(SOURCE_LIST ${CMAKE_PROJECT_NAME} SOURCES)
    set_source_files_properties(${SOURCE_LIST} PROPERTIES COMPILE_OPTIONS "-w")
else()
    message(FATAL_ERROR "Unknown target board: ${TARGET_BOARD}")
endif()



