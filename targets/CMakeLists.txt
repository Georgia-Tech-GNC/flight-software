# Allows user to select a target board to build. 
# Note that the toolchain directory must also be set in the root CMakePresets.json

set(TARGET_BOARD "unknown" CACHE STRING "Target microcontroller architecture (see ./targets directory for implementations)")
set_property(CACHE TARGET_BOARD PROPERTY STRINGS "Nucleo-h723zg" "Nucleo-f429zi")

find_program(UV_EXECUTABLE uv REQUIRED)

if (TARGET_BOARD STREQUAL "Nucleo-h723zg")
    message("Selected target board: Nucleo-h723zg")

    add_subdirectory(Nucleo-h723zg/cmake/stm32cubemx)

    set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "")

    add_library(freertos_config INTERFACE)
    target_include_directories(freertos_config INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Inc
    )

    add_custom_target(autogen_platform
        COMMAND ${UV_EXECUTABLE} run --project "${CMAKE_SOURCE_DIR}" 
            python "${CMAKE_SOURCE_DIR}/scripts/codegen/codegen.py" "targets/Nucleo-h723zg" 
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Autogenerating platform..."
        VERBATIM
    )

    add_custom_target(renode
        COMMAND renode
            -e "i @targets/Nucleo-h723zg/init.resc"
            -e "sysbus LoadELF @${PROJECT_BINARY_DIR}/flight-software.elf"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${CMAKE_PROJECT_NAME}
        VERBATIM
    )

    add_custom_target(renode-test
        COMMAND rm -rf test-results/Nucleo-h723zg
        COMMAND mkdir -p test-results/Nucleo-h723zg
        COMMAND cd test-results/Nucleo-h723zg && renode-test ${CMAKE_SOURCE_DIR}/renode/tests/test.robot
            --variable FIRMWARE_ELF:${PROJECT_BINARY_DIR}/flight-software.elf
            --variable PLATFORM_YAML:${CMAKE_SOURCE_DIR}/targets/Nucleo-h723zg/platform.yaml
            --variable PROJECT_ROOT:${CMAKE_SOURCE_DIR}
            --variable INIT_RESC_FILE:${CMAKE_SOURCE_DIR}/targets/Nucleo-h723zg/init.resc
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${CMAKE_PROJECT_NAME}
        VERBATIM
    )
elseif (TARGET_BOARD STREQUAL "Nucleo-f429zi")
    message("Selected target board: Nucleo-f429zi")

    add_subdirectory(Nucleo-f429zi/cmake/stm32cubemx)

    set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "")

    add_library(freertos_config INTERFACE)
    target_include_directories(freertos_config INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-f429zi/Core/Inc
    )

    add_custom_target(autogen_platform
        COMMAND ${UV_EXECUTABLE} run --project "${CMAKE_SOURCE_DIR}" 
            python "${CMAKE_SOURCE_DIR}/scripts/codegen/codegen.py" "targets/Nucleo-f429zi" 
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Autogenerating platform..."
        VERBATIM
    )

    add_custom_target(renode
        COMMAND renode 
            -e "i @targets/Nucleo-f429zi/init.resc" 
            -e "sysbus LoadELF @${PROJECT_BINARY_DIR}/flight-software.elf"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${CMAKE_PROJECT_NAME}
        VERBATIM
    )

    add_custom_target(renode-test
        COMMAND rm -rf test-results/Nucleo-f429zi
        COMMAND mkdir -p test-results/Nucleo-f429zi
        COMMAND cd test-results/Nucleo-f429zi && renode-test ${CMAKE_SOURCE_DIR}/renode/tests/test.robot
            --variable FIRMWARE_ELF:${PROJECT_BINARY_DIR}/flight-software.elf
            --variable PLATFORM_YAML:${CMAKE_SOURCE_DIR}/targets/Nucleo-f429zi/platform.yaml
            --variable PROJECT_ROOT:${CMAKE_SOURCE_DIR}
            --variable INIT_RESC_FILE:${CMAKE_SOURCE_DIR}/targets/Nucleo-f429zi/init.resc
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${CMAKE_PROJECT_NAME}
        VERBATIM
    )
else()
    message(FATAL_ERROR "Unknown target board: ${TARGET_BOARD}")
endif()

# Mark cubemx as a system header to avoid warnings when including it
get_target_property(CUBEMX_INCLUDES stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${CMAKE_PROJECT_NAME} SYSTEM PRIVATE ${CUBEMX_INCLUDES})
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx freertos_kernel)

# Disable warnings for items in targets
get_target_property(SOURCE_LIST ${CMAKE_PROJECT_NAME} SOURCES)
set_source_files_properties(${SOURCE_LIST} PROPERTIES COMPILE_OPTIONS "-w")

add_dependencies(${CMAKE_PROJECT_NAME} autogen_platform)
