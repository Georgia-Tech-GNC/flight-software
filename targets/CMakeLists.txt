# Allows user to select a target board to build. 
# Note that the toolchain directory must also be set in the root CMakePresets.json

set(TARGET_BOARD "unknown" CACHE STRING "Target microcontroller architecture (see ./targets directory for implementations)")
set_property(CACHE TARGET_BOARD PROPERTY STRINGS "Nucleo-h723zg" "fc-h725ve")

find_program(UV_EXECUTABLE uv REQUIRED)

if (TARGET_BOARD STREQUAL "Nucleo-h723zg")
    message("Selected target board: Nucleo-h723zg")

    add_subdirectory(Nucleo-h723zg/cmake/stm32cubemx)

    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Src/port.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Src/isr.c
    )

    set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "")

    add_library(freertos_config INTERFACE)
    target_include_directories(freertos_config INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Inc
    )
    set(CMSISCORE "${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Drivers/CMSIS" PARENT_SCOPE)

    add_custom_target(autogen_platform
        COMMAND ${UV_EXECUTABLE} run --project "${CMAKE_SOURCE_DIR}" 
            python "${CMAKE_SOURCE_DIR}/scripts/codegen/codegen.py" "targets/Nucleo-h723zg" 
        BYPRODUCTS
            ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Src/isr.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Inc/isr.h
            ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Inc/port.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Inc/port.h
            ${CMAKE_CURRENT_SOURCE_DIR}/Nucleo-h723zg/Core/Src/stm32h7xx_it.h
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Autogenerating platform..."
        VERBATIM
    )

    add_custom_target(renode
        COMMAND renode --ui
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
elseif (TARGET_BOARD STREQUAL "fc-h725ve")
    message("Selected target board: fc-h725ve")

    add_subdirectory(fc-h725ve/cmake/stm32cubemx)

    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Src/port.c
        ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Src/isr.c
    )

    set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "")

    add_library(freertos_config INTERFACE)
    target_include_directories(freertos_config INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Inc
    )
    set(CMSISCORE "${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Drivers/CMSIS" PARENT_SCOPE)

    add_custom_target(autogen_platform
        COMMAND ${UV_EXECUTABLE} run --project "${CMAKE_SOURCE_DIR}" 
            python "${CMAKE_SOURCE_DIR}/scripts/codegen/codegen.py" "targets/fc-h725ve" 
        BYPRODUCTS
            ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Src/isr.c
            ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Inc/isr.h
            ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Inc/port.c
            ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Inc/port.h
            ${CMAKE_CURRENT_SOURCE_DIR}/fc-h725ve/Core/Src/stm32h7xx_it.h
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Autogenerating platform..."
        VERBATIM
    )

    add_custom_target(renode
        COMMAND renode --ui
            -e "i @targets/fc-h725ve/init.resc"
            -e "sysbus LoadELF @${PROJECT_BINARY_DIR}/flight-software.elf"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${CMAKE_PROJECT_NAME}
        VERBATIM
    )

    add_custom_target(renode-test
        COMMAND rm -rf test-results/fc-h725ve
        COMMAND mkdir -p test-results/fc-h725ve
        COMMAND cd test-results/fc-h725ve && renode-test ${CMAKE_SOURCE_DIR}/renode/tests/test.robot
            --variable FIRMWARE_ELF:${PROJECT_BINARY_DIR}/flight-software.elf
            --variable PLATFORM_YAML:${CMAKE_SOURCE_DIR}/targets/fc-h725ve/platform.yaml
            --variable PROJECT_ROOT:${CMAKE_SOURCE_DIR}
            --variable INIT_RESC_FILE:${CMAKE_SOURCE_DIR}/targets/fc-h725ve/init.resc
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
