
############################################## CMSIS-DSP #############################################
if(NOT DEFINED CMSISCORE)
    message(FATAL_ERROR
        "In order to use CMSIS-DSP, the target must define CMSISCORE! "
        "Check targets/CMakeLists.txt and make sure that the CMSISCORE "
        "cmake variable is being set correctly, usually targets/.../Drivers/CMSIS"
    )
endif()
set(DISABLEFLOAT16 ON) # Our targets don't support hardware 16-bit floating point computation
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP/Source)

# Mark CMSIS-DSP headers as system headers to avoid compiler warnings on those files
get_target_property(CMSISDSP_INCLUDE_DIRS CMSISDSP INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(CMSISDSP SYSTEM INTERFACE ${CMSISDSP_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME} CMSISDSP)

############################################## FreeRTOS ##############################################
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)

########################################### Ground Station ###########################################
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ground-station)
target_link_libraries(${CMAKE_PROJECT_NAME} ground_station_core)

############################################# nanoprintf #############################################
# Nanoprintf is an stb-style header-only library, so we need to add a source file to the build that 
# loads the header in "implementation" mode (see two commands below)
set(NANOPRINTF_C_FILE "${CMAKE_CURRENT_BINARY_DIR}/nanoprintf_impl.c")
file(GENERATE OUTPUT "${NANOPRINTF_C_FILE}" CONTENT "
#define NANOPRINTF_IMPLEMENTATION 
#include \"nanoprintf.h\"
")
add_library(nanoprintf STATIC "${NANOPRINTF_C_FILE}")
target_include_directories(nanoprintf SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/nanoprintf)
target_compile_definitions(nanoprintf PUBLIC
    NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS=1
    NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS=1

    NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS=1
    NANOPRINTF_USE_FLOAT_HEX_FORMAT_SPECIFIER=0
    NANOPRINTF_USE_FLOAT_SCI_FORMAT_SPECIFIER=0
    NANOPRINTF_USE_FLOAT_SHORTEST_FORMAT_SPECIFIER=0

    NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS=1
    NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS=1
    NANOPRINTF_USE_FIXED_WIDTH_FORMAT_SPECIFIERS=0
    NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS=1
    
    NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS=0
    NANOPRINTF_USE_ALT_FORM_FLAG=0
    NANOPRINTF_USE_FLOAT_SINGLE_PRECISION=1    
    NANOPRINTF_USE_DIVISION_FREE_CONVERSION=0
)
target_link_libraries(${CMAKE_PROJECT_NAME} nanoprintf)


############################################### renode ###############################################

set(RENODE_DIR "${CMAKE_SOURCE_DIR}/lib/renode")
set(RENODE_BIN_DIR "${RENODE_DIR}/output/bin/Release")

ExternalProject_Add(renode_external
    SOURCE_DIR "${RENODE_DIR}"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND bash build.sh --ui
    BUILD_IN_SOURCE TRUE
    BUILD_ALWAYS FALSE
    INSTALL_COMMAND ""
    EXCLUDE_FROM_ALL TRUE
    USES_TERMINAL_BUILD TRUE
)

add_custom_target(renode-package
    COMMAND ${CMAKE_COMMAND} -E env
        "RENODE_SRC=${RENODE_DIR}"
        "RENODE_BIN=${RENODE_BIN_DIR}"
        nix build --impure ".#renode"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    DEPENDS renode_external
    USES_TERMINAL
    VERBATIM
)

add_custom_target(renode
    COMMAND "${CMAKE_SOURCE_DIR}/result/bin/renode" --ui
        -e "i @targets/${TARGET_BOARD}/init.resc"
        -e "sysbus LoadELF @${PROJECT_BINARY_DIR}/flight-software.elf"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    DEPENDS ${CMAKE_PROJECT_NAME} renode-package
    USES_TERMINAL
    VERBATIM
)

set(TEST_RESULTS_DIR "${CMAKE_SOURCE_DIR}/test-results/${TARGET_BOARD}")

file(MAKE_DIRECTORY "${TEST_RESULTS_DIR}")

add_custom_target(renode-test
    COMMAND "${CMAKE_SOURCE_DIR}/result/bin/renode-test"
        "${CMAKE_SOURCE_DIR}/renode/tests/test.robot"
        --variable "FIRMWARE_ELF:${PROJECT_BINARY_DIR}/flight-software.elf"
        --variable "PLATFORM_YAML:${CMAKE_SOURCE_DIR}/targets/${TARGET_BOARD}/platform.yaml"
        --variable "PROJECT_ROOT:${CMAKE_SOURCE_DIR}"
        --variable "INIT_RESC_FILE:${CMAKE_SOURCE_DIR}/targets/${TARGET_BOARD}/init.resc"
    WORKING_DIRECTORY "${TEST_RESULTS_DIR}"
    DEPENDS ${CMAKE_PROJECT_NAME} renode-package
    USES_TERMINAL
    VERBATIM
)
