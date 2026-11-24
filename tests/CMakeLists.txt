set(TEST_RESULTS_DIR "${CMAKE_SOURCE_DIR}/test-results/${TARGET_BOARD}")
file(MAKE_DIRECTORY "${TEST_RESULTS_DIR}")

add_custom_target(renode-test
    COMMAND "${CMAKE_SOURCE_DIR}/result/bin/renode-test"
        --tests "${CMAKE_CURRENT_SOURCE_DIR}/tests.yaml"
        -r "${TEST_RESULTS_DIR}"
        --variable "FIRMWARE_ELF:${PROJECT_BINARY_DIR}/flight-software.elf"
        --variable "PLATFORM_YAML:${CMAKE_SOURCE_DIR}/targets/${TARGET_BOARD}/platform.yaml"
        --variable "PROJECT_ROOT:${CMAKE_SOURCE_DIR}"
        --variable "INIT_RESC_FILE:${CMAKE_SOURCE_DIR}/targets/${TARGET_BOARD}/init.resc"
    DEPENDS ${CMAKE_PROJECT_NAME} renode-package
    USES_TERMINAL
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)
