cmake_minimum_required(VERSION 3.22)
project(state_estimation_tests LANGUAGES C)

enable_testing()

set(DSP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/CMSIS-DSP")
add_executable(test_state_estimation
    test_state_estimation.c
    ../src/state_estimation.c
    ../src/altitude_kalman_filter.c
    ${DSP_DIR}/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.c
    ${DSP_DIR}/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.c
    ${DSP_DIR}/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c
    ${DSP_DIR}/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.c
    ${DSP_DIR}/Source/MatrixFunctions/arm_mat_init_f32.c
    ${DSP_DIR}/Source/MatrixFunctions/arm_mat_vec_mult_f32.c
)
target_compile_features(test_state_estimation PRIVATE c_std_11)
# Use the CMSIS-DSP host build mode, without the MCU-specific CMSIS Core.
target_compile_definitions(test_state_estimation PRIVATE __GNUC_PYTHON__)
target_include_directories(test_state_estimation PRIVATE ../include)
target_include_directories(test_state_estimation SYSTEM PRIVATE "${DSP_DIR}/Include" "${DSP_DIR}/PrivateInclude")
if(NOT MSVC)
    target_link_libraries(test_state_estimation PRIVATE m)
endif()
add_test(NAME state_estimation_packets COMMAND test_state_estimation)
