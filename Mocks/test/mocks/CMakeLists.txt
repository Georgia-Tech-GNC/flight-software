cmake_minimum_required(VERSION 3.13)
project(flight-software C)

add_subdirectory(${CMAKE_SOURCE_DIR}/Testing/external/Unity)
add_subdirectory(${CMAKE_SOURCE_DIR}/Testing/external/CMock)

file(GLOB MOCK_SOURCES ${CMAKE_SOURCE_DIR}/Mocks/test/mocks/*.c)

# Build test executable
add_executable(run_tests
    ${MOCK_SOURCES}
)

target_include_directories(run_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/Mocks
    ${CMAKE_SOURCE_DIR}/Testing/external/Unity/src
)

target_link_libraries(run_tests unity cmock)