
add_definitions("-w")
# When adding code to core/src, make sure to add those files to this list
target_sources(${CMAKE_PROJECT_NAME} PUBLIC src/main.c)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE include/)