cmake_minimum_required(VERSION 3.22)
cmake_policy(SET CMP0135 NEW)

################################## CMake Top-Level Project Settings ##################################
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(flight-software)
add_executable(${CMAKE_PROJECT_NAME} "")
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

set(WARNING_FLAGS "-Wall" "-Wextra" "-Wpedantic" "-Wshadow")
list(APPEND WARNING_FLAGS "-Wconversion" "-Wcast-align" "-Wcast-qual" "-Wdouble-promotion" "-Wsign-conversion")
list(APPEND WARNING_FLAGS "-Wformat=2" "-Wundef" "-Wstrict-prototypes" "-Wmissing-prototypes" "-Wpointer-arith")

# TODO: apply compiler flags?

enable_language(C ASM)

########################################## Build Submodules ##########################################
add_subdirectory(targets)
add_subdirectory(core)

############################################# Build Docs #############################################
find_package(Doxygen)
if(Doxygen_FOUND)
    message(STATUS "Downloading doxygen theme...")
    include(FetchContent)
    FetchContent_Declare(
        doxygen-theme-css
        URL https://github.com/jothepro/doxygen-awesome-css/archive/refs/heads/main.zip
    )
    FetchContent_MakeAvailable(doxygen-theme-css)
    FetchContent_GetProperties(doxygen-theme-css SOURCE_DIR DOXYGEN_THEME_DIR)

    set(DOXYGEN_PROJECT_NAME "Flight Software")
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_QUIET YES)
    set(DOXYGEN_OPTIMIZE_OUTPUT_FOR_C YES)
    set(DOXYGEN_GENERATE_LATEX NO)
    set(DOXYGEN_HTML_EXTRA_STYLESHEET "${DOXYGEN_THEME_DIR}/doxygen-awesome.css")
 
    doxygen_add_docs(
        generate_docs
        ${PROJECT_SOURCE_DIR}/core 
        ALL
        COMMENT "Generating API documentation with Doxygen"
    )
else()
    message(STATUS "Doxygen not found. Documentation target will not be created.")
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
