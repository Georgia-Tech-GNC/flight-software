
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
