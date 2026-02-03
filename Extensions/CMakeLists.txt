#[[####################################################################
Extensions/CMakeLists.txt

This file handles the Extensions portion of the codebase.
It is NOT intended to be called independently, but is rather ran
via an "add_subdirectory" directive from the base CMakeLists.txt file.
Project settings like the C standard and the project name are 
inherited from the base CMakeLists.txt file as well.
Each extension can be optionally toggled on by the user
#]]####################################################################

#[[##########################################################
                            FATFS                            
#]]##########################################################
option(INCLUDE_FATFS_EXTENSION "Include and build the FatFs filesystem library" OFF)
if (INCLUDE_FATFS_EXTENSION) 
    message(STATUS "Including the FatFs Extension")
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE fatfs/src fatfs)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        fatfs/src/diskio.c
        fatfs/src/ff.c
        fatfs/src/ff_gen_drv.c
        fatfs/src/option/syscall.c
        fatfs/src/option/ccsbcs.c
        fatfs/user_diskio.c
    )
endif()