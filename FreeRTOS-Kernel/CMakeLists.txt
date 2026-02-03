#[[####################################################################
FreeRTOS-Kernel/CMakeLists.txt

This file adds FreeRTOS to the target cmake project.
It is NOT intended to be called independently, but is rather ran
via an "add_subdirectory" directive from the base CMakeLists.txt file.
Project settings like the C standard and the project name are 
inherited from the base CMakeLists.txt file as well.

You probably shouldn't need to ever edit this file unless you are 
updating the FreeRTOS kernel source code and there is a new file.
#]]####################################################################

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    source/croutine.c     
    source/event_groups.c     
    source/list.c     
    source/queue.c     
    source/stream_buffer.c     
    source/tasks.c     
    source/timers.c   
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE 
    ./include
)
