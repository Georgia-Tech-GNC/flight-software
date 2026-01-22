cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME flight_software)

#[[###########################################################################################

DEFINE CONFIGURATION OPTIONS

###########################################################################################]]
# Target STM Chip
set(TARGET_BOARD "F429ZI" CACHE STRING "Target STM chip architecture (see targets directory for implementations)")
set_property(CACHE TARGET_BOARD PROPERTY STRINGS "F429ZI" "H723ZG")

# Target Rocket
set(TARGET_ROCKET "Jet Vanes" CACHE STRING "Target rocket script (see rockets directory for implementations)")
set_property(CACHE TARGET_ROCKET PROPERTY STRINGS "Jet Vanes")


#[[###########################################################################################

Link cmake modules for targeted board architecture

###########################################################################################]]
# For each target board, include toolchain file, add STM32CubeMX generated sources, and set openocd configuration
if (TARGET_BOARD STREQUAL "F429ZI")
    message("Selected target board: F429zi")
    include("Targets/Nucleo-F429ZI/cmake/gcc-arm-none-eabi.cmake")
    add_subdirectory(Targets/Nucleo-F429ZI/cmake/stm32cubemx)
    set(OPENOCD_CFG_FILE "./Targets/Nucleo-F429ZI/openocd.cfg")
endif()

if (TARGET_BOARD STREQUAL "H723ZG")
    message("Selected target board: H723zg")
    include("Targets/Nucleo-H723ZG/cmake/gcc-arm-none-eabi.cmake")
    add_subdirectory(Targets/Nucleo-H723ZG/cmake/stm32cubemx)
    set(OPENOCD_CFG_FILE "./Targets/Nucleo-H723ZG/openocd.cfg")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
# Enable CMake support for ASM and C languages
project(${CMAKE_PROJECT_NAME} C ASM)
enable_language(C ASM)
# Core project settings
message("Build type: " ${CMAKE_BUILD_TYPE})
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
cmake_path(ABSOLUTE_PATH OPENOCD_CFG_FILE NORMALIZE)


#[[###########################################################################################

Link cmake modules for targeted rocket

###########################################################################################]]

if (TARGET_ROCKET STREQUAL "Jet Vanes")
    # Add Jet Vanes-specific sources
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Rockets/JetVanes/Src/commands.c     
        Rockets/JetVanes/Src/jet_vanes.c     
        Rockets/JetVanes/Src/lib.c     
        Rockets/JetVanes/Src/rocket.c 
    )

    # Add Jet Vanes-specific includes
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        Rockets/JetVanes/Inc 
    )

    # Add Jet Vanes-specific project symbols (macros)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        RocketStateStruct=JetVanesRocketState
    )
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    HALAL/Src/halal.c     
    HALAL/Src/hal_timebase.c
    HALAL/Src/Rcc/rcc_nucleo_f429zi.c
    HALAL/Src/Timebase/timebase_tim.c
    HALAL/Src/Adc/adc_internal.c     
    HALAL/Src/Adc/adc_jet_vanes.c     
    HALAL/Src/Debug/debug_uart.c     
    HALAL/Src/Radio/radio_uart.c     
    HALAL/Src/StateEstimation/state_estimation_uart.c     
    HALAL/Src/Magnetometer/lis3mdl.c
    HALAL/Src/FlashChip/fake_flash_chip.c     
    HALAL/Src/Storage/storage_sd_spi.c    
    HALAL/Src/Isr/uart_isr.c     
    HALAL/Src/Isr/adc_isr.c
    HALAL/Src/Isr/tim_isr.c  
    RocketLib/USBHost/Class/MSC/Src/usbh_msc_bot.c     
    RocketLib/USBHost/Class/MSC/Src/usbh_msc_scsi.c     
    RocketLib/USBHost/Class/MSC/Src/usbh_msc.c     
    RocketLib/USBHost/Core/Src/usbh_core.c     
    RocketLib/USBHost/Core/Src/usbh_conf.c     
    RocketLib/USBHost/Core/Src/usbh_ctlreq.c     
    RocketLib/USBHost/Core/Src/usbh_ioreq.c     
    RocketLib/USBHost/Core/Src/usbh_pipes.c     
    RocketLib/FatFs/src/diskio.c     
    RocketLib/FatFs/src/ff.c     
    RocketLib/FatFs/src/ff_gen_drv.c     
    RocketLib/FatFs/src/option/syscall.c     
    RocketLib/FatFs/src/option/ccsbcs.c     
    RocketLib/Controls/Src/controls.c     
    RocketLib/Telemetry/Src/crc_hash.c     
    RocketLib/Telemetry/Src/packet_encode.c     
    RocketLib/Telemetry/Src/protocol.c     
    RocketLib/Telemetry/Src/telemetry.c     
    RocketLib/Utils/Src/log.c     
    RocketLib/Utils/Src/util.c     
    RocketLib/Utils/Src/fs_wrapper.c     
    FreeRTOS-Kernel/Src/croutine.c     
    FreeRTOS-Kernel/Src/event_groups.c     
    FreeRTOS-Kernel/Src/list.c     
    FreeRTOS-Kernel/Src/queue.c     
    FreeRTOS-Kernel/Src/stream_buffer.c     
    FreeRTOS-Kernel/Src/tasks.c     
    FreeRTOS-Kernel/Src/timers.c     
    Tasks/Src/init_tasks.c     
    Tasks/Src/state_flash.c     
    Tasks/Src/state_tx.c     
    Tasks/Src/telemetry_rx.c         
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    HALAL/Inc 
    HALAL/Src/Adc/ 
    HALAL/Src/Isr/ 
    Tasks/Inc 
    FreeRTOS-Kernel/Inc 
    RocketLib/USBHost/Class/MSC/Inc 
    RocketLib/USBHost/Core/Inc 
    RocketLib/Controls/Inc 
    RocketLib/Utils/Inc 
    RocketLib/FatFs/src 
    RocketLib/Telemetry/Inc 
    Rockets/Inc 
    Targets/Inc 
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_target(flash
    DEPENDS ./flight_software.elf
    COMMENT "Starting OpenOCD Server..."
    COMMAND ${CMAKE_COMMAND} -E echo ${OPENOCD_CFG_FILE_GLOBAL}
    COMMAND sudo openocd -s /usr/share/openocd/scripts -f ${OPENOCD_CFG_FILE} -c "program ./flight_software.elf verify reset exit" VERBATIM
)