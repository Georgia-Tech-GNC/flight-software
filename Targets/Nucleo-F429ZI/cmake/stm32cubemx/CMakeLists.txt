cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F429xx
    TARGET_NUCLEO_F429ZI
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc 
    ../../Modules/Inc 
    ../../Drivers/STM32F4xx_HAL_Driver/Inc 
    ../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy 
    ../../Drivers/CMSIS/Device/ST/STM32F4xx/Include 
    ../../Drivers/CMSIS/DSP/Include 
    ../../Drivers/CMSIS/Include 
    USB_HOST/App 
    USB_HOST/Target
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c 
    ../../Core/Src/port.c 
    ../../Core/Src/syscalls.c 
    ../../Core/Src/sysmem.c 
    ../../Core/Src/system_stm32f4xx.c 
    ../../Modules/Src/gpio.c 
    ../../Modules/Src/rcc.c 
    ../../Modules/Src/tim.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c 
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c 
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 17)
    message(ERROR "Generated code requires C17 or higher")
endif()


