## CubeMX configuration instructions

### System Core

1. Set SYS->Timebase Source to TIMx (this is so that FreeRTOS can have systick to itself)

### FatFS

1. Check Middlewares->FATFS->User-defined
2. Enable USE_LFN option to "Enabled with static working buffer on the BSS"
3. Set MAX_SS to 4096
4. Configure an unused GPIO pin to GPIO_Output and give it the user label "SD_CS"
5. Set default output level for SD_CS to High

### USART

1. Enable three USARTx/UARTx modules as asynchronous (one as communication with the H723, one as communication with the XBee, and one as debug output)
2. Set baud rate on the XBee UART modeule to 38400
3. Set baud rate on the other modules to 115200
4. Enable DMA for TX/RX on two modules (one as communication with H723 and one as communication with XBee)
5. Set DMA mode for RX to circular on the two modules
6. Enable USARTx global interrupt on all three modules with preemption priority 5

### SPI

1. Enable one SPIx module as Full-Duplex Master
2. Configure baud rate prescaler such that baud rate is somewhere near 5 MBits/s
3. Set Data Size to 8 Bits

### Project Manager

1. Label project name as either "Nucleo-XXXXX" or "MCU-XXXXX" depending on whether the configuration was built off of an MCU or Nucleo board
2. Set project location to /path/to/jet-vanes-flight-software/MainMCU/
3. Set Toolchain/IDE to Makefile
4. Generate code!

## Post-Generation

## Configuration

1. Write port_config.h. You may use the config file in the Nucleo-F429ZI folder as a template

## Fix Autogen

1. In the project root directory, run "python tools/fix_autogen.py" and select the MCU/Nucleo you just generated
2. _This will have to be run every time you regenerate from CubeMX_

## FreeRTOS

1. Depending on the exact type of Cortex-M, add in the appropriate freertos portmacro.h and port.c files (copy from H723 if Cortex-M7, and from F429 if Cortex-M4).

## FATFS

1. Replace code in FATFS/Target/user_dskio.c with what is in the Nucleo-F429ZI folder.

## main.c file

1. Include "port_layer.h" file in user code includes
2. Call port_init() and port_start() in user code section 2

## Makefile

1. Update makefile by adding the following lines to the C_SOURCES list:

Core/Src/port.c \\\
../Core/FreeRTOS-Kernel/croutine.c \\\
../Core/FreeRTOS-Kernel/event_groups.c \\\
../Core/FreeRTOS-Kernel/list.c \\\
../Core/FreeRTOS-Kernel/queue.c \\\
../Core/FreeRTOS-Kernel/stream_buffer.c \\\
../Core/FreeRTOS-Kernel/tasks.c \\\
../Core/FreeRTOS-Kernel/timers.c \\\
../Core/Src/crc_hash.c \\\
../Core/Src/fatfs_sd.c \\\
../Core/Src/packet_encode.c \\\
../Core/Src/port_layer.c \\\
../Core/Src/protocol.c \\\
../Core/Src/periph_io.c \\\
../Core/Src/state_rx.c \\\
../Core/Src/telemetry.c \\\
../Core/Src/transmission_manager.c \\\
../Core/Tests/Src/blink.c \\\
../Core/Tests/Src/sd_test.c \\

2. Next, add the following lines to the C_INCLUDES list:

-I../Core/Include \\\
-I../Core/Tests/Include \\\
-I../Core/FreeRTOS-Kernel/include \\
