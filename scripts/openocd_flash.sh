#!/usr/bin/env bash
set -e

case "$1" in
    "ncl-h723")
        echo "Flashing STM32H723..."
        openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/ncl-h723-dev/flight-software.elf verify reset exit"
        ;;
    "fc-h725")
        echo "Flashing STM32H725..."
        openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/fc-h725-dev/flight-software.elf verify reset exit"
        ;;
    "fc-h725")
        echo "Flashing STM32H725..."
        openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/fc-h725-dev/flight-software.elf verify reset exit"
        ;;
    "fc-h725")
        echo "Flashing STM32H725..."
        openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/fc-h725-dev/flight-software.elf verify reset exit"
        ;;
    *)
        exit 1
        ;;
esac

echo "Success"
