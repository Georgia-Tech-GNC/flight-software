# GNC Flight Software

This monorepository is the home of all flight software and related on-device programs for all major GNC projects. 
Certain standalone and helper programs have their own repositories, see:
- Ground Station: https://github.com/Georgia-Tech-GNC/ground-station

This repository is intended to serve as a flexible and modular embedded project for designing avionics software targeting [STMicroelectronics microcontrollers](https://www.st.com/en/microcontrollers-microprocessors.html) and [FreeRTOS](https://www.freertos.org/). More specific details can be found in [the wiki](https://github.com/Georgia-Tech-GNC/flight-software/wiki).

**NEW MEMBERS:** Check out the [setup guide](https://github.com/Georgia-Tech-GNC/flight-software/wiki/Setup-Guide) and the [onboarding materials](https://github.com/Georgia-Tech-GNC/flight-software/wiki/Setup-Guide) in the wiki!

## Git and Repository Organization Strategy
### Branch Overview:
- **main** - Base project template and reusable drivers and code segments
- **project/\<project-name\>** - Project-specific branches for ongoing rocket projects
- **other** - Miscellaneous feature/development branches

It is very important that every commit in main and project-specific branches represents a functional step in project history.
This makes it easy for users to go back through project commit logs and see when/how features are implemented. 
To help enforce this rule, project branches (and main) can __**only be updated using squash and merge**__. 
All features, improvements, and bugfixes should be developed on a dedicated branch, and then pulled into project branches via a PR. 
PR's to main and project branches should compile, pass all CI checks, and be tested on target hardware prior to being merged.

### File Structure
- `core/src` - Source files for the active project and the shared `main.c` entrypoint 
- `core/include` - Header files for the active project
- `drivers/<driver-name>` - Source files and headers for reusable device and peripheral drivers
- `lib/<name>` - Reusable submodules and utility functions 
- `targets/<target-name>` - CubeMX-generated code, including HAL and CMSIS drivers, target-specific sources, FreeRTOS source and header files, and the target-specific main.c entrypoints

### Supporting multiple targets
- **Never** include device-specific HAL headers (for example, `stm32h7xx_hal.h`). Include `main.h` instead, which is defined per-target and will recursively include all available HAL headers for the chosen target.
- FreeRTOS headers can be included directly, and all targets are assumed to implement it.
- All targets must call `void shared_main(void)` inside their `main()` functions immediately after initializing peripherals. The prototype for this method must go in `main.h`.
- Since different configurations may or may not compile with different targets, the only compilation requirement is that <u>everything should compile when using the "dev" cmake profile</u> (the `Nucleo-h723zg` target). Project branches should also compile when targeting that rocket's specific flight computer chip.

### Compiling
This repository is intended to be compiled using CMake with the Ninja backend. The main `CMakeLists.txt` can be found in the root directory of the project. 
Each subdirectory (`core/`, `drivers/`, `lib/`, and `targets/`) has its own `CMakeLists.txt`, which is recursively included into the CMake project.
Cache variables can be used to select a specific target and library list. Use `ccmake` to view all relevant options in a terminal user interface.

The recommended way to compile this repository is via the presets available in `CMakePresets.json`. 
The most of important of these is `dev` and `dev-release`, which builds the flight-software and **all** modules targeting the STM32h723zg Nucleo-144 development board.
- To configure the `dev` preset (you must re-run this every time the CMake configuration is modified), run the command `cmake --preset dev`.
- To build the `dev` preset, run the command `cmake --build --preset dev`

After running both, the compiled `flight-software.elf` file should appear in `build/dev`. 
This project also supports Doxygen as an optional dependency. 
If you have it installed, then compiling will automatically generate documentation for the project. 
This documentation can be viewed by opening `build/dev/html/index.html` in a browser window.