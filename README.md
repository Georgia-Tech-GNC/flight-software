# jet-vanes-flight-software

This is the full repository for the flight software for jet-vanes rocket

## Prerequisites

The following software tools are required for contributing to the jet vanes simulations repository:
- [Visual Studio Code](https://code.visualstudio.com/) -Use to program code
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) -Generating Code Configurations for STM32 controllers
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) - Flashing code
- [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) -Required if not using Linux


## Terminal Commands
Updating Ubuntu

```
sudo apt update && sudo apt upgrade -y
```

Installating specific packages

```
sudo apt install make gcc gcc-arm-none-eabi
```

## Usage

1. Open STM32CubeMX and create a new project by doing File/New Project
2. Type in the part number for the STM32 controller you are using. On the bottom right, hover over it and double clicking on the name of the STM32  controller. This will change your screen and you should have an image of the STM32 Controller. 
3. Set the configurations you need for your schematic
4. Go into Project Manager, and under Toolchain/IDE, change it to MakeFile
5. This is VERY important, or else you cannot upload code onto the STM32 controller
6. Save your project with CTRL + S. Put the project in the folder of your choice. The project will output a .ioc(integrated online configurator) which you can open again with STM32CubeMX to change project settings.
7. Click on the Generate Code button. You will be asked to Log into your ST account. Login and then wait. It will have a screen with the folder where the code is in
8. Open the folder in Visual Studio Code. There should be all the programming files needed
9. Program the project with whatever code you need
10. Open a terminal in VSCode by clicking Terminal/New Terminal
11. In the terminal, cd to where the MakeFile is(named MakeFile)
12. Type in make -j4(or make clean if you are starting over). This would create a build folder with a .bin file
13. Open STM32CubeProgrammer. Select the method of connection(in GNC’s case ST-Link) and then click connect.
14. Open the .bin file with Open File. 
15. Wait for it to upload, and you are now done with programming an STM32 microcontroller. 
