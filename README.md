# STM32 Bootloader Project

## Overview

This repository contains a bootloader project for STM32 microcontrollers, enabling firmware updates through a serial communication interface. The project is organized into three main directories: `Host`, `Study_Material`, and `workspace`.

### Directories

1. **Host**

   - Contains user programs or applications designed for communication with the bootloader via the serial port.
   - **Status:** In development phase.

2. **Study_Material**

   - Essential documents and notes for understanding the project.
     - **01_Flash_Organization.txt:** Details flash organization for the 01 project in the workspace directory.
     - **Linker_Script_Notes.txt:** Provides insights into linker script usage.
     - **Secure_Bootloader.pdf:** A guide to understanding secure bootloaders.
   
3. **workspace**

   - Main working directory for application and bootloader projects.

     - **01_App_Blink  01_Blink_Before_Main_Application:**
       - Application and bootloader coexist; the bootloader blinks a red LED and jumps to the application at address 0x08002000.

     - **02_App:**
       - A standalone application.

     - **02_Bootloader:**
       - A command line-based bootloader allowing communication when the user button is pressed within the initial 5 seconds.

   - **Status:** In development phase.

## Contact Information

For inquiries or assistance, please contact: adepsumit123@gmail.com

## Acknowledgement

Developed with support from Fastbit Embedded Brain Academy.

## Contributing

We welcome contributions and feedback. Feel free to submit issues or pull requests.

**Note:** Detailed documentation for this project will be available soon.

Refer to the documentation in the `Study_Material` directory for a better understanding of the project's structure and functionality.

