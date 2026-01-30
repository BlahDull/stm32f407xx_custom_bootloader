
# stm32f407xx_custom_bootloader

This project implements a custom bootloader for the STM32F407xx family of MCUs. Allows for programming of flash memory, reading flash, erasing flash, jumping to given addresses, and setting up memory protection configurations.

The user can enter the bootloader mode by holding the user button (PA0) down during reset. If the user button is not held then the bootloader application will simply jump to the reset handler of the user application.

The bootloader is interacted with by sending commands and receiving responses via the USART2 peripheral. Several commands are supported.

| Command                   | Code   | Response                          | Description                                                         |
| ------------------------- | ------ | --------------------------------- | ------------------------------------------------------------------- |
| **BL_GET_VER**            | `0x51` | Bootloader version (1 byte)       | Reads the bootloader firmware version from the MCU.                 |
| **BL_GET_HELP**           | `0x52` | Supported command list (10 bytes) | Returns all commands supported by the bootloader.                   |
| **BL_GET_CID**            | `0x53` | Chip ID (2 bytes)                 | Reads the MCU chip identification number.                           |
| **BL_GET_RDP_STATUS**     | `0x54` | RDP level (1 byte)                | Reads the Flash Read Protection (RDP) level.                        |
| **BL_GO_TO_ADDR**         | `0x55` | Status (1 byte)                   | Jumps execution to a specified memory address.                      |
| **BL_FLASH_ERASE**        | `0x56` | Status (1 byte)                   | Performs mass erase or sector erase on user flash.                  |
| **BL_MEM_WRITE**          | `0x57` | Status (1 byte)                   | Writes data into MCU memory (Flash/RAM).                            |
| **BL_EN_R_W_PROTECT**     | `0x58` | Status (1 byte)                   | Enables read/write protection on selected flash sectors.            |
| **BL_MEM_READ**           | `0x59` | Memory data (variable length)     | Reads data from MCU memory. *(TODO: not implemented)*               |
| **BL_READ_SECTOR_STATUS** | `0x5A` | Sector status (2 bytes)           | Reads protection status of all flash sectors.                       |
| **BL_OTP_READ**           | `0x5B` | OTP data                          | Reads One-Time Programmable (OTP) memory. *(TODO: not implemented)* |
| **BL_DIS_R_W_PROTECT**    | `0x5C` | Status (1 byte)                   | Disables read/write protection and restores default state.          |


# IMPORTANT

The bootloader has reserved the first two sectors of the flash memory, so the user application must be placed at sector 2+. It is important to ensure that the user application vector table is placed at 0x08008000 in flash.
To do this I just changed the linker script of my user application to place the program starting at that address. In addition, it is important to note that the user application vector table must be placed with an offset as well.
This can be done in the Vector Table Offset Register (VTOR) in the SCB of the ARM-CORTEX-M4 processor.

# Implementation

I utilized the STM32CubeMX HAL libraries for this project since the focus was on the bootloader operations, not on GPIO control or UART communication. The bootloader is entirely written in C, and primarily uses UART for communication.

# Background
I wanted to learn more about bootloaders after I did my custom startup project, in order to understand what the MCU does from the moment of reset to when it reaches main().

# Known Issues

# ***TODO***
Finish implementing 
* MemRead
* SectorStatus
* OTPRead

Clean up the user application and push it here
