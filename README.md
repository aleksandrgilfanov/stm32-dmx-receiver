# stm32-dmx-receiver

There are a lot of different DMX512 projects on github, but all of them do not
check timings of DMX512 packet. The STM32 solution does this, but, sadly STM32
does not open source code for their DMX 512 demonstration.

This project aims to recreate STM32's design of DMX512 receiver.

Overview of STM32 solution can be found in following documents:
UM1004 DMX 512 based LED lighting solution
UM0791 STM_DMX-Receiver FW

This firmware runs on cheap chinese STM32F103C8T6 board:
* Black Pill (HW-621)

Source code initially was generated in CubeMX and does not contain following
CubeMX parts:
* Drivers
* Middlewares 

## Build
1. Copy Drivers and Middlewares directories into project
2. make
3. make flash
