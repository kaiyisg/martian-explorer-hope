# Martian Explorer, HOPE
## Introduction
HOPE is a project based on the LPC17Xpresso baseboard and ARM-styled architecture LPC1769 microcontrollers, and models the logic behind an autonomous rover on mars, with capabilities of monitoring and broadcasting information on its surroundings, and self preservation during dangerous situations. 

[HOPE Screenshot](HOPE_HOMESCREEN.jpeg)

## Features

HOPE’s key functions involve sending data captured from it’s accelerometer, light sensor, and temperature sensor, to a transmission relay, HOME, computer terminal connected to it through UART. Several of HOPE's key configurations can also be externally controlled by a GUI displayed on the OLED display, controlled by the joystick.

## How to Install

You need the following hardware:
-LPCXpresso Base Board
-LPC1769 LPCXpresso Development Board
-Mini B to A USB Cable x 2
-Digi XBee RF-Module x 3

You need the following software:
-A laptop installed with LPCXpresso IDE for debugging

You need the following library projects, which must exist in the same workspace in order to build the project successfully:
- CMSISv1p30_LPC17xx : for CMSIS 1.30 files relevant to LPC17xx
- MCU_Lib        	 : for LPC17xx peripheral driver files
- EaBaseBoard_Lib    : for Embedded Artists LPCXpresso Base Board peripheral drivers


If you want to see the source code, first clone this repository, then import it into your LPCXpresso IDE to debug.

Cheers,

Kai Yi & Daryl