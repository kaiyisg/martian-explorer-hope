# Martian Explorer, HOPE

![alt tag](https://cloud.githubusercontent.com/assets/10717593/16744483/aac4faf0-47b1-11e6-8a1f-e6297567fd15.JPG)

## Introduction
HOPE is a project based on the LPC17Xpresso baseboard and ARM-styled architecture LPC1769 microcontrollers, and models the logic behind an autonomous rover on mars, with capabilities of monitoring and broadcasting information on its surroundings, and self preservation during dangerous situations. 

## Features

HOPE’s key functions involve sending data captured from it’s accelerometer, light sensor, and temperature sensor, to a transmission relay, HOME, computer terminal connected to it through UART. Several of HOPE's key configurations can also be externally controlled by a GUI displayed on the OLED display, controlled by the joystick.

##Screenshot - Main Menu
![alt tag](https://cloud.githubusercontent.com/assets/10717593/16744551/f2ea9f92-47b1-11e6-9d31-2c8f6840385b.png)

HOPE has a controllable GUI for the user to control it's settings, and to get real time key information from HOPE.

## How to Install

You need the following hardware:
- LPCXpresso Base Board
- LPC1769 LPCXpresso Development Board
- Mini B to A USB Cable x 2
- Digi XBee RF-Module x 3

You need the following software:
- A laptop installed with LPCXpresso IDE for debugging

You need the following library projects, which must exist in the same workspace in order to build the project successfully:
- CMSISv1p30_LPC17xx : for CMSIS 1.30 files relevant to LPC17xx
- MCU_Lib        	 : for LPC17xx peripheral driver files
- EaBaseBoard_Lib    : for Embedded Artists LPCXpresso Base Board peripheral drivers


If you want to see the source code, first clone this repository, then import it into your LPCXpresso IDE to debug.

## Contributors
- Kai Yi ([@kaiyisg](https://github.com/kaiyisg))
- Daryl ([@darylyong](https://github.com/darylyong))
