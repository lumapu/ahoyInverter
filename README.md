# Quick 'n Dirty (Hoymiles) Inverter simulator

## Intended use
This firmware was written to test [AhoyDTU](https://github.com/lumapu/ahoy). It has only basic implementation isn't compareable to a real inverter hardware. Due too much dark hours in winter time (Europe) it can be used to test functionality during night time.

## Overview
For now only a 4 channel inverter will simulated.
The following commands are supported:
* `00` hardware version
* `01` firmware version
* `05` system config
* `0b` live data

## How to use / compile
1. edit the DTU ID in `main.cpp`
2. open this project in VSCode with the extention PlatformIO
3. flash and enjoy (or better contribute to this project 😊)

## Contribution
Any contribution is welcome.