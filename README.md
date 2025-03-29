# Slamtec RP-LIDAR scan viewer, using SFML, in Windows / Linux

This code reads in realtime a 2D point-cloud scan-data from a USB connected RPLIDAR Device and displays the output on an SFML display window.
Also it displays the LIDAR itself, its settings, closest/farthest obstacle arrows, low/hight distance limits, and the cartesian axes.
The user can move the axes origin with the keyboard arrow keys and zoom with the mouse wheel.
Intended use is for obstacle detection of a diy rover I build on a Raspberry Pi-3B in RasbianOS.
On a Raspberry, the LIDAR connection is either via USB, or directly via GPIO (both devices use 3.3V level serial port / UART).

RPLIDAR A1M8 performs a 360° horizontal scan, ranging from 0.15 m up to 1.2 m.

Sample Rate can be 2, or 4, or 8 KiloSamples per second, (default is 8 in Firmware v1.29).

Scan Rate is 7.5 Hz, aka 450 RPM, aka 1053 Samples per round.

Requires: 
* Windows, or Linux (although CMakeLists.txt may need adaptation)
* CMake build scripts generator, https://github.com/Kitware/CMake
* Slamtec RPLIDAR SDK, https://github.com/Slamtec/rplidar_sdk
* SFML graphic framework (auto fetched by cmake), https://github.com/SFML/SFML

Tested on:
* RPLIDAR A1M8-R6 (firmware version: 1.29), connected via the accompanied usb adapter,
* Windows 11 (64bit)
* Visual Studio 2022 v17.7 including CMake v3.26
* Slamtec RPLIDAR SDK v2 (targeting WIN32)
* SFML v2.6

Developed in C/C++, without STL (except once in scanModes) to facilitate its future migration to Arduino.

Author: Tassos, Created in October 2023
