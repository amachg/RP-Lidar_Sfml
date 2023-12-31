# Slamtec RP-LIDAR scan viewer, using SFML, in Windows / Linux

This code reads a 2D point-cloud scan-data from a USB connected RPLIDAR Device and displays the output on an SFML display window, in realtime.
Also it displays the LIDAR itself, its settings, closest/farthest obstacle arrows, low/hight distance limits, and the cartesian axes.
The user can move the axes origin with the keyboard arrow keys and zoom with the mouse wheel.
Intended use is for obstacle detection of a diy rover, in RasbianOS on a Raspberry Pi-3B+.
On Raspberry connection is either via USB, or directly via GPIO (both devices use 3.3V level serial port / UART).

Developed in C/C++, without STL (except once in scanModes) to facilitate its migration to Arduino.

Author: Tassos, Created in October 2023

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

RPLIDAR A1M8 performs a 360° scan, from 15 cm to 1200 cm.

Sample Rate: 2, 4, 8 KiloSamples per second, (default in Firmware v1.29 is 8)

Scan Rate 7.5Hz (=450 RPM) (=1053 samples per round)
