# Slamtec RP-LIDAR scan viewer, using SFML, in Windows / Linux

This code reads a 2D point-cloud scan-data from a usb connected RPLIDAR Device and displays the output on an SFML display window, in realtime.
Also it displays the LIDAR itself, its settings, closest/farthest obstacle arrows, low/hight distance limits, and the cartesian axes.
The user can move the axes origin with the keyboard arrow keys and zoom with the mouse wheel.
Intended use is in Rasbian on a Raspberry Pi-3B+ for obstacle detection of a diy rover.

Developed in C/C++, without STL (except once in scanModes) to facilitate its migration to Arduino.

Author: Tassos, Created in October 2023

Requires: 
* Windows, or Linux (although CMakeLists.txt may need adaptation)
* CMake build scripts generator, https://github.com/Kitware/CMake
* Slamtec RPLIDAR SDK, https://github.com/Slamtec/rplidar_sdk
* SFML graphic framework (auto fetched by cmake), https://github.com/SFML/SFML

Tested on:
* Windows 11 (64bit)
* Visual Studio 2022 v17.7
* CMake v3.26
* Slamtec RPLIDAR SDK v2 (targeting WIN32)
* SFML v2.6
* RPLIDAR model A1M8 (firmware version: 1.29), via accompanied usb adapter

RPLIDAR A1M8 performs a 360° scan, from 15 cm to 1200 cm.

Sample Rate: 2, 4, 8 KiloSamples per second, (default in Firmware v1.29 is 8)

Scan Rate 7.5Hz (=450 RPM) (=1053 samples per round)
