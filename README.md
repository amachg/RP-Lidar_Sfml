# Slamtec RP-LIDAR scan viewer, using SFML, on Windows / Linux

This code reads a 2D point-cloud scan-data from a usb connected RPLIDAR Device and displays the output on an SFML display window, in realtime.
Also it displays the LIDAR itself, its settings, closest/farthest distance arrows, low/hight distance limits, and the cartesian axes.

The user can move the axes origin with the arrow keys and zoom with the mouse wheel.

Developed in C++ without STL (except once) in order to make it easy to migrate to Arduino.

Author: Tassos, Created in October 2023

Requires: 
* Windows	or Linux
* Slamtec RPLIDAR SDK https://github.com/Slamtec/rplidar_sdk
* CMake build system generator https://github.com/Kitware/CMake
* SFML graphic framework (auto installed by cmake) https://github.com/SFML/SFML

Tested with: 
* RPLIDAR model: A1M8 (Firmware ver: 1.29)
* Slamtec RPLIDAR SDK v2, built on Windows 11
* Visual-Studio-2023 and CMake v3.26
* SFML v2.6

RPLIDAR A1M8 performs a 360° scan, from 15 cm to 12 m,

Sample Rate: 2, 4, 8 KiloSamples per second, (default in Firmware v1.29 is 8)

Scan Rate 7.5Hz (=450 RPM) (=1053 samples per round)
