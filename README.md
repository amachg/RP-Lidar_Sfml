# Slamtec RPLIDAR Viewer, using SFML, for Windows	or	Linux

 2D LIDAR scan-data display, using SFML graphic, built by cmake.

This code reads point clouds data from a RPLIDAR Device and displays the output on an SFML display window in realtime.
User can move origin with arrow keys and zoom with mouse wheel.

Author: Tassos, Created on October 2023

Requires: 
* Windows	or	Linux
* Slamtec RPLIDAR SDK: https://github.com/Slamtec/rplidar_sdk
* CMake build system 
* SFML graphic framework (auto installed by cmake)

Tested with: 
* Slamtec RPLIDAR SDK v2
* CMake v3.26 ( within Visual-Studio-2023 - Windows 11)
* SFML v2.6
* RPLIDAR model: A1M8 (Firmware ver: 1.29)

RPLIDAR A1M8 performs a 360° scan, from 15 cm to 12 m,
Sample Rate (in KiloSamples per second): 2, 4, 8 (default in Firmware ver: 1.29)
Scan Rate 7.5Hz (450 RPM or 1053 samples per round), can possibly be adjusted 2-10 Hz, by motor PWM signal
