# Slamtec RPLIDAR scan graphics viewer, using SFML, on Windows	or	Linux

This code creates a LIDAR 2D scan data display, that reads point-cloud data from an RPLIDAR Device and displays the output, on an SFML display window, in realtime.
Also displayed are closest/farthest distance arrows, low/hight distance limits, the axes and the LIDAR device.

The user can move the axes origin with the arrow keys and zoom with the mouse wheel.

Developed in C++. I have avoided using STL (except once) in order to make it easy to migrate to Arduino.

Author: Tassos, Created in October 2023

Requires: 
* Windows	or	Linux
* Slamtec RPLIDAR SDK: https://github.com/Slamtec/rplidar_sdk
* CMake build system 
* SFML graphic framework (auto installed by cmake)

Tested with: 
* RPLIDAR model: A1M8 (Firmware ver: 1.29)
* Slamtec RPLIDAR SDK v2, built on Windows 11
* Visual-Studio-2023 and CMake v3.26
* SFML v2.6

RPLIDAR A1M8 performs a 360° scan, from 15 cm to 12 m,

Sample Rate (in KiloSamples per second): 2, 4, 8 (default in Firmware ver: 1.29)

Scan Rate 7.5Hz (450 RPM or 1053 samples per round), can possibly be adjusted 2-10 Hz, by motor PWM signal
