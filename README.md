# Slamtec RPLIDAR Viewer, using SFML, built by cmake

LIDAR scan-data display, using SFML graphics v2.6, built by cmake 3.26.

This code reads point clouds data from a RPLIDAR Device and displays the output on an SFML display window in realtime.
User can move origin with arrow keys and zoom with mouse wheel.

Author: Tassos, Created on October 2023

Requires: 
* Slamtec RPLIDAR SDK:  https://github.com/Slamtec/rplidar_sdk
* CMake build system (tested on Visual-Studio-2023, Windows 11)
* SFML v2.6 graphic framework (auto installed by cmake) 

RPLIDAR A1M8 performs a 360° scan, in a 0.15-12m disk,
Sample Rate (KSa/s) : 2, 4, 8 (=default in Firmware ver: 1.29)
Scan Rate 7.5Hz (450 RPM or 1053 samples per round), 
can be adjusted 2-10 Hz, by Motor PWM Signal
