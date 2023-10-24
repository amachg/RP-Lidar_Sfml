# RP-Lidar_Sfml
Slamtec RPLIDAR A1M8 scan display using SFML graphics
* 
* This code reads point clouds data from a RPLIDAR Device
* and displays the output on an SFML display window in realtime.
* User can move origin with arrow keys and zoom with mouse wheel.
* 
* Author: Tassos - https://github.com/amachg...........
* Date: 10 October 2023
* 
* Requires: 
*      Slamtec RPLIDAR SDK:  https://github.com/Slamtec/rplidar_sdk
*      CMake build system (tested in VisualStudio2023)
*      SFML graphic framework (installed by cmake) 
* 
* RPLIDAR A1M8 performs a 360° scan, in a 0.15-12m disk,
* Sample Rate (KSa/s) : 2, 4, 8 (=default in Firmware ver: 1.29)
* Scan Rate 7.5Hz (450 RPM or 1053 samples per round), 
* can be adjusted 2-10 Hz, by Motor PWM Signal
