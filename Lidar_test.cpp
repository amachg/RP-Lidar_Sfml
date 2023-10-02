﻿/*
 * RPLIDAR A1M8 can perform a 360° scan, within a 0.15-12 meter range.
 * Firmware Version: 1.29 ( default scan mode being changed to boost mode )
 * Sample Frequency: 2 default, 4 Express, 8 boost, kHz
 * Round Scan Rate: 5.5 by default, or 2-10 Hz
 * Angular Resolution ≤ 1°
 * Sample Duration 0.125 milliseconds
 * UART @ 115200 bps
 * pre-heat RPLIDAR A1 (Start the scan mode and the scan motor is rotating) for
 * more than 2 minutesto get the best measurement accuracy.
 */

#include "../../rplidar_sdk/sdk/include/sl_lidar.h"
//#include <signal.h>

using namespace sl;
//using namespace std;

#include "Lidar_test.h"

int main() {
    ///  Create a LIDAR driver and a communication channel instance.
    auto* lidar_driver = *createLidarDriver();

    //auto com_channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
    auto com_channel = createSerialPortChannel("\\\\.\\com4", 115200);

    /// Make connection to the lidar via the channel.
    auto op_result = lidar_driver->connect(*com_channel);
    if (SL_IS_OK(op_result)) {
        sl_lidar_response_device_info_t deviceInfo;
        op_result = lidar_driver->getDeviceInfo(deviceInfo);
        if (SL_IS_OK(op_result)) {
            /// print out the device firmware and hardware version number.
            printf("\nModel: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                deviceInfo.model,
                deviceInfo.firmware_version >> 8,
                deviceInfo.firmware_version & 0xffu,
                deviceInfo.hardware_version);
        }
    }
    /// Retrieve the health status
    sl_lidar_response_device_health_t healthinfo;
    op_result = lidar_driver->getHealth(healthinfo);
    if (SL_IS_OK(op_result) && healthinfo.status == SL_LIDAR_STATUS_OK)
        printf("Lidar health status is OK.\n");


    /// Start scan
    lidar_driver->setMotorSpeed();
    LidarScanMode scanMode;
    lidar_driver->startScan(false, true, 0, &scanMode);
    //lidar_driver->startScan(/*force=*/false, /*useTypicalScanMode=*/true);

    constexpr const size_t array_size{ 8192 };
    sl_lidar_response_measurement_node_hq_t nodes[array_size];

    /// take only one 360 deg scan and display the result as a histogram
    size_t nodes_count{ array_size };
    op_result = lidar_driver->grabScanDataHq(nodes, nodes_count);
    if (SL_IS_FAIL(op_result)) {
        fprintf(stderr, "Failed to get scan data from LIDAR \n");
        return false;
    }

    /// Rank the scan data according to its angle value.
    lidar_driver->ascendScanData(nodes, nodes_count);

    /// Print numbers
    print_histogram(nodes, nodes_count);

    /// Plot in characters
    plot_histogram(nodes, nodes_count);

    /// Stop scan
    lidar_driver->stop();
    lidar_driver->setMotorSpeed(0);

    delete lidar_driver;
    return 0;
}