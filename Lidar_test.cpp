/*
 * RPLIDAR A1M8 can perform a 360° scan, within a 0.15-12 meter range.
 * Firmware Version: 1.29 ( default scan mode being changed to boost mode )
 * Sample Frequency: 2 default, 4 Express, 8 boost, kHz
 * Round Scan Rate: 5.5 by default, or 2-10 Hz
 * Angular Resolution ≤ 1°
 * Sample Duration 0.125 milliseconds
 * UART @ 115200 bps
 * For accuracy, pre-heat for 2' (Start the scan mode and the scan motor is rotating).
 */

#include "../../rplidar_sdk/sdk/include/sl_lidar.h"
using namespace sl;
#include "Lidar_test.h"

#ifdef __unix__
# include <unistd.h>
#elif defined _WIN32
# include <windows.h>
#define sleep(x) Sleep(1000 * (x))
#endif

int main() {
    // Setup Lidar driver, serial data channel and check health status.
    ILidarDriver* lidar_driver{};
    if ( !setup(lidar_driver) ) {
        delete lidar_driver;
        return false;
    }

    /// Start scan
    lidar_driver->setMotorSpeed();//setMotorSpeed(pwm);
    LidarScanMode scanMode;
    //lidar_driver->startScan(/*force=*/false, /*useTypicalScanMode=*/true);
    if (SL_IS_FAIL(lidar_driver->startScan(false, true, 0, &scanMode))) {
        fprintf(stderr, "Error, cannot start the scan operation.\n");
        delete lidar_driver;
        return false;
    }
    printf(" waiting for data...\n");
    sleep(2);

    constexpr const size_t array_size{ 8192 };
    sl_lidar_response_measurement_node_hq_t nodes[array_size];

    /// Wait and grab a complete 0-360 degree scan data previously received.
    size_t nodes_count{ array_size };
    do {
        auto op_result = lidar_driver->grabScanDataHq(nodes, nodes_count);
        if (SL_IS_FAIL(op_result)) {
            fprintf(stderr, "Failed to get scan data with error code: %x\n", op_result);
            return false;
        }
        /// Rank the scan data according to its angle value.
        lidar_driver->ascendScanData(nodes, nodes_count);

        /// Print numbers
        //print_data(nodes, nodes_count);
        /// Plot in characters histogram
        plot_histogram(nodes, nodes_count);
        printf("Press ENTER to refresh..");
        getchar();
    } while (true);

    /// Stop scan
    lidar_driver->stop();
    sleep(.5);
    lidar_driver->setMotorSpeed(0);

    delete lidar_driver;
}