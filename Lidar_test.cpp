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

#include "Lidar_test.h"

int main() {
    setup_GUI();

    // Setup Lidar driver, serial data channel and check health status.
    sl::ILidarDriver* lidar_driver{};
    if (!setup_Lidar(lidar_driver)) {
        delete lidar_driver;
        return false;
    }

    constexpr const size_t array_size{ 8192 }; // actually ~1090
    sl_lidar_response_measurement_node_hq_t nodes[array_size];
    size_t nodes_count{ array_size };

    sf::Event συμβάν;
    while (παράθυρο.isOpen()) {
        while (παράθυρο.pollEvent(συμβάν)) {
            if (συμβάν.type == sf::Event::Closed) παράθυρο.close();
            //else if (συμβάν.type == sf::Event::Resized) {
            //    sf::FloatRect visible( 0,0, συμβάν.size.width, συμβάν.size.height);
            //    παράθυρο.setView(sf::View(visible));
            //}
            else if (συμβάν.type == sf::Event::MouseWheelMoved) { // Zomm in or out
                camera_view.zoom( 1 + συμβάν.mouseWheel.delta * 0.1f);
            }
            else if (συμβάν.type == sf::Event::KeyPressed) {
                switch (συμβάν.key.code) {
                case sf::Keyboard::Left: camera_view.move(-10, 0); break;
                case sf::Keyboard::Right:camera_view.move(10, 0); break;
                case sf::Keyboard::Up:   camera_view.move(0, -10); break;
                case sf::Keyboard::Down: camera_view.move(0, 10);
                }
            }
        }
        /// Wait and grab a complete 0-360 degree scan data asyncly received with startScan.
        auto op_result = lidar_driver->grabScanDataHq(nodes, nodes_count);
        if (SL_IS_FAIL(op_result)) {
            fprintf(stderr, "Failed to get scan data with error code: %x\n", op_result);
            return false;
        }
        /// Rank the scan data according to its angle value.
        lidar_driver->ascendScanData(nodes, nodes_count);

        //Redraw on screen
        παράθυρο.clear(χρώμα_φόντου);
        παράθυρο.setView(camera_view);
        σχεδίασε(παράθυρο, nodes, nodes_count);
        παράθυρο.display(); // Show everything
    }

    /// Stop scan
    lidar_driver->stop();
    sleep(.020);
    lidar_driver->setMotorSpeed(0);

    delete lidar_driver;
    return EXIT_SUCCESS;
}

int main_NO_GUI() {
    // Setup Lidar driver, serial data channel and check health status.
    sl::ILidarDriver* lidar_driver{};
    if ( !setup_Lidar(lidar_driver) ) {
        delete lidar_driver;
        return false;
    }

    constexpr const size_t array_size{ 8192 }; // actually ~1090
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

        //print_data(nodes, nodes_count);
        plot_histogram(nodes, nodes_count);

        printf("Press ENTER to refresh..");
    } while (getchar());

    /// Stop scan
    lidar_driver->stop();
    sleep(.020);
    lidar_driver->setMotorSpeed(0);

    delete lidar_driver;
    return EXIT_SUCCESS;
}