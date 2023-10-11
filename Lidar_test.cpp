/*
 * Slamtec LIDAR Display for SFML
 * 
 * This code reads point cloud data from a Slamtec RPLIDAR Device and
 * displays the output on an SFML display in realtime.
 * 
 * Author: Tassos - https://github.com/amachg
 * Date: 10 October 2023
 * 
 * Requires: 
 *      Slamtec RPLIDAR SDK:  https://github.com/Slamtec/rplidar_sdk (needs installation)
 *      CMake build system (tested in VS)
 *      SFML graphic framework (installed automatically) 
 */

#include "Lidar_test.h"

int main() {
    setup_GUI();
    // Setup Lidar driver, serial data channel and check health status.
    sl::ILidarDriver* lidar_driver{};
    if (!setup_Lidar(lidar_driver)) {
        //delete lidar_driver;
        //return false;
    }
    constexpr const size_t array_size{ 8192 }; // actually ~1090 used
    sl_lidar_response_measurement_node_hq_t nodes[array_size];
    size_t nodes_count{ array_size };
    sf::Event event;

    while (window.isOpen()) {
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::Resized) {
                sf::Vector2f new_size(event.size.width, event.size.height);
                camera_view.setSize(new_size);
            }
            else if (event.type == sf::Event::MouseWheelMoved)
                camera_view.zoom( 1 + event.mouseWheel.delta * 0.1f);
            else if (event.type == sf::Event::KeyPressed) {
                switch (event.key.code) {
                case sf::Keyboard::Left: camera_view.move(-10, 0); break;
                case sf::Keyboard::Right:camera_view.move(10, 0); break;
                case sf::Keyboard::Up:   camera_view.move(0, -10); break;
                case sf::Keyboard::Down: camera_view.move(0, 10);
                }
            }
        }
        ///// Wait and grab a complete 0-360 degree scan data asyncly received with startScan.
        //auto op_result = lidar_driver->grabScanDataHq(nodes, nodes_count);
        //if (SL_IS_FAIL(op_result)) {
        //    fprintf(stderr, "Failed to get scan data with error code: %x\n", op_result);
        //    //return false;
        //}
        /// Rank the scan data according to its angle value.
        lidar_driver->ascendScanData(nodes, nodes_count);

        //Redraw on screen
        window.setView(camera_view);
        window.clear(sf::Color::Blue);
        draw_all(window, nodes, nodes_count);
        window.display(); // Show everything
    }

    /// Stop scan
    lidar_driver->stop();
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
    lidar_driver->setMotorSpeed(0);

    delete lidar_driver;
    return EXIT_SUCCESS;
}