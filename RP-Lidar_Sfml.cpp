/*
* Slamtec RPLIDAR A1M8 scan display using SFML graphics
* 
* This code reads point clouds data from a RPLIDAR Device
* and displays the output on an SFML display window in realtime.
* User can move origin with arrow keys and zoom with mouse wheel.
* 
* Author: Tassos - https://github.com/amachg/RP-Lidar_Sfml
* Date: 10 October 2023
* 
* Requires: 
*      Slamtec RPLIDAR SDK:  https://github.com/Slamtec/rplidar_sdk
*      CMake build system
*      SFML graphic framework 
* 
* RPLIDAR A1M8 performs a 360° scan, in a 0.15-12m disk,
* Sample Rate (KSa/s): 2, 4, 8 (=default in Firmware ver: 1.29)
* Scan Rate: 7.5Hz (450 RPM or 1053 samples per round), 
* This can be changed from 2 to 10 Hz by directly adjusting 
* motor PWM signal, e.g. on arduino. Impossible through usb.
* 
*/

#include "RP-Lidar_Sfml.h"

int main() {
    setup_GUI();

    // Setup Lidar driver, serial data channel and check health status.
    sl::ILidarDriver* lidar_driver{};
    if(!setup_Lidar(lidar_driver)
    //|| !print_infos(lidar_driver)
    //|| !start_Lidar(lidar_driver)
        ) return false;

    constexpr size_t array_size{ 8192 };
    //sl_lidar_response_measurement_node_hq_t nodes[array_size];
    auto nodes = new sl_lidar_response_measurement_node_hq_t[array_size];
    auto dist_accummul = new sl_lidar_response_measurement_node_hq_t[array_size]{};
    short scans_count{ 0 };
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
            else if (event.type == sf::Event::MouseWheelMoved) {
                camera_view.zoom(1 - event.mouseWheel.delta * 0.1f);
            }
            else if (event.type == sf::Event::KeyPressed) {
                switch (event.key.code) {
                case sf::Keyboard::Left: camera_view.move(-10, 0); break;
                case sf::Keyboard::Right:camera_view.move(10, 0); break;
                case sf::Keyboard::Up:   camera_view.move(0, -10); break;
                case sf::Keyboard::Down: camera_view.move(0, 10);
                }
            }
            window.setView(camera_view); // adjust to updated view
            text_pos = - camera_view.getSize() * .5f;
            text.setCharacterSize(camera_view.getSize().y / wind_size.y * 30); // height in pixels
        }
        /// Wait and grab a complete 0-360 degree scan data, asyncly received with startScan.
        auto op_result = lidar_driver->grabScanDataHq(nodes, nodes_count);
        if (SL_IS_FAIL(op_result)) {
            fprintf(stderr, "Failed to get scan data with error code: %x\n", op_result);
            break;
        }
        /// Rank the scan data according to its angle value.
        lidar_driver->ascendScanData(nodes, nodes_count);

        /// Accummulate
        if (scans_count++ < 5) {
            for (size_t i = 0; i < nodes_count; ++i)
                dist_accummul[i].dist_mm_q2 += nodes[i].dist_mm_q2;
        }
        else {
            /// calc average node distance
            for (size_t i = 0; i < nodes_count; ++i)
                nodes[i].dist_mm_q2 = dist_accummul[i].dist_mm_q2 / 5;
            ///reset array to zero
            memset(dist_accummul, 0, nodes_count * sizeof(*dist_accummul));
            scans_count = 0;
        }

        ///Redraw screen
        window.clear(sf::Color::Blue);
        draw_Scan(window, lidar_driver, nodes, nodes_count);
        window.display();
    }
    /// Stop scan and exit
    lidar_driver->stop();
    delete lidar_driver;    //delete heap based pointed object by this variable
    delete[] nodes;
    delete[] dist_accummul;

    return EXIT_SUCCESS;
}

int main_NO_GUI() {
    /// Setup Lidar driver, serial data channel and check health status.
    sl::ILidarDriver* lidar_driver{};
    if(!setup_Lidar(lidar_driver)
    //|| !print_infos(lidar_driver)
    || !start_Lidar(lidar_driver)
        ) {
        return false;
    }
    constexpr const size_t array_size{ 8192 };
    sl_lidar_response_measurement_node_hq_t nodes[array_size];
    size_t nodes_count{ array_size };

    /// Wait and grab a complete 0-360 degree scan data previously received.
    do {
        auto op_result = lidar_driver->grabScanDataHq(nodes, nodes_count);
        if (SL_IS_FAIL(op_result)) {
            fprintf(stderr, "Failed to get scan data with error code: %x\n", op_result);
            return false;
        }
        /// Rank the scan data according to its angle value.
        lidar_driver->ascendScanData(nodes, nodes_count);

        //print_data(nodes, nodes_count);
        print_histogram(nodes, nodes_count);

        printf("Press ENTER to refresh..");
    } while (getchar());

    /// Stop scan and exit
    lidar_driver->stop();
    delete lidar_driver;
    return EXIT_SUCCESS;
}