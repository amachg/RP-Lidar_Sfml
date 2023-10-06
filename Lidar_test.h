// Lidar_test.h : project specific include file
#pragma once

constexpr float pi = 3.141592654f;
#include <sl_lidar.h>
#include <SFML/Graphics.hpp>

// Δημιουργία κύριου παραθύρου.
const sf::Vector2u wind_size(900, 900);
sf::RenderWindow παράθυρο({ wind_size.x, wind_size.y }, "RP-LIDAR");
sf::View camera_view;

sf::CircleShape κουκίδα;

void setup_GUI() {

    camera_view.setCenter(0, 0);
    //camera_view.setRotation(90);// Normally lidar motor is on the left of the Window
    camera_view.zoom(1); // >0 means zoom-out
    παράθυρο.setView(camera_view);
    παράθυρο.setPosition({ 0,0 });
    παράθυρο.setFramerateLimit(5);

    κουκίδα.setRadius(.1);
    κουκίδα.setFillColor(sf::Color::Yellow);
}

void σχεδίασε(sf::RenderTarget& render_window, auto nodes, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const auto node = nodes[i];
        const int quality = node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if (quality > 0) {
            const auto start_node = node.flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT ? "Start " : "      ";
            const float theta_deg = node.angle_z_q14 * 90.f / 16384;
            const float theta_rad = theta_deg * pi / 180;
            const int distance_mm = node.dist_mm_q2 / 4;
            const sf::Vector2f endpoint_cm(distance_mm * cos(theta_rad) /10,
                                           distance_mm * sin(theta_rad) /10);
            κουκίδα.setPosition(endpoint_cm);
            render_window.draw(κουκίδα);

            sf::Vertex ray[] = {
                sf::Vertex(sf::Vector2f(0, 0), sf::Color::Red),
                sf::Vertex(endpoint_cm, sf::Color::Red)
            };
            render_window.draw(ray, 2, sf::Lines);
        }
    }

    static const float cross_size = wind_size.y / 5u;
    const sf::Vertex cross_line_hor[] = {
        sf::Vertex({-cross_size, 0}),
        sf::Vertex({ cross_size, 0})
    };
    render_window.draw(cross_line_hor, 2, sf::Lines);
    static const sf::Vertex cross_line_ver[] = {
        sf::Vertex({0,-cross_size}),
        sf::Vertex({0, cross_size})
    };
    render_window.draw(cross_line_ver, 2, sf::Lines);

}

#ifdef __unix__
# include <unistd.h>
#elif defined _WIN32
# include <windows.h>
#define sleep(x) Sleep(1000 * (x))
#endif

bool setup_Lidar(sl::ILidarDriver* & lidar_driver) {
    ///  Create a LIDAR driver
    lidar_driver = *sl::createLidarDriver();
    if (!lidar_driver) {
        fprintf(stderr, "Error, insufficent memory. Exiting..\n");
        return false;
    }

    ///  Create a LIDAR communication channel
    auto com_device = "com4";    // "/dev/ttyUSB0"  (Linux or Win32)
    auto com_channel = sl::createSerialPortChannel(com_device, 115200);

    /// Make connection to the lidar via the serial channel.
    auto op_result = lidar_driver->connect(*com_channel);
    if (SL_IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot bind to the specified serial port. Exiting..\n");
        return false;
    }

    // Retrieve the device info
    sl_lidar_response_device_info_t deviceInfo;
    op_result = lidar_driver->getDeviceInfo(deviceInfo);
    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
            // you can check the detailed failure reason
            fprintf(stderr, "Error, could not get device info, operation time out.\n");
        } else {
            fprintf(stderr, "Error, could not get device info, unexpected error, code: %x\n", op_result);
        }
        return false;
    }

    // print out the device serial number.
    //printf("SLAMTEC LIDAR S/N: ");
    //for (int pos = 0; pos < 16; ++pos) {
    //    printf("%02X", deviceInfo.serialnum[pos]);
    //}
    /// print out the device firmware and hardware version number.
    //printf("\nModel: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
    //    deviceInfo.model,
    //    deviceInfo.firmware_version >> 8,
    //    deviceInfo.firmware_version & 0xffu,
    //    deviceInfo.hardware_version);

    /// Retrieve the health status
    sl_lidar_response_device_health_t healthinfo;
    op_result = lidar_driver->getHealth(healthinfo);
    if (SL_IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
    switch (healthinfo.status) {
        case SL_LIDAR_STATUS_OK:
            printf("Lidar health status is OK.");
            break;
        case SL_LIDAR_STATUS_WARNING:
            printf("Lidar Warning (errorcode: %d)\n", healthinfo.error_code);
            break;
        case SL_LIDAR_STATUS_ERROR:
            printf("Error. Lidar internal error detected.");
            printf(" (errorcode: %d) Rebooting Lidar to retry..\n", healthinfo.error_code);
            lidar_driver->reset();
    }

    /// Start scan useTypicalScanMode
    lidar_driver->setMotorSpeed();//setMotorSpeed(pwm);
    sl::LidarScanMode scanMode;
    if (SL_IS_FAIL(lidar_driver->startScan( false, true, 0, &scanMode))) {
        fprintf(stderr, "Error, cannot start the scan operation.\n");
        delete lidar_driver;
        return false;
    }
    printf(" waiting for data...\n");
    sleep(2);

    return true;
}

void print_data(sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const auto node = nodes[i];
        const float theta_deg = node.angle_z_q14 * 90.f / 16384;
        const int distance_mm = node.dist_mm_q2 / 4;
        const float radians = theta_deg * 3.141 / 180;
        const float x = distance_mm * cos(radians);
        const float y = distance_mm * sin(radians);
        const int quality = node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        const auto start_node = node.flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT ? "Start " : "      ";
        if (quality > 0) {
            // Heading CW angle from positive axis with direction opposite to motor
            printf("%s Angle: %6.2f°", start_node, theta_deg);
            printf("\t\tDistance: %4d mm \n", distance_mm);
        }
    }
}

void plot_histogram(sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    const size_t bars_number = 230;//best 360
    const size_t bars_height = 50;

    size_t histogram[bars_number];
    size_t max_distance{ 0 };

    for (size_t i = 0; i < count; ++i) {
        size_t degree = nodes[i].angle_z_q14 * 90 / 16384;
        if (degree >= bars_number)
            break;

        size_t dist_mm = nodes[i].dist_mm_q2 / 4.0f;
        if (dist_mm > max_distance)
            max_distance = dist_mm;

        histogram[degree] = dist_mm;
    }

    for (int h = bars_height; h > 0; --h) {
        const size_t threshold_h = h * max_distance / bars_height;
        for (size_t screen_x = 0; screen_x < bars_number; ++screen_x)
            putc( (histogram[screen_x] >= threshold_h) ? '*' : ' ', stdout);
        printf("\n");
    }

    for (size_t screen_x = 0; screen_x < bars_number; ++screen_x) {
        if (screen_x % 10 == 0)
            printf("\b\b%3d", screen_x);
        else
            printf(" ");
    }
    printf("\n");
}