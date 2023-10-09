/* Lidar_test.h : project specific include file
*
* RPLIDAR A1M8 can perform a 360° scan, within a 15cm..12m range.
* For accuracy, pre - heat for 2' (Start the scan mode and the motor).
*
*Firmware Version : 1.29 (new default scan mode is : Boost)
* Sample Frequency(kHz) : Normal scan mode = 2k, Express = 4k, Boost = 8k
* Round Scan Rate(Hz) : 5.5 by default, or 2 - 10
* Angular Resolution ≤ 1°
* Sample Duration 0.125 milliseconds
* UART @ 115200 bps
*
*/

#pragma once
#include <sl_lidar.h> //RPLIDAR sdk
#include <SFML/Graphics.hpp>

// GLOBALS 
// App Window and View
const sf::Vector2u wind_size(900, 900);
sf::RenderWindow window({ wind_size.x, wind_size.y }, "RP-LIDAR Scan");
sf::View camera_view;
// Graphics 
sf::CircleShape lidar(20), motor(10);
const float cross_size = 450;
const sf::Vertex cross_lines[] = {
    sf::Vertex({-cross_size, 0}),
    sf::Vertex({ cross_size, 0}),
    sf::Vertex({0,-cross_size}),
    sf::Vertex({0, cross_size})
};
const sf::Vertex origin(sf::Vector2f(0, 0), sf::Color::Red);
// Text
sf::Font font;
sf::Text text;
char text_str[20];

void setup_GUI() {
    window.setPosition({ 0,0 }); // Placement of app window on screen
    window.setFramerateLimit(5);
    camera_view.setCenter(0, 0);
    //camera_view.setRotation(90);// Normally lidar motor is on the left of the Window
    camera_view.zoom(1); // >0 means zoom-out
    window.setView(camera_view);

    lidar.setFillColor(sf::Color::Black);
    lidar.setOutlineThickness(1);
    lidar.setOrigin(lidar.getRadius(), lidar.getRadius());//relative to top-left corner of object
    motor.setFillColor(lidar.getFillColor());
    motor.setOutlineThickness(1);
    motor.setOrigin(lidar.getRadius()+ motor.getRadius(), motor.getRadius());

    if (!font.loadFromFile(R"(../../../arial.ttf)"))
        exit(EXIT_FAILURE);
    text.setFont(font);
    text.setPosition(-cross_size, -cross_size);
}

void draw_all(sf::RenderTarget& window, auto nodes, size_t count) {
    static int max_distance_mm{ 0 };

    for (size_t i = 0; i < count; ++i) {
        const auto node = nodes[i];
        const int quality = node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if (quality > 0) {
            const auto start_node = node.flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT ? "Start " : "      ";
            const float theta_deg = node.angle_z_q14 * 90.f / 16384;
            static constexpr auto pi{ 3.141592654f };
            const float theta_rad = theta_deg * pi / 180;
            const int distance_mm = node.dist_mm_q2 / 4;
            static constexpr auto max = [](const int& a, const int& b) {
                return a > b ? a : b; 
            };
            max_distance_mm = max(distance_mm, max_distance_mm);

            const sf::Vector2f endpoint_cm(
                distance_mm * cos(theta_rad) /10,
                distance_mm * sin(theta_rad) /10);
            const sf::Vertex ray[] = { origin, sf::Vertex(endpoint_cm) };
            window.draw(ray, 2, sf::Lines);
        }
    }
    sprintf(text_str, "Max(m): %d\n", max_distance_mm / 10'000);
    text.setString(text_str);
    window.draw(text);
    window.draw(cross_lines, 2, sf::Lines);
    window.draw(&cross_lines[2], 2, sf::Lines);
    window.draw(motor);
    window.draw(lidar);
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

    ///  Create a LIDAR communication channel in Linux or Win32
    auto com_device = "com5";    // "/dev/ttyUSB0" for Linux
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
            op_result = lidar_driver->reset();
            if (SL_IS_FAIL(op_result)) {
                fprintf(stderr, "Error, cannot reset rplidar: %x\n", op_result);
                return false;
            }
    }

    /// Use lidar's typical scan mode, to fetch scan data continuously by background thread
    lidar_driver->setMotorSpeed();
    sl::LidarScanMode outUsedScanMode;
    if (SL_IS_FAIL(lidar_driver->startScan( false, true, 0, &outUsedScanMode))) {
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