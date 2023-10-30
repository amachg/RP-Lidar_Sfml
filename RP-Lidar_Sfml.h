/*
* RP-Lidar_Sfml.h
*/

#pragma once
#include <sl_lidar.h>        // RPLIDAR sdk dependancy
#include <SFML/Graphics.hpp> // SFML dependancy
 
// App Window and View
static const sf::Vector2u wind_size(985, 985);
static sf::RenderWindow window({ wind_size.x, wind_size.y }, "RP-LIDAR Scan");
static sf::View camera_view;
// Graphics 
static sf::CircleShape lidar(5), motor(2), low_range(15), high_range(1200);
static const float cross_size = 1.22 * wind_size.x;
static const sf::Vertex cross[] = {
    sf::Vertex({-cross_size, 0}, sf::Color::Red),
    sf::Vertex({ cross_size, 0}, sf::Color::Red),
    sf::Vertex({ 0, -cross_size}, sf::Color::Red),
    sf::Vertex({ 0, cross_size}, sf::Color::Red)
};
// Text
static sf::Font font;
static sf::Text text;
static auto text_pos = camera_view.getSize().x / 2;

void setup_GUI() {
    window.setPosition({ 0, 0 }); // Placement of app window on screen
    window.setFramerateLimit(5);

    camera_view.setCenter(0, 0);
    //camera_view.setRotation(90);// Normally lidar motor is on the left of the Window
    camera_view.zoom(1); // >1 means zoom-out
    window.setView(camera_view);

    lidar.setFillColor(sf::Color::Black);
    // covert origin from top-left corner of object to its center
    lidar.setOrigin(lidar.getRadius(), lidar.getRadius());
    motor.setFillColor(lidar.getFillColor());
    motor.setOrigin(lidar.getRadius() + motor.getRadius(), motor.getRadius());

    low_range.setFillColor(sf::Color::Transparent);
    low_range.setOutlineThickness(1);
    low_range.setOutlineColor(sf::Color::Yellow);
    low_range.setOrigin(low_range.getRadius(), low_range.getRadius());
    high_range.setFillColor(sf::Color::Transparent);
    high_range.setOutlineThickness(1);
    high_range.setOutlineColor(sf::Color::Yellow);
    high_range.setOrigin(high_range.getRadius(), high_range.getRadius());

    if (!font.loadFromFile(R"(../../../arial.ttf)"))
        exit(EXIT_FAILURE);
    text.setFont(font);
}

static sl::LidarScanMode actual_ScanMode;
static float freq;
static sf::Vector2f prev_reflect;
static constexpr float pi{ 3.14159265359 };

void draw_arrow(const float length, const float angle, sf::Vector2f end_pt) {
    sf::RectangleShape rectangle{ {length-2, 1} };
    rectangle.setFillColor(sf::Color::Red);
    rectangle.setRotation(angle);
    rectangle.setOrigin(0, 0.5);
    window.draw(rectangle);     

    sf::CircleShape triangle(/*radius*/ 4, /*pointCount*/ 3);
    triangle.setFillColor(sf::Color::Red);
    triangle.setPosition(end_pt);
    triangle.setRotation(angle -30);
    const auto r = triangle.getRadius();
    triangle.setOrigin( 2*r, r+2);
    window.draw(triangle);
}

void draw_Scan(sf::RenderTarget& window, sl::ILidarDriver*& lidar_driver,
    auto& nodes, size_t count) {
    /// Draw cross
    window.draw( cross,    2, sf::Lines);
    window.draw(&cross[2], 2, sf::Lines);
    // Draw Lidar device
    window.draw(motor);
    window.draw(lidar);
    /// Draw theoretical rangle limits
    window.draw(low_range);
    window.draw(high_range);

    auto min_dist_cm{ static_cast<unsigned>(high_range.getRadius()) };
    auto max_dist_cm{ static_cast<unsigned>(low_range.getRadius()) };
    float min_dist_theta{}, max_dist_theta{};
    sf::Vector2f min_reflect_pt, max_reflect_pt;

    for (size_t i = 0; i < count; ++i) {
        const auto& node = nodes[i];
        const int brightness = node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if (brightness > 0) {
            const auto start_node = node.flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT ? "Start " : "      ";
            const float theta_deg = node.angle_z_q14 * 90.f / 16384;
            const auto distance_mm = node.dist_mm_q2 / 4;
            const auto distance_cm = distance_mm / 10;

            /// Calc cartesian coordinates
            const float theta_rad = theta_deg * pi / 180;
            const sf::Vector2f reflect_pt( cos(theta_rad) * distance_cm,
                                           sin(theta_rad) * distance_cm );
            /// update min/max distances
            if (distance_cm < min_dist_cm) {
                min_dist_cm = distance_cm;
                min_dist_theta = theta_deg;
                min_reflect_pt = reflect_pt;
            }
            if (distance_cm > max_dist_cm) {
                max_dist_cm = distance_cm;
                max_dist_theta = theta_deg;
                max_reflect_pt = reflect_pt;
            }
            /// Draw Ray lines or Perimeter lines
            //const sf::Vertex line[] = { sf::Vector2f{0, 0}, reflect_pt };
            const sf::Vertex line[] = { prev_reflect, reflect_pt };
            window.draw(line, 2, sf::Lines);
            prev_reflect = reflect_pt;
        }
    }
    /// Print statistics. Must have made full scan to give true freq.
    lidar_driver->getFrequency(actual_ScanMode, nodes, count, freq);
    static auto rpm = static_cast<unsigned>(60 * freq);
    static auto sample_rate = 1000 / actual_ScanMode.us_per_sample;
    static auto samples_per_round = sample_rate / freq;
    static char text_chars[50];
    sprintf(text_chars, "Mode: %s, Scan: %2.1f Hz (%u rpm), "
        "Sample: %2.1f Ksps (%3.2f Kspr)\n", 
        actual_ScanMode.scan_mode, freq, rpm, sample_rate, samples_per_round);
    text.setString(text_chars);
    text.setPosition(-text_pos, -text_pos);
    window.draw(text);
    /// Print bounds
    sprintf(text_chars, "Scan bounds (cm): min=%u @ %f deg, max=%u @ %f deg\n",
        min_dist_cm, min_dist_theta, max_dist_cm, max_dist_theta);
    text.setString(text_chars);
    text.setPosition(-text_pos, text_pos - 50);
    window.draw(text);

    /// Draw min / max direction arrows
    draw_arrow(min_dist_cm, min_dist_theta, min_reflect_pt);
    draw_arrow(max_dist_cm, max_dist_theta, max_reflect_pt);
}

/// Non graphical functions

bool setup_Lidar(sl::ILidarDriver* & lidar_driver) {
    ///  Create a LIDAR driver
    lidar_driver = *sl::createLidarDriver();
    if (!lidar_driver) {
        fprintf(stderr, "Error, insufficent memory. Exiting..\n");
        return false;
    }
    ///  Create a LIDAR communication channel in Linux or Win32
    auto com_device = "com4";    // comX for Windows, or "/dev/ttyUSB0" for Linux
    auto com_channel = sl::createSerialPortChannel(com_device, 115200);
    /// Make connection to the lidar via the serial channel.
    auto op_result = lidar_driver->connect(*com_channel);
    if (SL_IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot bind to the specified serial port. Exiting..\n");
        return false;
    }
    return true;
}

bool print_infos(sl::ILidarDriver*& lidar_driver) {
    // Retrieve the device info
    sl_lidar_response_device_info_t deviceInfo;
    auto op_result = lidar_driver->getDeviceInfo(deviceInfo);
    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
            fprintf(stderr, "Error, no device info, operation time out.\n");
        } else {
            fprintf(stderr, "Error, no device info, error code: %x\n", op_result);
        }
        return false;
    }

    // print out the device serial number.
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", deviceInfo.serialnum[pos]);
    }
    /// print out the device firmware and hardware version number.
    printf("\nModel: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
        deviceInfo.model,
        deviceInfo.firmware_version >> 8,
        deviceInfo.firmware_version & 0xffu,
        deviceInfo.hardware_version);

    /// Retrieve the health status
    sl_lidar_response_device_health_t healthinfo;
    op_result = lidar_driver->getHealth(healthinfo);
    if (SL_IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
    switch (healthinfo.status) {
        case SL_LIDAR_STATUS_OK:
            printf("Lidar health status is OK.\n");
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
    return true;
}

bool start_Lidar(sl::ILidarDriver*& lidar_driver) {
    /// Select a scan mode to fetch scan data by the background thread.
    /// Use typical scan mode (For last model A1 this is "Sensitivity"),
    /// or select mode (0->Standard, 1->Express, 2->Boost, 3->Sensitivity 4->Stability).
 
    //std::vector<sl::LidarScanMode> scanModes;
    //lidar_driver->getAllSupportedScanModes(scanModes);

    auto op_result = lidar_driver->
        startScan(false /*not force scan*/, true /*Use typical scan mode*/,
        //startScanExpress(false, scanModes[3].id, // Select scan Mode
        0, &actual_ScanMode);
    if (SL_IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot start the scan operation.\n");
        delete lidar_driver;
        return false;
    }
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

void print_histogram(sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    const size_t bars_number = 120;//best 360
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