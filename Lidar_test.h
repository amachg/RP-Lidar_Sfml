// Lidar_test.h : project specific include file
#pragma once

void print_histogram(sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const auto node = nodes[i];
        const float angle_deg = node.angle_z_q14 * 90.f / 16384;
        const int distance_mm = node.dist_mm_q2 / 4;
        const float radians = angle_deg * 3.141 / 180;
        const float x = distance_mm * cos(radians);
        const float y = distance_mm * sin(radians);
        const int quality = node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        const auto start_node = node.flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT ? "Start " : "      ";
        if (quality > 0) {
            printf("%s Heading: %6.2f°", start_node, angle_deg);
            printf("\t\tDistance: %4d mm \n", distance_mm);
        }
    }
}

void plot_histogram(sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    const size_t bars = 231;//best 360
    const size_t bars_height = 48;

    size_t histogram[bars];
    size_t max_distance{ 0 };

    for (size_t i = 0; i < count; ++i) {

        size_t degree = nodes[i].angle_z_q14 * 90 / 16384;
        if (degree >= bars)
            break;

        size_t dist_mm = nodes[i].dist_mm_q2 / 4.0f;
        if (dist_mm > max_distance)
            max_distance = dist_mm;

        histogram[degree] = dist_mm;
    }

    for (size_t screen_y = 0; screen_y < bars_height; ++screen_y) {
        const size_t threshold_h = (bars_height - screen_y - 1) * (max_distance / bars_height);
        for (size_t screen_x = 0; screen_x < bars; ++screen_x)
            putc( (histogram[screen_x] >= threshold_h) ? '*' : ' ', stdout);
        printf("\n");
    }

    for (size_t screen_x = 0; screen_x < bars; ++screen_x) {
        if (screen_x % 10 == 0)
            printf("\b\b%3d", screen_x);
        else
            printf(" ");
    }
}