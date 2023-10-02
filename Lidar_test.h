// Lidar_test.h : project specific include file

#pragma once

//#include <iostream>

void plot_histogram(sl_lidar_response_measurement_node_hq_t* nodes, size_t count)
{
    const size_t BARCOUNT = 75;
    const size_t MAXBARHEIGHT = 20;
    // const float ANGLESCALE = 360.0f/BARCOUNT;

    int histogram[BARCOUNT]{0};

    float max_val = 0;
    //size_t degree = 0;
    //for (size_t pos = 0; pos < count || degree < BARCOUNT; ++pos) {
    //    degree = nodes[pos].angle_z_q14 * 90.f / 16384.f;
    //    int cached_distance = histogram[degree];
    //    cached_distance = nodes[pos].dist_mm_q2 / 4.0f;

    //    if (cached_distance > max_val)
    //        max_val = cached_distance;
    //    histogram[degree] = cached_distance;
    //}
    for (int pos = 0; pos < (int)count; ++pos) {
        int degree = (int)(nodes[pos].angle_z_q14 * 90.f / 16384.f);
        if (degree >= BARCOUNT) degree = 0;

        float cached_distance = histogram[degree];
        if (cached_distance == 0.0f) {
            cached_distance = nodes[pos].dist_mm_q2 / 4.0f;
        } else {
            cached_distance = (nodes[pos].dist_mm_q2 / 4.0f + cached_distance) / 2.0f;
        }
    }

    for (int height = 0; height < MAXBARHEIGHT; ++height) {

        const size_t threshold_h = (MAXBARHEIGHT - height - 1) * (max_val / MAXBARHEIGHT);
        for (size_t xpos = 0; xpos < BARCOUNT; ++xpos)
            putc( (histogram[xpos] >= threshold_h) ? '*' : ' ', stdout);
         printf("\n");
    }
}