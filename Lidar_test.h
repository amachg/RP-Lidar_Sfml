// Lidar_test.h : project specific include file

#pragma once

//#include <iostream>

void plot_histogram(sl_lidar_response_measurement_node_hq_t* nodes, size_t count)
{
    const int BARCOUNT = 75;
    const int MAXBARHEIGHT = 20;
    // const float ANGLESCALE = 360.0f/BARCOUNT;

    float histogram[BARCOUNT];
    for (int pos = 0; pos < BARCOUNT; ++pos) {
        histogram[pos] = 0.0f;
    }

    float max_val = 0;
    for (int pos = 0; pos < (int)count; ++pos) {
        int int_deg = (int)(nodes[pos].angle_z_q14 * 90.f / 16384.f);
        if (int_deg >= BARCOUNT) int_deg = 0;
        float cachedd = histogram[int_deg];
        if (cachedd == 0.0f) {
            cachedd = nodes[pos].dist_mm_q2 / 4.0f;
        } else {
            cachedd = (nodes[pos].dist_mm_q2 / 4.0f + cachedd) / 2.0f;
        }

        if (cachedd > max_val) max_val = cachedd;
        histogram[int_deg] = cachedd;
    }

    for (int height = 0; height < MAXBARHEIGHT; ++height) {
        float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val / MAXBARHEIGHT);
        for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
            if (histogram[xpos] >= threshold_h) {
                putc('*', stdout);
            } else {
                putc(' ', stdout);
            }
        }
        printf("\n");
    }
    for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
        putc('-', stdout);
    }
    printf("\n");
}