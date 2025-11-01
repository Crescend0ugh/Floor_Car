//
// Created by Adithya Somashekhar on 11/1/25.
//

#pragma once

#include <cstdint>

#pragma pack( 1)

namespace robo
{

    enum
    {
        HEADER = 12,
        VER_LEN = 0x2c,
        POINTS_PER_PACK = 12
    };

    struct lidar_point {
        uint16_t distance;
        uint8_t intensity;
    };

    struct lidar_frame {
        uint8_t header;
        uint8_t ver_len;
        uint16_t speed;
        uint16_t start_angle;
        lidar_point points[POINTS_PER_PACK];
        uint16_t end_angle;
        uint8_t crc8;
    };




}
#pragma pack()