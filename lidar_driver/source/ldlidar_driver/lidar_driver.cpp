//
// Created by Adithya Somashekhar on 11/16/25.
//

#include "lidar_driver.h"
#include "concurrentqueue.h"

namespace robo::ld19
{
    lidar_data_interface::lidar_data_interface(const std::string &path) :
            serial::serial_interface(path),
            data_ready(false),
            packet_index(0),
            state(HEADER),
            current_data_packet({})
    {
        points.resize(200);
        set_handler(std::bind(&lidar_data_interface::parse_bytes, this, std::placeholders::_1, std::placeholders::_2));
    }

    void lidar_data_interface::open()
    {
        serial::serial_interface::open(BAUD_RATE);
    }

    bool lidar_data_interface::parse_byte(uint8_t byte)
    {
        switch (state) {
            case HEADER:
                if (byte == HEADER) {
                    current_data_packet.bytes[packet_index++] = byte;
                    state = VER_LEN;
                }
                break;

            case VER_LEN:
                if (byte == VER_LEN) {
                    current_data_packet.bytes[packet_index++] = byte;
                    state = DATA;
                } else {
                    state = HEADER;
                    packet_index = 0;
                    return false;
                }
                break;

            case DATA:
                current_data_packet.bytes[packet_index++] = byte;
                if (packet_index >= sizeof(data_packet)) {
                    uint8_t crc = calc_crc8(current_data_packet.bytes, sizeof(data_packet) - 1);
                    state = HEADER;
                    packet_index = 0;
                    if (crc == current_data_packet.crc_8) {
                        return true;
                    } else {
                        return false;
                    }
                }
                break;
            default:
                break;

        }
        return false;
    }


    void lidar_data_interface::parse_bytes(const uint8_t *bytes, size_t bytes_transferred)
    {
        for (size_t i = 0; i < bytes_transferred; i++) {
            if (parse_byte(bytes[i])) {
                // parse a package is success
                const auto& packet = current_data_packet;
                uint32_t diff = ((uint32_t)packet.end_angle + 36000 - (uint32_t)packet.start_angle) % 36000;
                float step = diff / (POINTS_PER_PACKET - 1) / 100.0;
                float start = (double)packet.start_angle / 100.0;
                for (int j = 0; j < POINTS_PER_PACKET; ++j) {
                    float angle = start + (i*step);
                    if (angle >= 360.0) {
                        angle -= 360.0;
                    }
                    uint16_t distance = packet.points[j].distance;
                    std::cout << angle << " " << distance << "\n";
                    std::tuple<int16_t, int16_t> coord = {distance * cos(angle), distance * sin(angle)};

                }
            }

        }
    }
}