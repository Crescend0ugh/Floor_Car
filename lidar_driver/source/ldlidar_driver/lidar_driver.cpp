//
// Created by Adithya Somashekhar on 11/16/25.
//

#include "ldlidar_driver/lidar_driver.h"

namespace robo::ld19
{
    lidar_data_interface::lidar_data_interface(const std::string &path) :
            serial_interface(path),
            data_ready(false),
            packet_index(0),
            state(HEADER),
            current_data_packet({})
    {
        points.resize(2000);
        serial_interface.set_handler([&](const uint8_t* bytes, size_t len)
                    {
                        this->parse_bytes(bytes, len);
                    });
    }

    void lidar_data_interface::open()
    {
        serial_interface.open(BAUD_RATE);
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
                double step = diff / (POINTS_PER_PACKET - 1) / 100.0;
                double start = (double)packet.start_angle / 100.0;

                for (int j = 0; j < POINTS_PER_PACKET; ++j) {
                    double angle = start + step * j;
                    if (angle >= 360.0) {
                        angle -= 360.0;
                    }
                    uint16_t distance = current_data_packet.points[j].distance;
                    int16_t x = distance * cos(angle*3.14159 /180);
                    int16_t y = distance * sin(angle*3.14159 /180);
                    std::cout << x << " " << y << "\n";

                    points.push_front({x, y});
                    points.pop_back();
                }
            }

        }
    }
}