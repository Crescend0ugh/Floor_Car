//
// Created by Adithya Somashekhar on 11/15/25.
//

#include "serial_interface.h"

namespace robo::serial
{
    serial::serial(const std::string &path) : serial_port(io_context), path(path) {}

    void serial::open(uint32_t baud, size_t character_size, robo::serial::parity_option parity,
                                robo::serial::stop_bits_option stop_bits)
    {
        serial_port.open(path);

        serial_port.set_option(asio::serial_port_base::baud_rate(baud));
        serial_port.set_option(asio::serial_port_base::character_size(8));
        serial_port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serial_port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serial_port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

        serial_port.async_read_some(asio::buffer(read_buffer),  std::bind(&serial::handle_read,
                                                                          this,
                                                                          boost::asio::placeholders::error,
                                                                          boost::asio::placeholders::bytes_transferred));
    }
}

