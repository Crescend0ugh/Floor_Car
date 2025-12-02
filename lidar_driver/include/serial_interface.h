//
// Created by Adithya Somashekhar on 11/15/25.
//

#pragma once

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <utility>
#include <iomanip>
#include <iostream>

using namespace boost;
namespace robo::serial
{
    enum class parity_option
    {
        none = 0,
        odd = 1,
        even = 2
    };

    enum class stop_bits_option
    {
        one,
        three_halves,
        two
    };


    struct serial
    {
        explicit serial(const std::string &path);

        void open(uint32_t baud, size_t character_size = 8,
                  parity_option parity = parity_option::none,
                  stop_bits_option stop_bits = stop_bits_option::one);

        void close()
        {
            serial_port.close();
        };

        void poll()
        {
            io_context.poll();
        }

        void poll_one()
        {
            io_context.poll_one();
        }

        void set_handler(const std::function<void(const uint8_t *, size_t )>& callback)
        {
            read_callback = callback;
        }

        void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred)
        {


            read_callback(read_buffer, bytes_transferred);
            serial_port.async_read_some(asio::buffer(read_buffer),  std::bind(&serial::handle_read,
                                                                              this,
                                                                              boost::asio::placeholders::error,
                                                                              boost::asio::placeholders::bytes_transferred));
        }



        std::function<void(const uint8_t*, size_t)> read_callback;
        asio::io_context io_context;
        asio::serial_port serial_port;
        std::string path;
        uint8_t read_buffer[8092]{};
    };


}