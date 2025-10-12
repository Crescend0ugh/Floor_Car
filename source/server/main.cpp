//
// Created by avsom on 10/4/2025.
//
#if defined(_WIN32)
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif
#include "network.h"
#include "network_data.h"
#include <raylib.h>
#include <iostream>

int main(int argc, char* argv[]) {

    asio::io_context io_context;
    network::server server(io_context, 12345);
    std::thread thread([&io_context] {
        io_context.run();
    });

    network::received_data data;
    while(1)
    {
        // READ
        while (server.poll(data))
        {
            message m;
            network::deserialize(m, data);
            std::cout << m.str;
        }
    }

}