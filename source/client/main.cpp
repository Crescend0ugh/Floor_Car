//
// Created by avsom on 10/12/2025.
//
#if defined(_WIN32)
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif

#include "network.h"
#include "network_data.h"
#include "client/camera_feed.h"

#include <raylib.h>

#include <optional>
#include <algorithm>
#include <sstream>

int main()
{
    // If the Pi is connected to "nyu" WiFi, the IP is something that starts with "10.20"
    // Run "hostname -I" in a Pi terminal and use the IP address it gives you
    std::string ip = "192.168.1.154"; //"192.168.1.154"; // "127.0.0.1"
    asio::io_context io_context;
    network::client client(io_context, ip, 12345);
    std::thread thread([&io_context] {
        io_context.run();
    });

    network::received_data server_data;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(0, 0, "Client Window");
    SetWindowSize(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetWindowPosition(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetTargetFPS(60);

    camera_feed_visualizer camera_feeds(2);
   
    while (!WindowShouldClose())
    {
        std::ostringstream oss;
        if (IsKeyDown(KEY_W))
        {
            oss << "W";
        }
        if (IsKeyDown(KEY_A))
        {
            oss << "A";
        }
        if (IsKeyDown(KEY_S))
        {
            oss << "S";
        }
        if (IsKeyDown(KEY_D))
        {
            oss << "D";
        }

        message m = {oss.str()};
        //client.send(0, m);

        while (client.poll(server_data))
        {
            switch (server_data.protocol_id)
            {
            case (protocol::camera_feed):
            {
                camera_frame frame;
                network::deserialize(frame, server_data);

                camera_feeds.load(frame);
            }
            }
        }

        BeginDrawing();
        ClearBackground(WHITE);

        camera_feeds.create_feed(0, GetScreenWidth() / 2, 0, GetScreenWidth() / 2, GetScreenHeight() / 2); // Left camera
        camera_feeds.create_feed(1, GetScreenWidth() / 2, GetScreenHeight() / 2, GetScreenWidth() / 2, GetScreenHeight() / 2); // Right camera
        camera_feeds.draw();

        EndDrawing();
    }

    CloseWindow();
}