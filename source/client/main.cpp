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

// If the Pi is connected to "nyu" WiFi, the IP is something that starts with "10.20"
// Run "hostname -I" in a Pi terminal and use the IP address it gives you
std::string ip = "127.0.0.1"; // "127.0.0.1"
short port = 12345;

int main()
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(0, 0, "Client Window");
    SetWindowSize(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetWindowPosition(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetTargetFPS(60);

    asio::io_context io_context;
    network::client client(io_context, ip, port);
    std::thread thread([&io_context] {
        io_context.run();
    });

    network::received_data server_data;

    camera_feed_visualizer camera_feed_visualizer(1);
   
    while (!WindowShouldClose())
    {
        rc_command command = rc_command::none;

        if (IsKeyDown(KEY_X))
        {
            command = rc_command::stop;
        }
        else if (IsKeyDown(KEY_W))
        {
            command = rc_command::w;
        }
        else if (IsKeyDown(KEY_A))
        {
            command = rc_command::a;
        }
        else if (IsKeyDown(KEY_S))
        {
            command = rc_command::s;
        }
        else if (IsKeyDown(KEY_D))
        {
            command = rc_command::d;
        }
        
        if (command != rc_command::none)
        {
            client.send(protocol::rc, command);
        }
        
        while (client.poll(server_data))
        {
            switch (server_data.protocol_id)
            {
            case (protocol::camera_feed):
            {
                camera_frame frame;
                network::deserialize(frame, server_data);

                camera_feed_visualizer.load(frame);
            }
            }
        }

        BeginDrawing();
        ClearBackground(WHITE);

        if (!client.is_connected)
        {
            DrawText(TextFormat("CONNECTING TO %s:%d ...", ip.c_str(), port), 50, 50, 50, RED);

            // This can be negative because resolving the endpoints blocks
            auto time_till_retry = std::chrono::duration_cast<std::chrono::seconds>(client.retry_timer.expiry() - std::chrono::steady_clock::now());

            DrawText(
                TextFormat("Retrying connection in %s", 
                    std::format("{}", std::max(time_till_retry, std::chrono::seconds(0))).c_str()
                ),
                50, 120, 50, GREEN
            );

            camera_feed_visualizer.clear();
        }
        else
        {
            camera_feed_visualizer.create_feed(0, GetScreenWidth() / 2, 0, GetScreenWidth() / 2, GetScreenHeight() / 2);
            camera_feed_visualizer.draw();
        }

        EndDrawing();
    }

    CloseWindow();

    thread.join();
    client.disconnect();
}