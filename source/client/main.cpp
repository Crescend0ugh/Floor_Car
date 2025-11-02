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
#include "client/connection_screen.h"

#include <raylib.h>

#include <optional>
#include <algorithm>
#include <sstream>

using namespace std::chrono_literals;

// If the Pi is connected to "nyu" WiFi, the IP is something that starts with "10.20"
// Run "hostname -I" in a Pi terminal and use the IP address it gives you

const std::string port = "12345";

asio::io_context io_context;
network::client client(io_context, "127.0.0.1", port);

ui::camera_feed_visualizer camera_feed_visualizer(1);
ui::connection_screen connection_screen;

int main()
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(0, 0, "Client Window");
    SetWindowSize(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetWindowPosition(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetTargetFPS(60);

    std::thread thread([&] {
        io_context.run();
    });

    network::received_data server_data;

    while (!WindowShouldClose())
    {
        robo::network::rc_command command = robo::network::rc_command::none;

        if (IsKeyDown(KEY_X))
        {
            command = robo::network::rc_command::stop;
        }
        else if (IsKeyDown(KEY_P))
        {
            command = robo::network::rc_command::pick_up;
        }
        else if (IsKeyDown(KEY_W))
        {
            command = robo::network::rc_command::w;
        }
        else if (IsKeyDown(KEY_A))
        {
            command = robo::network::rc_command::a;
        }
        else if (IsKeyDown(KEY_S))
        {
            command = robo::network::rc_command::s;
        }
        else if (IsKeyDown(KEY_D))
        {
            command = robo::network::rc_command::d;
        }
        
        if (command != robo::network::rc_command::none && client.is_connected)
        {
            client.send(robo::network::protocol::rc, command);
        }
        
        while (client.poll(server_data))
        {
            switch (server_data.protocol_id)
            {
            case (robo::network::protocol::camera_feed):
            {
                robo::network::camera_frame frame;
                network::deserialize(frame, server_data);

                camera_feed_visualizer.load(frame);
            }
            }
        }

        BeginDrawing();
        ClearBackground(WHITE);

        if (!client.is_connected)
        {
            auto ip_address_to_connect = connection_screen.get_submitted_input();
            if (ip_address_to_connect.has_value())
            {
                client.set_ip_address(ip_address_to_connect.value());
            }

            DrawText(TextFormat("Connecting to %s ...", client.ip.c_str()), 50, 50, 50, MAROON);

            // This can be negative because resolving the endpoints blocks
            auto time_till_retry = std::chrono::duration_cast<std::chrono::seconds>(client.retry_timer.expiry() - std::chrono::steady_clock::now());

            if (time_till_retry >= 0s)
            {
                DrawText(
                    TextFormat("Retrying connection in %s",
                        std::format("{}", std::max(time_till_retry, std::chrono::seconds(0))).c_str()
                    ),
                    50, 120, 50, GREEN
                );
            }
            else
            {
                DrawText("...", 50, 120, 50, GREEN);
            }
            

            connection_screen.draw();
            camera_feed_visualizer.clear();
        }
        else
        {
            camera_feed_visualizer.create_feed(0, GetScreenWidth() / 2, 0, GetScreenWidth() / 2, GetScreenHeight() / 2);
            camera_feed_visualizer.draw();
            connection_screen.reset();
        }

        EndDrawing();
    }

    CloseWindow();

    io_context.stop();
    thread.join();
    client.disconnect();

    return 0;
}