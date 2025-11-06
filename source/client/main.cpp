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
#include "client/point_cloud_visualizer.h"

#include <raylib.h>

#include <optional>
#include <algorithm>
#include <sstream>

// If the Pi is connected to "nyu" WiFi, the IP is something that starts with "10.20"
// Run "hostname -I" in a Pi terminal and use the IP address it gives you

const std::string port = "12345";

asio::io_context io_context;
network::client client(io_context, "127.0.0.1", port);

ui::camera_feed_visualizer camera_feed_visualizer(1);
ui::connection_screen connection_screen(client);
ui::point_cloud_visualizer point_cloud_visualizer;

static void handle_server_messages()
{
    network::received_data server_data;
    while (client.poll(server_data))
    {
        switch (server_data.protocol_id)
        {
        case (robo::network::protocol::camera_feed):
        {
            robo::network::camera_frame frame;
            network::deserialize(frame, server_data);

            camera_feed_visualizer.load(frame);
            break;
        }
        case (robo::network::protocol::point_cloud):
        {
            robo::network::point_cloud_mesh_update update;
            network::deserialize(update, server_data);

            //if (update.is_delta_points)
            //{
            //    point_cloud_visualizer.add_points(update.points);
            //}
            //else
            //{
            //    point_cloud_visualizer.set_points(update.points);
            //}

            break;
        }
        }
    }
}

static void send_rc_command()
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
    else if (IsKeyDown(KEY_LEFT))
    {
        command = robo::network::rc_command::servo_ccw;
    }
    else if (IsKeyDown(KEY_RIGHT))
    {
        command = robo::network::rc_command::servo_cw;
    }

    if (command != robo::network::rc_command::none && client.is_connected)
    {
        client.send(robo::network::protocol::rc, command);
    }
}

int main()
{
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(0, 0, "Client Window");
    SetWindowSize(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetWindowPosition(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetTargetFPS(60);

    std::thread thread([&] {
        io_context.run();
    });

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 15.0f, 5.0f };
    camera.target = { 0.0f, 0.0f, -10.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 90.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose())
    {  
        send_rc_command();
        handle_server_messages();

        BeginDrawing();
        ClearBackground(WHITE);

        if (!client.is_connected)
        {
            auto ip_address_to_connect = connection_screen.get_submitted_input();
            if (ip_address_to_connect.has_value())
            {
                client.set_ip_address(ip_address_to_connect.value());
            }

            connection_screen.draw();
            camera_feed_visualizer.clear();
        }
        else
        {
            camera_feed_visualizer.create_feed(0, GetScreenWidth() / 2, 0, GetScreenWidth() / 2, GetScreenHeight() / 2);
            camera_feed_visualizer.draw();
            connection_screen.reset();

            BeginMode3D(camera);
            point_cloud_visualizer.draw();
            EndMode3D();
        }

        EndDrawing();
    }

    CloseWindow();

    io_context.stop();
    thread.join();
    client.disconnect();

    return 0;
}