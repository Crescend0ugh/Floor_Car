//
// Created by avsom on 10/12/2025.
//
#if defined(_WIN32)
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif
#include "network.h"
#include "network_data.h"
#include <raylib.h>
#include <sstream>

int main()
{
    InitWindow(0,0,"Client Window");
    SetWindowSize(GetScreenWidth()/2, GetScreenHeight()/2);
    SetWindowPosition(GetScreenWidth()/2, GetScreenHeight()/2);

    std::string ip = "127.0.0.1";

    asio::io_context io_context;
    network::client client(io_context, ip, 12345);
    std::thread thread([&io_context] {
        io_context.run();
    });

    while(!WindowShouldClose())
    {
        std::ostringstream oss;
        if(IsKeyDown(KEY_W))
        {
            oss << "W";
        }
        if(IsKeyDown(KEY_A))
        {
            oss << "A";
        }
        if(IsKeyDown(KEY_S))
        {
            oss << "S";
        }
        if(IsKeyDown(KEY_D))
        {
            oss << "D";
        }

        message m = {oss.str()};
        client.send(0, m);

        BeginDrawing();
        ClearBackground(WHITE);
        EndDrawing();
    }
    CloseWindow();
}