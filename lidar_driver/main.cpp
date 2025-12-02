//
// Created by Adithya Somashekhar on 11/22/25.
//

#include "ldlidar_driver/lidar_driver.h"
#include <raylib.h>

int main()
{
    const int screenWidth = 800*2;
    const int screenHeight = 450*2;

    InitWindow(screenWidth, screenHeight, "raylib 2D Draw Pixels");

    robo::ld19::lidar_data_interface li("/dev/tty.usbserial-0001");
    li.open();
    SetTargetFPS(24);
    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(BLACK);  // Clear the background to a specific color
        li.poll();
        for (int i = 0; i < li.points.size(); ++i) {
            const auto& point = li.points[i];
            auto x = screenWidth/2 + point.first/5;
            auto y =  screenHeight/2 + point.second/5;
            DrawPixel(x, y, RED);
            if (i < 12)
            DrawLine(screenWidth/2, screenHeight/2, x, y, WHITE);

        }


        EndDrawing();
    }
    CloseWindow();
}