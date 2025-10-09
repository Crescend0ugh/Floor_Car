//
// Created by avsom on 10/4/2025.
//

#include "raylib.h"
#include <iostream>

int main(int argc, char* argv[]) {
    const int screen_width = 800;
    const int screen_height = 450;

    InitWindow(screen_width, screen_height, "RAYLIB TEST");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("hello world!", 200, 80, 20, RED);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}