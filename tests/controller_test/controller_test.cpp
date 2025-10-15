#include "raylib.h"
#include "controller.h"
#include "vector.h"

Vector2 rotate_vector2(Vector2 point, Vector2 origin, float angle)
{
    float s = sin(angle * PI / 180.0);
    float c = cos(angle * PI / 180.0);

    float translated_x = point.x - origin.x;
    float translated_y = point.y - origin.y;

    float new_x = translated_x * c - translated_y * s;
    float new_y = translated_x * s + translated_y * c;

    return Vector2{ new_x + origin.x, new_y + origin.y };
}

void draw_car_triangle(maid::vector3f position, float angle)
{
    Vector2 center = { position.x, position.z };
    Vector2 v1 = rotate_vector2(Vector2 { center.x, center.y - 50 }, center, angle);
    Vector2 v2 = rotate_vector2(Vector2{ center.x - 50, center.y + 50 }, center, angle);
    Vector2 v3 = rotate_vector2(Vector2{ center.x + 50, center.y + 50 }, center, angle);

    DrawTriangle(v1, v2, v3, BLUE);

    // Represent the heading
    DrawCircle(v1.x, v1.y, 10, RED);
}

int main()
{
	const int screen_width = 1600;
	const int screen_height = 900;

    controller controller;

	InitWindow(screen_width, screen_height, "controller demo");

	SetTargetFPS(144);

    Camera2D camera = { 0 };
    camera.target = Vector2 { 0, 0 };
    camera.offset = Vector2{ GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while (!WindowShouldClose())
    {
        if (IsKeyDown(KEY_W))
        {
            controller.move_forward_distance(0.5);
        }
        if (IsKeyDown(KEY_A))
        {
            controller.rotate_by(-0.5);
        }
        if (IsKeyDown(KEY_S))
        {
            controller.clear_command_queue();
        }
        if (IsKeyDown(KEY_D))
        {
            controller.rotate_by(0.5);
        }

        controller.update();

        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode2D(camera);

        draw_car_triangle(controller.position, controller.heading);

        EndMode2D();

        DrawText(TextFormat("Position: %.2f, %.2f", controller.position.x, controller.position.z), 10, 40, 20, DARKGRAY);
        DrawText(TextFormat("Velocity: %.2f, %.2f", controller.velocity.x, controller.velocity.z), 10, 40 + 20 + 5, 20, DARKGRAY);
        DrawText(TextFormat("Angle (deg): %.2f", controller.heading), 10, 40 + 2 * (20 + 5), 20, DARKGRAY);
        DrawText(TextFormat("Command Queue Size: %d", controller.get_command_queue_size()), 10, 40 + 3 * (20 + 5), 20, DARKGRAY);

        DrawFPS(10, 10);

        EndDrawing();
    }

    CloseWindow();

	return 0;
}