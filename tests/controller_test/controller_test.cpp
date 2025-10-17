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

const char* get_command_text(command::command* command_ptr)
{
    if (command::delay* command = std::get_if<command::delay>(command_ptr))
    {
        return TextFormat("Delay: %dms", command->milliseconds);
    }
    else if (command::move_for_seconds* command = std::get_if<command::move_for_seconds>(command_ptr))
    {
        return TextFormat("Move for: %ds", command->seconds);
    }
    else if (command::rotate_for_seconds* command = std::get_if<command::rotate_for_seconds>(command_ptr))
    {
        return TextFormat("Rotate for: %ds", command->seconds);
    }
    else if (command::move_distance* command = std::get_if<command::move_distance>(command_ptr))
    {
        return TextFormat("Move distance: %.2f units", command->distance);
    }
    else if (command::rotate_to_heading* command = std::get_if<command::rotate_to_heading>(command_ptr))
    {
        return TextFormat("Rotate to heading: %.2f deg", command->heading);
    }
    else if (command::rotate_by* command = std::get_if<command::rotate_by>(command_ptr))
    {
        return TextFormat("Rotate by: %.2f deg", command->delta_heading);
    }
}

void draw_controller_state(controller controller)
{
    const int font_size = 20;
    const int padding = 5;
    const int x = 10;
    const int y = 40;

    const int max_commands = 10;

    auto get_y = [&](const int order) 
        {
            return y + order * (font_size + padding);
        };

    DrawText(TextFormat("Position: %.2f, %.2f", controller.position.x, controller.position.z), x, get_y(0), font_size, DARKGRAY);
    DrawText(TextFormat("Velocity: %.2f, %.2f", controller.velocity.x, controller.velocity.z), x, get_y(1), font_size, DARKGRAY);
    DrawText(TextFormat("Angle (deg): %.2f", controller.heading), x, get_y(2), font_size, DARKGRAY);
    DrawText(TextFormat("Command Queue Size: %d", controller.get_command_queue().size()), x, get_y(3), font_size, DARKGRAY);

    if (controller.get_current_command().has_value())
    {
        auto command = *controller.get_current_command();
        DrawText(TextFormat("[Current Command] %s", get_command_text(&command)), x, get_y(4), font_size, GREEN);
    }
    else
    {
        DrawText(TextFormat("[Current Command] None"), x, get_y(4), font_size, RED);
    }

    auto command_queue_copy = controller.get_command_queue();
    int queued_command_count = 1;

    while (!command_queue_copy.empty() && queued_command_count <= max_commands)
    {
        auto element = command_queue_copy.front();
        DrawText(TextFormat("%d) %s", queued_command_count, get_command_text(&element)), x, get_y(4 + queued_command_count), font_size, DARKGRAY);
        command_queue_copy.pop();

        ++queued_command_count;
    }
}

int main()
{
	const int screen_width = 1600;
	const int screen_height = 900;

    controller controller;
    controller.is_remote_controlled = true;

	InitWindow(screen_width, screen_height, "controller demo");

	SetTargetFPS(60);

    Camera2D camera = { 0 };
    camera.target = Vector2 { 0, 0 };
    camera.offset = Vector2{ GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while (!WindowShouldClose())
    {
        if (IsKeyDown(KEY_W))
        {
            controller.drive_state = remote_control_drive_state::forward;
        } 
        else
        {
            controller.drive_state = remote_control_drive_state::not_driving;
        }

        if (IsKeyDown(KEY_A))
        {
            controller.steer_state = remote_control_steer_state::left;
        }
        else if (IsKeyDown(KEY_D))
        {
            controller.steer_state = remote_control_steer_state::right;
        }
        else {
            controller.steer_state = remote_control_steer_state::not_steering;
        }

        if (IsKeyDown(KEY_R))
        {
            controller.rotate_by(-5);
        }
        if (IsKeyDown(KEY_F))
        {
            controller.move_forward_distance(1);
        }

        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode2D(camera);

        draw_car_triangle(controller.position, controller.heading);

        EndMode2D();
    
        draw_controller_state(controller);

        DrawFPS(10, 10);

        EndDrawing();

        controller.update();
    }

    CloseWindow();

	return 0;
}