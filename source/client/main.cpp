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

#include <optional>
#include <algorithm>
#include <sstream>

void load_camera_frame(camera_frame& frame, Image& image, Texture2D& texture)
{
    std::vector<uint8_t> pixel_data(frame.frame_width * frame.frame_height * 3);

    // Convert BGR to RGB
    for (size_t i = 0; i < frame.bgr_pixels.size(); i += 3)
    {
        uint8_t blue = frame.bgr_pixels[i];
        uint8_t green = frame.bgr_pixels[i + 1];
        uint8_t red = frame.bgr_pixels[i + 2];

        pixel_data[i] = red;
        pixel_data[i + 1] = green;
        pixel_data[i + 2] = blue;
    }

    image.data = pixel_data.data();
    image.width = frame.frame_width;
    image.height = frame.frame_height;
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;

    if (texture.id > 1)
    {
        UnloadTexture(texture);
    }

    texture = LoadTextureFromImage(image);
}

void draw_camera_feed(uint8_t id, std::optional<camera_frame>& frame, int x, int y, int w, int h, Texture2D& texture)
{
    const int header_x_padding = 6;
    const int header_y_padding = 8;
    const int header_font_size = 18;

    const int no_feed_font_size = 36;

    DrawRectangle(x, y, w, h, BLACK);

    const char* header_text = TextFormat("Camera %d", id);
    int header_width = MeasureText(header_text, header_font_size);
    DrawText(header_text, x + header_x_padding, y + header_y_padding, header_font_size, WHITE);

    if (frame)
    {
        // Display processing time
        DrawText(
            TextFormat(" | Processing Time: %d ms", frame.value().processing_time), 
            x + 2 * header_x_padding + header_width, 
            y + header_y_padding, 
            header_font_size, 
            GREEN
        );

        // Rescale texture to fit in the feed
        const int max_texture_height = h - (2 * (header_y_padding + header_font_size));
        const int max_texture_width = w - (2 * header_x_padding);

        float height_scale = (float)max_texture_height / texture.height;
        float width_scale = (float)max_texture_width / texture.width;

        float scale_factor = std::min(height_scale, width_scale);

        DrawTextureEx(
            texture, 
            Vector2{ 
                (float)x + header_x_padding + (w / 2) - (scale_factor * (float)texture.width / 2),
                (float)y + header_y_padding + (h / 2) - (scale_factor * (float)texture.height / 2)
            }, 
            0.0f, 
            scale_factor,
            WHITE
        );
    }
    else
    {
        const char* text = TextFormat("NO FEED FROM CAMERA %d", id);
        int text_width = MeasureText(text, no_feed_font_size);
        DrawText(
            text,
            x + (w / 2) - (text_width / 2),
            y + (h / 2) - (no_feed_font_size / 2),
            no_feed_font_size,
            RED
        );
    }
}

int main()
{
    // If the Pi is connected to "nyu" WiFi, the IP is: "10.20.2.14"
    // Otherwise, run "hostname -I" in a Pi terminal and use the IP address it gives you
    std::string ip = "127.0.0.1"; 
    asio::io_context io_context;
    network::client client(io_context, ip, 12345);
    std::thread thread([&io_context] {
        io_context.run();
    });

    std::optional<camera_frame> left_camera_frame = std::nullopt;
    std::optional<camera_frame> right_camera_frame = std::nullopt;

    network::received_data server_data;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(0, 0, "Client Window");
    SetWindowSize(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetWindowPosition(GetScreenWidth() / 2, GetScreenHeight() / 2);
    SetTargetFPS(60);

    Image left_frame_pixels = { 0 };
    Texture2D left_frame_texture = { 0 };

    Image right_frame_pixels = { 0 };
    Texture2D right_frame_texture = { 0 };

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

                if (frame.camera_id == 0)
                {
                    left_camera_frame = frame;
                    load_camera_frame(frame, left_frame_pixels, left_frame_texture);
                }
                else 
                {
                    right_camera_frame = frame;
                    load_camera_frame(frame, right_frame_pixels, right_frame_texture);
                }
            }
            }
        }

        BeginDrawing();
        ClearBackground(WHITE);

        draw_camera_feed(0, left_camera_frame, GetScreenWidth() / 2, 0, GetScreenWidth() / 2, GetScreenHeight() / 2, left_frame_texture);
        draw_camera_feed(1, right_camera_frame, GetScreenWidth() / 2, GetScreenHeight() / 2, GetScreenWidth() / 2, GetScreenHeight() / 2, right_frame_texture);

        EndDrawing();
    }

    CloseWindow();
}