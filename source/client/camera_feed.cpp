#include "client/camera_feed.h"

void camera_feed_panel::draw()
{
    const int header_x_padding = 6;
    const int header_y_padding = 8;
    const int header_font_size = 18;

    const int no_feed_font_size = 36;

    DrawRectangle(x, y, w, h, BLACK);

    const char* header_text = TextFormat("Camera %d | Frame %d", camera_id, frame_count);
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
        const char* text = TextFormat("NO FEED FROM CAMERA %d", camera_id);
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

camera_feed_visualizer::camera_feed_visualizer(size_t feed_count):
    feed_count(feed_count)
{
    panels.resize(feed_count);
}

void camera_feed_visualizer::create_feed(int id, int x, int y, int w, int h)
{
    assert(id >= 0 && id < feed_count);

    camera_feed_panel& panel = panels[id];
    panel.camera_id = id;
    panel.x = x;
    panel.y = y;
    panel.w = w;
    panel.h = h;
}

void camera_feed_visualizer::load(camera_frame& frame)
{
    int camera_id = frame.camera_id;
    camera_feed_panel& feed = panels[camera_id];
    feed.frame = frame;
    feed.frame_count += 1;

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

    feed.pixels.data = pixel_data.data();
    feed.pixels.width = frame.frame_width;
    feed.pixels.height = frame.frame_height;
    feed.pixels.mipmaps = 1;
    feed.pixels.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;

    if (feed.texture.id > 1)
    {
        UnloadTexture(feed.texture);
    }

    feed.texture = LoadTextureFromImage(feed.pixels);
}

void camera_feed_visualizer::draw()
{
    for (camera_feed_panel& panel : panels)
    {
        panel.draw();
    }
}

void camera_feed_visualizer::clear()
{
    for (camera_feed_panel& panel : panels)
    {
        panel.frame = std::nullopt;
        panel.frame_count = 0;
    }
}