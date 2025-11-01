#pragma once

#include "network_data.h"
#include "raylib.h"

#include <vector>
#include <optional>

namespace ui
{
    struct camera_feed_panel
    {
        Image pixels = { 0 };
        Texture2D texture = { 0 };
        int camera_id;
        int x;
        int y;
        int w;
        int h;
        int frame_count;

        std::optional<robo::network::camera_frame> frame = std::nullopt;

        void draw();
    };

    class camera_feed_visualizer
    {
    private:
        std::vector<camera_feed_panel> panels;
        size_t feed_count;

    public:
        camera_feed_visualizer(size_t feed_count);
        void create_feed(int id, int x, int y, int w, int h);
        void load(robo::network::camera_frame& frame);
        void draw();
        void clear();
    };
}
