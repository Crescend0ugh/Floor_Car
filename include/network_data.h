//
// Created by avsom on 10/12/2025.
//

#pragma once

#include "vector.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>

namespace robo
{
    namespace network
    {
        enum protocol
        {
            placeholder = 0,
            camera_feed,
            rc,
            point_cloud,
        };

        // Server to client
        struct camera_frame
        {
            uint8_t camera_id;
            int frame_height;
            int frame_width;
            std::vector<uchar> bgr_pixels; // [B, G, R, B, G, R, ...]
            uint16_t processing_time;
        };

        // Client to server
        enum rc_command
        {
            none = 0,
            stop,
            w,
            s,
            a,
            d,
            pick_up
        };

        struct point_cloud_mesh_update
        {
            std::vector<vector3f> points;
            std::vector<uint32_t> triangles;
            bool is_delta_points; // If true, append the points received onto the end of the client's point cloud
        };
    }
}

struct ClientMessage
{

};

struct ServerMessage
{
//    std::vector<vector3> points;
//    std::vector<uint32>  indices;
//    vector3 position;
//    vector3 velocity;
//    vector3 acceleration;
//    vector3 rotation;
//    Image   image;
//    vector3 lidar_rotation;
};