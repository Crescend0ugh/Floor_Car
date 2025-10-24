//
// Created by avsom on 10/12/2025.
//

#pragma once

#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>

enum protocol
{
    placeholder = 0,
    camera_feed,
};

struct camera_frame
{
    uint8_t camera_id;
    int frame_height;
    int frame_width;
    std::vector<uchar> bgr_pixels; // [B, G, R, B, G, R, ...]
    uint16_t processing_time;
};

struct message
{
    std::string str;
};

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