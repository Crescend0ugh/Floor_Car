//
// Created by avsom on 10/4/2025.
//
#if defined(_WIN32)
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif

#include "network.h"
#include "network_data.h"
#include "vision.h"

#include <raylib.h>
#include <iostream>
#include <thread>
#include <functional>

// How often we're running object detection
const auto vision_interval = std::chrono::milliseconds(1000);

// Convert cv::Mat to a string and processing_time to u16
camera_frame serialize_detection_results(detection_results& results)
{
    cv::Mat image = results.annotated_image.value();

    std::vector<uchar> pixels;

    if (image.isContinuous()) 
    {
        // If the matrix is continuous (no padding between rows),
        // copy all data at once
        pixels.assign(image.data, image.data + image.total() * image.elemSize());
    }
    else 
    {
        // If the matrix is not continuous, copy row by row
        for (int i = 0; i < image.rows; ++i) 
        {
            pixels.insert(pixels.end(), image.ptr<uchar>(i), image.ptr<uchar>(i) + image.cols * image.elemSize());
        }
    }

    return camera_frame
    {
        .camera_id = results.camera_id,
        .frame_height = image.rows,
        .frame_width = image.cols,
        .bgr_pixels = pixels,
        .processing_time = static_cast<uint16_t>(results.processing_time.value().count()),
    };
}

void run_vision(network::server& server, vision& vision, asio::steady_timer& vision_timer)
{
    // TODO: Also run on the other camera
    detection_results results = vision.detect_from_camera(0);

    // Send vision results to clients, if any are connected
    if (vision.is_client_connected)
    {
        auto serialized = serialize_detection_results(results);
        server.send(protocol::camera_feed, serialized);
    }

    // Schedule next object detection cycle
    vision_timer.expires_at(vision_timer.expiry() + vision_interval);
    vision_timer.async_wait(std::bind(&run_vision, std::ref(server), std::ref(vision), std::ref(vision_timer)));
}

int main(int argc, char* argv[]) {

    asio::io_context io_context;
    network::server server(io_context, 12345);
    std::thread thread([&io_context] {
        io_context.run();
    });

    vision vision;

    // Run vision and object detection every second
    asio::basic_waitable_timer<std::chrono::steady_clock> vision_timer(io_context, vision_interval);
    vision_timer.async_wait(std::bind(
        &run_vision, 
        std::ref(server), 
        std::ref(vision), 
        std::ref(vision_timer)
    ));

    network::received_data data;

    while (1)
    {
        vision.is_client_connected = server.get_client_count() > 0;

        // READ
        while (server.poll(data))
        {
            message m;
            network::deserialize(m, data);
            std::cout << m.str;
        }
    }
}