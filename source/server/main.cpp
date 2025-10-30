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
#include "controller.h"
#include "serialib.h"

#include <raylib.h>
#include <iostream>
#include <thread>
#include <functional>

// Delay between each object detection cycle
const auto vision_interval = std::chrono::milliseconds(50);

// Convert cv::Mat to a string and processing_time to u16
camera_frame serialize_detection_results(detection_results& results)
{
    cv::Mat image = results.annotated_image.value();

    std::vector<uchar> pixels;

    if (image.isContinuous()) 
    {
        // If the matrix is continuous (no padding between rows), copy all data at once
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
    if (vision.is_enabled)
    {
        bool succeeded = vision.grab_frame();
        detection_results results;
        
        if (succeeded)
        {
            results = std::move(vision.detect_from_camera());
            // TODO: Estimate 3D positions of detection bounding box centers
            // vision.estimate_3d_positions(...);
        }

        // Send vision results to clients, if any are connected
        if (vision.is_client_connected)
        {
            if (results.annotated_image.has_value())
            {
                auto serialized = serialize_detection_results(results);
                server.send(protocol::camera_feed, serialized);
            }
        }
    }

    // Schedule next object detection cycle
    vision_timer.expires_at(std::chrono::steady_clock::now() + vision_interval);
    vision_timer.async_wait(std::bind(&run_vision, std::ref(server), std::ref(vision), std::ref(vision_timer)));
}

int main(int argc, char* argv[]) {

    asio::io_context network_io_context;
    network::server server(network_io_context, 12345);
    std::thread network_thread([&] {
        network_io_context.run();
    });

    vision vision;

    // Run vision and object detection every second
    asio::io_context vision_io_context;
    asio::basic_waitable_timer<std::chrono::steady_clock> vision_timer(vision_io_context, vision_interval);
    std::thread vision_thread([&] {
        vision_io_context.run();
    });
    
    vision_timer.async_wait(std::bind(
        &run_vision, 
        std::ref(server), 
        std::ref(vision), 
        std::ref(vision_timer)
    ));
    

    controller controller;
    if (!controller.arduino_serial.is_connected())
    {
        std::cerr << "||| Warning |||: Failed to connect to Arduino UNO" << std::endl;
    };

    network::received_data data;
    while (1)
    {
        vision.is_client_connected = server.get_client_count() > 0;

        controller.move_forward_for(0.01);
        controller.update();

        // READ
        while (server.poll(data))
        {
            message m;
            network::deserialize(m, data);
            std::cout << m.str << std::endl;
        }


        // TODO
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    network_thread.join();
    vision_thread.join();
}