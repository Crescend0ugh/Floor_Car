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

asio::io_context network_io_context;
network::server server(network_io_context, 12345);

asio::io_context vision_io_context;
robo::vision vision;

// Delay between each object detection cycle
const auto vision_interval = std::chrono::milliseconds(50);
asio::basic_waitable_timer<std::chrono::steady_clock> vision_timer(vision_io_context, vision_interval);

robo::controller controller;

// Objects we're looking for
std::vector<std::string> valid_detection_class_names = {"apple", "orange", "sports ball"};

static bool is_valid_detection(int label)
{
    return std::find(
        valid_detection_class_names.begin(), 
        valid_detection_class_names.end(), 
        yolo::get_detection_class_name(label)
    ) == valid_detection_class_names.end();
}

static void run_vision(network::server& server, robo::vision& vision, asio::steady_timer& vision_timer)
{
    if (vision.is_enabled)
    {
        bool succeeded = vision.grab_frame();

        if (succeeded)
        {
            vision.detect_from_camera();
            // TODO: Estimate 3D positions of detection bounding box centers
            // vision.estimate_3d_positions(...);
        }

        // Send results to clients, if any are connected
        if (server.get_client_count() > 0)
        {
            auto serialized = vision.serialize_detection_results();
            server.send(robo::network::protocol::camera_feed, serialized);
        }
    }

    // Schedule next object detection cycle
    vision_timer.expires_at(std::chrono::steady_clock::now() + vision_interval);
    vision_timer.async_wait(std::bind(&run_vision, std::ref(server), std::ref(vision), std::ref(vision_timer)));
}

static void handle_client_messages()
{
    network::received_data data;
    while (server.poll(data))
    {
        switch (data.protocol_id)
        {

        case (robo::network::protocol::rc):
        {
            robo::network::rc_command command;
            network::deserialize(command, data);
            controller.send_rc_command_to_arduino(command);

            break;
        }

        default:
            break;

        }
    }
}

int main(int argc, char* argv[]) 
{
    std::thread network_thread([&] {
        network_io_context.run();
    });

    std::thread vision_thread([&] {
        vision_io_context.run();
    });
    
    vision_timer.async_wait(std::bind(&run_vision, std::ref(server), std::ref(vision), std::ref(vision_timer)));

    while (1)
    {
        controller.update();

        for (const auto& detection : vision.detections)
        {
            if (!is_valid_detection(detection.label))
            {
                continue;
            };

            float delta_yaw = vision.compute_delta_yaw_to_detection_center(detection);
            std::cout << delta_yaw << std::endl;

            controller.rotate_by(delta_yaw);
            break;
        }

        handle_client_messages();

        // TODO
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    network_thread.join();
    vision_thread.join();
}