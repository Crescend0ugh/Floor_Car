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

// Set to false when we Ctrl+C and stops the main loop
std::atomic<bool> is_running = true;

asio::io_context network_io_context;
network::server server(network_io_context, 12345);
asio::signal_set shutdown_signals(network_io_context, SIGINT, SIGTERM);

asio::io_context vision_io_context;
robo::vision vision;

// Delay between each object detection cycle
const auto vision_interval = std::chrono::milliseconds(16);
asio::basic_waitable_timer<std::chrono::steady_clock> vision_timer(vision_io_context, vision_interval);

robo::controller controller;

// Objects we're looking for
std::vector<std::string> valid_detection_class_names = {"apple", "orange", "sports ball"};

// PLACEHOLDER FOR BENCHMARK
float current_delta_yaw = 0.0f; // Degrees
float delta_yaw_epsilon = 0.1f; // Degrees

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

    shutdown_signals.async_wait([&](const asio::error_code& error, int signal_number) {
        if (!error) {
            std::cout << "Server shutting down. Received signal: " << signal_number << std::endl;
            server.shutdown();

            network_io_context.stop();
            vision_io_context.stop();

            is_running.store(false);
        }
    });

    while (is_running.load())
    {
        controller.update();

        for (const auto& detection : vision.detections)
        {
            if (!is_valid_detection(detection.label))
            {
                continue;
            };

            float delta_yaw = vision.compute_delta_yaw_to_detection_center(detection);

#if 0
            // Turn until we're facing the object
            if (std::abs(delta_yaw) > delta_yaw_epsilon)
            {
                if (delta_yaw > 0.0f) // To the right
                {
                    controller.send_rc_command_to_arduino(robo::network::rc_command::d);
                }
                else if (delta_yaw < 0.0f) // Object is to the left
                {
                    controller.send_rc_command_to_arduino(robo::network::rc_command::a);
                }
            }
            // Move forward?
#endif
            
            break;
        }

        handle_client_messages();

        // std::cout << std::chrono::steady_clock::now().time_since_epoch() << std::endl;
        // TODO
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    network_thread.join();
    vision_thread.join();

    return 0;
}