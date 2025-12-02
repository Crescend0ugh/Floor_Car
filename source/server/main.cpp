//
// Created by avsom on 10/4/2025.
//
#if defined(_WIN32)
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif

#include "transforms.h"
#include "world.h"
#include "network.h"
#include "network_data.h"
#include "vision.h"
#include "controller.h"
#include "arduino_serial.h"
#include "voice_detection.h"
#include "object_tracker.h"

#include <opencv2/core/utils/logger.hpp>
#include <iostream>
#include <thread>
#include <functional>

// Set to false when we Ctrl+C and stops the main loop
std::atomic<bool> is_running = true;

// Objects we're looking for
std::vector<std::string> valid_detection_class_names = { "apple", "orange", "sports ball", "cell phone" };

enum class state
{
    listening_for_voice_command,
    running_object_detection,
    wandering,

    meshing,
    computing_path,
    navigating,
    picking_up,
    returning,
};

std::atomic<state> robo_state = state::listening_for_voice_command;

// Networking
asio::io_context network_context;
network::server server(network_context, 12345);
asio::signal_set shutdown_signals(network_context, SIGINT, SIGTERM);

robo::transforms& transforms = robo::transforms::get();

// Vision
robo::vision vision;
robo::object_tracker object_tracker;

// Voice detection
robo::voice_detection voice_detection(valid_detection_class_names);

// Arduino serial
robo::arduino_serial arduino_serial;

// Navmesh, meshing
robo::world world;

robo::controller controller;

constexpr float delta_yaw_epsilon = 3.0f;
constexpr float pickup_y_threshold = 0.8f;
static std::optional<robo::tracking_result> target = std::nullopt;

static bool is_valid_detection(int label)
{
    return std::find(
        valid_detection_class_names.begin(), 
        valid_detection_class_names.end(), 
        yolo::get_detection_class_name(label)
    ) != valid_detection_class_names.end();
}

static void run_voice_detection(robo::voice_detection& voice)
{
    voice.start_continuous(
        [&voice](const robo::voice_result& result)
        {
            if (!result.detected)
            {
                return;
            }

            const std::string& command = result.command;
            std::cout << "Got command: " << command << std::endl;

            voice.stop_continuous();
        }
    );
}

static void run_lidar_sweep()
{
}

static void run_object_detection(network::server& server, robo::vision& vision)
{
    vision.start_continuous(
        [&server, &vision](const robo::vision_result& result)
        {
            if (!result.success) 
            {
                std::cout << "Object detection failed" << std::endl;
                return;
            }

            std::cout << std::format("Detected {} objects in {}ms\n", result.detections.size(), result.processing_time.count());

            // Send results to clients
            if (server.get_client_count() > 0) 
            {
                server.send(robo::network::protocol::camera_feed, vision.serialize_detection_results(result));
            }
        }
    );
}

// Call with is_returning = true for the return trip so that the entire point cloud gets meshed
static void run_meshing(robo::world& world, bool is_returning = false)
{
    world.is_returning.store(is_returning);
    world.trigger_single(
        [&world](const robo::meshing_result& result)
        {
            if (!result.success)
            {
                std::cout << "Meshing failed" << std::endl;
                return;
            }

            // TODO: Send results to client
        }
    );
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
            arduino_serial.send_rc_command(command);

            break;
        }

        default:
            break;

        }
    }
}

int main(int argc, char* argv[]) 
{
    transforms.load();
    cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_ERROR);

    if (!vision.initialize()) 
    {
        std::cerr << "Failed to initialize vision" << std::endl;
        return 1;
    }

    if (!voice_detection.initialize())
    {
        std::cerr << "Failed to initialize voice" << std::endl;
        return 1;
    }

    world.initialize();
    
    shutdown_signals.async_wait([&](const asio::error_code& error, int signal_number) {
        if (!error) 
        {
            std::cout << "\nServer shutting down. Received signal: " << signal_number << std::endl;

            server.shutdown();
            vision.shutdown();
            voice_detection.shutdown();

            is_running.store(false);
        }
    });

    std::thread network_thread([&] { network_context.run(); });
    run_object_detection(std::ref(server), std::ref(vision));
    run_voice_detection(std::ref(voice_detection));

    while (is_running.load())
    {
        controller.update();
        arduino_serial.read();


        if (!object_tracker.is_empty())
        {
            bool succeeded = vision.grab_frame();
            const auto& image = vision.capture_frame();

            // This does not take a trivial amount of time
            auto [results, located_something] = object_tracker.infer(image);

            // All the objects we were tracking are out of frame, so begin the object detection loop again
            if (!located_something)
            {
                object_tracker.clear_trackers();
                vision.clear_detections();
                target = std::nullopt;

                std::cout << "Rerunning object detection" << std::endl;
                run_object_detection(std::ref(server), std::ref(vision));
            }
            else
            {
                for (const auto& result : results)
                {
                    // No target? Set this one as it
                    if (!target.has_value())
                    {
                        target = result;
                    }
                    else if (result.id == target.value().id)
                    {
                        if (!result.is_located)
                        {
                            // TODO: Target is out of view
                            target = std::nullopt;
                            break;
                        }

                        // Update target with latest information
                        target = result;
                        break;
                    }
                }

                if (server.get_client_count() > 0)
                {
                    server.send(robo::network::protocol::camera_feed, object_tracker.serialize_tracker_results(image, results));
                }
            }
        }
        else
        {
            target = std::nullopt;
            bool all_invalid = true;
            if (vision.get_detections().size() > 0)
            {
                for (const auto& detection : vision.get_detections())
                {
                    std::cout << yolo::get_detection_class_name(detection.label) << std::endl;

                    if (!is_valid_detection(detection.label))
                    {
                        continue;
                    };

                    object_tracker.init(vision.undistorted_camera_frame, detection);
                    all_invalid = false;
                }

                vision.clear_detections();
            }
            
            // Found at least 1 valid detection -> Stop object detection and start tracking
            if (!all_invalid)
            {
                vision.stop_continuous();
            }
        }

        if (target.has_value())
        {
            const auto& result = target.value();
            float delta_yaw = vision.compute_delta_yaw_to_bbox_center(result.bbox);

            std::cout << delta_yaw << std::endl;

            // Turn until we're facing the object
            if (std::abs(delta_yaw) > delta_yaw_epsilon)
            {
                if (delta_yaw > 0.0f) // To the right
                {
                    arduino_serial.send_rc_command(robo::network::rc_command::d);
                }
                else if (delta_yaw < 0.0f) // Object is to the left
                {
                    arduino_serial.send_rc_command(robo::network::rc_command::a);
                }
            }
            else
            {
                // If the top left corner of the bbox is below the y threshold, try to pickup
                if (result.bbox.y >= pickup_y_threshold * vision.image_size.height)
                {
                    // CCW or CW depends on how the servo is positioned
                    arduino_serial.send_rc_command(robo::network::rc_command::servo_cw);
                }
                else
                {
                    arduino_serial.send_rc_command(robo::network::rc_command::w);
                }
            }
        }

        handle_client_messages();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    network_thread.join();
    arduino_serial.close();

    return 0;
}