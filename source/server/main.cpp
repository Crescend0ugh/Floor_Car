//
// Created by avsom on 10/4/2025.
//
#if defined(_WIN32)
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif

#include "mapgen.h"
#include "navmesh.h"
#include "path.h"
#include "network.h"
#include "network_data.h"
#include "vision.h"
#include "controller.h"
#include "serialib.h"
#include "arduino_serial.h"
#include "voice_detection.h"
#include "object_tracker.h"

#include <raylib.h>
#include <opencv2/core/utils/logger.hpp>
#include <iostream>
#include <thread>
#include <functional>
#include <mutex>

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

// Vision
robo::vision vision;
robo::object_tracker object_tracker;

// Voice detection
robo::voice_detection voice_detection(valid_detection_class_names);

// Arduino serial
robo::arduino_serial arduino_serial;

// Rotation and translation relative to world coordinate system
Eigen::Affine3f robot_world_pose = Eigen::Affine3f::Identity(); 
robo::point_cloud world_point_cloud;

// Navmesh, meshing
robo::navmesh navmesh(robo::navigation_params
    {
        .agent_radius = 0.2f,
        .agent_height = 0.5f,
        .max_slope = 30.0f,
        .max_climb = 0.1f
    }
);
robo::navgeometry navgeometry;
robo::path path;
asio::io_context meshing_io_context;
asio::steady_timer meshing_timer(meshing_io_context);

robo::controller controller;

constexpr float delta_yaw_epsilon = 0.3f;
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
                std::cout << "Detection failed" << std::endl;
                return;
            }

            std::cout << "Detected " << result.detections.size() << " objects in " << result.processing_time.count() << "ms" << std::endl;

            // Send results to clients
            if (server.get_client_count() > 0) 
            {
                server.send(robo::network::protocol::camera_feed, vision.serialize_detection_results(result));
            }

            if (!result.detections.empty()) 
            {
                vision.stop_continuous();
                std::cout << "got detection -> stop continuous" << std::endl;
                // extrapolation
            }
            else 
            {
                // do nothing
            }
        }
    );
}

static void run_meshing()
{
    if (world_point_cloud.points->points.empty())
    {
        meshing_timer.expires_at(meshing_timer.expiry() + std::chrono::seconds(1));
        meshing_timer.async_wait(std::bind(&run_meshing));
        return;
    }

    // Maybe downsample in robo::point_cloud?
    // auto downsampled = world_point_cloud.voxel_grid_downsample();
    auto reconstructed_mesh = world_point_cloud.reconstruct_mesh_from_points(); // This will take a while

    auto components = world_point_cloud.extract_mesh_components(reconstructed_mesh);
    navgeometry.load(components.vertices, components.triangles);

    // TODO: ALSO FIGURE OUT BOUNDS FOR NAVGEOMETRY TO MAKE NAVMESH GENERATION A LOT FASTER
    navmesh.set_geometry(navgeometry);
    navmesh.build();
    path.init(&navmesh);
    // path.set_start(); // Our current position

    meshing_timer.expires_at(meshing_timer.expiry() + std::chrono::seconds(10));
    meshing_timer.async_wait(std::bind(&run_meshing));
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