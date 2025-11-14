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

// Networking
asio::io_context network_io_context;
network::server server(network_io_context, 12345);
asio::signal_set shutdown_signals(network_io_context, SIGINT, SIGTERM);

// Vision
robo::vision vision;
asio::io_context vision_io_context;
asio::steady_timer vision_timer(vision_io_context);
asio::executor_work_guard<asio::io_context::executor_type> vision_work_guard = asio::make_work_guard(vision_io_context);

robo::object_tracker object_tracker;

// Voice detection
robo::voice_detection voice_detection;
asio::io_context voice_detection_io_context;
asio::steady_timer voice_detection_timer(voice_detection_io_context);

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

// Objects we're looking for
std::vector<std::string> valid_detection_class_names = {"apple", "orange", "sports ball"};

constexpr float delta_yaw_epsilon = 0.1f;
constexpr float pickup_y_threshold = 0.8f;
static std::optional<robo::tracking_result> target = std::nullopt;

static bool is_valid_detection(int label)
{
    return std::find(
        valid_detection_class_names.begin(), 
        valid_detection_class_names.end(), 
        yolo::get_detection_class_name(label)
    ) == valid_detection_class_names.end();
}

static void run_voice_detection(robo::voice_detection& voice_detection, asio::steady_timer& timer)
{
    std::string command = voice_detection.process();
    if (!command.empty())
    {
        std::cout << "GOT VOICE COMMAND: " << command << std::endl;
    }

    timer.expires_at(std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
    timer.async_wait(std::bind(&run_voice_detection, std::ref(voice_detection), std::ref(timer)));
}

static void run_lidar_sweep()
{
}

// Runs object detection
static void run_vision(network::server& server, robo::vision& vision, asio::steady_timer& vision_timer)
{
    if (!vision.is_enabled)
    {
        return;
    }

    // TODO: Grab the current transform matrix of the robot here
    // Eigen::Affine3f pose_at_capture = ...
    bool succeeded = vision.grab_frame();

    if (succeeded)
    {
        vision.detect_from_camera();
        // Filter the current point cloud to only get points in front of the camera. Then pass that into estimate_detection_3d_bounds
        /*
        auto obbs = vision.estimate_detection_3d_bounds();
        for (const auto& obb : obbs)
        {
            if (!is_valid_detection(obb.label))
            {
                continue;
            }

            robo::vector3f goal(obb.center.x, obb.center.y, obb.center.z);

            // Transform goal with pose_at_capture

            path.set_end(goal);
            break;
        }
        */
    }

    // Send results to clients, if any are connected
    if (server.get_client_count() > 0)
    {
        server.send(robo::network::protocol::camera_feed, vision.serialize_detection_results());
    }

    // Didn't get anything, so rerun
    if (vision.detections.size() == 0)
    {
        vision_timer.expires_at(std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
        vision_timer.async_wait(std::bind(&run_vision, std::ref(server), std::ref(vision), std::ref(vision_timer)));
    }
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

    voice_detection.init();

    vision_timer.expires_after(std::chrono::seconds(1));
    vision_timer.async_wait(std::bind(&run_vision, std::ref(server), std::ref(vision), std::ref(vision_timer)));

    voice_detection_timer.expires_after(std::chrono::milliseconds(100));
    voice_detection_timer.async_wait(std::bind(&run_voice_detection, std::ref(voice_detection), std::ref(voice_detection_timer)));

    shutdown_signals.async_wait([&](const asio::error_code& error, int signal_number) {
        if (!error) 
        {
            std::cout << "Server shutting down. Received signal: " << signal_number << std::endl;
            server.shutdown();

            network_io_context.stop();
            vision_io_context.stop();
            voice_detection_io_context.stop();

            is_running.store(false);
        }
    });

    std::thread voice_detection_thread([&] { voice_detection_io_context.run(); });
    std::thread network_thread([&] { network_io_context.run(); });
    std::thread vision_thread([&] { vision_io_context.run(); });

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
                vision.detections.clear();
                target = std::nullopt;

                std::cout << "Rerunning object detection" << std::endl;

                // Rerun object detection
                vision_timer.expires_at(std::chrono::steady_clock::now() + std::chrono::milliseconds(1));
                vision_timer.async_wait(std::bind(&run_vision, std::ref(server), std::ref(vision), std::ref(vision_timer)));
            }
            else
            {
                for (const auto& result : results)
                {
                    if (!target.has_value())
                    {
                        target = result;
                    }

                    // Is this tracker result for our target?
                    if (result.id == target.value().id)
                    {
                        if (!result.is_located)
                        {
                            // TODO: Target is out of view
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
            if (vision.detections.size() > 0)
            {
                for (const auto& detection : vision.detections)
                {
                    if (!is_valid_detection(detection.label))
                    {
                        continue;
                    };

                    object_tracker.init(vision.undistorted_camera_frame, detection);
                }
            }
        }

        if (target.has_value())
        {
            const auto& result = target.value();
            float delta_yaw = vision.compute_delta_yaw_to_bbox_center(result.bbox);

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
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    vision_work_guard.reset();

    network_thread.join();
    vision_thread.join();
    voice_detection_thread.join();
    arduino_serial.close();

    return 0;
}