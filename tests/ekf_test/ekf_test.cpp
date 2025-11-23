#include "kalman_filter.h"
#include "raylib.h"

#include <iostream>
#include <random>
#include <numbers>
#include <chrono>

constexpr uint32_t fps = 60;
constexpr double delta_time = 1.0 / fps;

struct agent
{
    double x, y;
    double theta; // radians
    double vx, vy;
    double omega; // rad/s

    double vx_prev, vy_prev;
    double omega_prev;

    double wheel_radius;
    double wheel_base_y;

    std::mt19937 rng;
    std::normal_distribution<double> accel_noise{ 0.0, 0.01 };
    std::normal_distribution<double> gyro_noise{ 0.0, 0.001 };
    std::normal_distribution<double> encoder_noise{ 0.0, 0.001 };

    agent() :
        x(0), y(0), theta(0), vx(0), vy(0), omega(0),
        vx_prev(0), vy_prev(0), omega_prev(0),
        wheel_radius(0.05), wheel_base_y(0.3),
        rng(std::random_device{}())
    {
    }

    void apply_control(double accel_input, double angular_accel_input)
    {
        // Store previous velocities
        vx_prev = vx;
        vy_prev = vy;
        omega_prev = omega;

        // Compute forward velocity
        double v_forward = std::sqrt(vx * vx + vy * vy);
        if (std::cos(theta) * vx + std::sin(theta) * vy < 0)
            v_forward = -v_forward;

        // Apply control
        v_forward += accel_input * delta_time;
        omega += angular_accel_input * delta_time;

        // Damping
        v_forward *= 0.98;
        omega *= 0.98;

        // Update orientation
        theta += omega * delta_time;

        // Normalize theta to [-pi, pi]
        while (theta > std::numbers::pi)
        {
            theta -= 2.0 * std::numbers::pi;
        }
        while (theta < -std::numbers::pi)
        {
            theta += 2.0 * std::numbers::pi;
        }

        // Convert to world velocity
        vx = v_forward * std::cos(theta);
        vy = v_forward * std::sin(theta);

        // Update position
        x += vx * delta_time;
        y += vy * delta_time;
    }

    robo::imu_reading simulated_imu_reading()
    {
        // World frame acceleration (computed from velocity change)
        double ax_world = (vx - vx_prev) / delta_time;
        double ay_world = (vy - vy_prev) / delta_time;

        // Rotate to body frame
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);

        // Correct rotation from world to body frame
        double ax_body = cos_theta * ax_world + sin_theta * ay_world;
        double ay_body = -sin_theta * ax_world + cos_theta * ay_world;
        double az_body = 9.81;  // Gravity in body z-axis (assuming level, z-up)

        // Add bias and noise
        ax_body += accel_noise(rng);
        ay_body += accel_noise(rng);
        az_body += accel_noise(rng) * 0.1;

        double gx = 0.0;
        double gy = 0.0;
        double gz = omega;

        // Add noise
        gx += gyro_noise(rng);
        gy += gyro_noise(rng);
        gz += gyro_noise(rng);

        return robo::imu_reading{ ax_body, ay_body, az_body, gx, gy, gz };
    }

    robo::encoder_reading simulated_encoder_reading()
    {
        double v_forward = std::sqrt(vx * vx + vy * vy);
        if (std::cos(theta) * vx + std::sin(theta) * vy < 0)
            v_forward = -v_forward;

        double v_left = v_forward - (omega * wheel_base_y / 2.0);
        double v_right = v_forward + (omega * wheel_base_y / 2.0);

        // Convert to angular velocities (rad/s)
        double left_vel = v_left / wheel_radius;
        double right_vel = v_right / wheel_radius;

        // Add noise
        left_vel += encoder_noise(rng);
        right_vel += encoder_noise(rng);

        return robo::encoder_reading{ left_vel, right_vel };
    }
};

int main()
{
    InitWindow(1200, 800, "UKF Robot Tracking");
    SetTargetFPS(fps);

    agent agent;
    robo::ukf ukf(agent.wheel_radius, agent.wheel_base_y);

    std::vector<Vector2> ground_truth_trail;
    std::vector<Vector2> estimated_trail;

    double scale = 50.0;
    Vector2 origin = { GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };

    int frame_count = 0;

    while (!WindowShouldClose())
    {
        double accel_input = 0.0;
        double angular_accel_input = 0.0;

        // All inputs in radians
        if (IsKeyDown(KEY_W))
        {
            accel_input = 2.0;
        }
        else if (IsKeyDown(KEY_S))
        {
            accel_input = -2.0;
        }

        if (IsKeyDown(KEY_A))
        {
            angular_accel_input = 90.0 * std::numbers::pi / 180.0;
        }
        else if (IsKeyDown(KEY_D))
        {
            angular_accel_input = -90.0 * std::numbers::pi / 180.0;
        }

        agent.apply_control(accel_input, angular_accel_input);

        auto imu_reading = agent.simulated_imu_reading();
        ukf.predict(imu_reading, delta_time);

        // Update with encoders at 30 Hz
        if (frame_count % 2 == 0)
        {
            auto encoder_reading = agent.simulated_encoder_reading();
            ukf.update_encoders(encoder_reading);
        }

        // Get positions
        const auto& transform = ukf.get_transform();
        Vector2 true_position = {
            static_cast<float>(origin.x + agent.x * scale),
            static_cast<float>(origin.y - agent.y * scale)
        };

        Vector2 estimated_position = {
            static_cast<float>(origin.x + transform.translation().x() * scale),
            static_cast<float>(origin.y - transform.translation().y() * scale)
        };

        // Collect trails
        if (frame_count % 3 == 0)
        {
            ground_truth_trail.push_back(true_position);
            estimated_trail.push_back(estimated_position);

            if (ground_truth_trail.size() > 300)
            {
                ground_truth_trail.erase(ground_truth_trail.begin());
                estimated_trail.erase(estimated_trail.begin());
            }
        }

        // Debug output
        if (frame_count % 60 == 0)
        {
            double error = std::sqrt(
                std::pow(transform.translation().x() - agent.x, 2) +
                std::pow(transform.translation().y() - agent.y, 2)
            );
            std::cout << "Frame " << frame_count
                << " | Error: " << error << " m"
                << " | True: (" << agent.x << ", " << agent.y << ")"
                << " | Est: (" << transform.translation().x() << ", "
                << transform.translation().y() << ")" << std::endl;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw grid
        for (int i = -20; i <= 20; i++)
        {
            float pos_x = origin.x + i * scale;
            DrawLine(pos_x, 0, pos_x, GetScreenHeight(), i == 0 ? DARKGRAY : LIGHTGRAY);
        }
        for (int i = -20; i <= 20; i++)
        {
            float pos_y = origin.y + i * scale;
            DrawLine(0, pos_y, GetScreenWidth(), pos_y, i == 0 ? DARKGRAY : LIGHTGRAY);
        }

        // Draw trails
        for (size_t i = 1; i < ground_truth_trail.size(); ++i)
        {
            DrawLineEx(ground_truth_trail[i - 1], ground_truth_trail[i], 2.0f, BLUE);
        }

        for (size_t i = 1; i < estimated_trail.size(); ++i)
        {
            DrawLineEx(estimated_trail[i - 1], estimated_trail[i], 2.0f, RED);
        }

        // Draw true robot
        DrawCircleV(true_position, 10, BLUE);
        Vector2 true_direction = {
            static_cast<float>(true_position.x + 20 * std::cos(agent.theta)),
            static_cast<float>(true_position.y - 20 * std::sin(agent.theta))
        };
        DrawLineEx(true_position, true_direction, 3, DARKBLUE);

        // Draw estimated robot
        DrawCircleV(estimated_position, 10, RED);

        Eigen::Matrix3d rot = transform.rotation();
        double estimated_yaw = std::atan2(rot(1, 0), rot(0, 0));

        Vector2 estimated_direction = {
            static_cast<float>(estimated_position.x + 20 * std::cos(estimated_yaw)),
            static_cast<float>(estimated_position.y - 20 * std::sin(estimated_yaw))
        };
        DrawLineEx(estimated_position, estimated_direction, 3, MAROON);

        // Draw legend and info
        DrawText("BLUE = Ground Truth", 10, 10, 16, BLUE);
        DrawText("RED = UKF Estimate", 10, 30, 16, RED);

        // Position error
        double pos_error = std::sqrt(
            std::pow(transform.translation().x() - agent.x, 2) +
            std::pow(transform.translation().y() - agent.y, 2)
        );

        // Angle error
        double angle_error = estimated_yaw - agent.theta;
        // Normalize to [-pi, pi]
        while (angle_error > std::numbers::pi)
        {
            angle_error -= 2.0 * std::numbers::pi;
        }
        while (angle_error < -std::numbers::pi)
        {
            angle_error += 2.0 * std::numbers::pi;
        }

        DrawText(TextFormat("Position Error: %.3f m", pos_error), 10, 90, 16, BLACK);
        DrawText(TextFormat("Angle Error: %.3f (%.4f rad)", angle_error * 180.0 / std::numbers::pi, angle_error), 10, 110, 16, BLACK);
        DrawText(TextFormat("True Pos: (%.3f, %.2f) m", agent.x, agent.y), 10, 140, 16, BLUE);
        DrawText(TextFormat("Est Pos: (%.3f, %.3f) m", transform.translation().x(), transform.translation().y()), 10, 160, 16, RED);
        DrawText(TextFormat("True Yaw: %.3f (%.4f rad)", agent.theta * 180.0 / std::numbers::pi, agent.theta), 10, 190, 16, BLUE);
        DrawText(TextFormat("Est Yaw: %.3f (%.4f rad)", estimated_yaw * 180.0 / std::numbers::pi, estimated_yaw), 10, 210, 16, RED);

        DrawFPS(10, GetScreenHeight() - 30);

        EndDrawing();

        ++frame_count;
    }

    CloseWindow();
    return 0;
}