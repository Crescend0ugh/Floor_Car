#include "ekf.h"
#include "raylib.h"

#include <iostream>
#include <random>
#include <chrono>

const uint32_t fps = 60;
const double delta_time = 1.0f / fps;

struct agent
{
    double x, y;
    double theta;
    double vx, vy;
    double omega;

    // Store previous values to compute actual acceleration
    double vx_prev, vy_prev;
    double omega_prev;

    double wheel_radius;
    double wheel_base_y;
    double max_accel;
    double max_angular_accel;

    std::mt19937 rng;
    std::normal_distribution<double> accel_noise{ 0.0f, 0.01f };
    std::normal_distribution<double> gyro_noise{ 0.0f, 0.001f };
    std::normal_distribution<double> encoder_noise{ 0.0f, 0.002f };

    agent() :
        x(0), y(0), theta(0), vx(0), vy(0), omega(0),
        vx_prev(0), vy_prev(0), omega_prev(0),
        wheel_radius(0.05), wheel_base_y(0.3),
        max_accel(2.0), max_angular_accel(2.0),
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
        v_forward *= 0.98f;
        omega *= 0.98f;

        // Update orientation
        theta += omega * delta_time;

        // Convert to world velocity
        vx = v_forward * std::cos(theta);
        vy = v_forward * std::sin(theta);

        // Update position
        x += vx * delta_time;
        y += vy * delta_time;
    }

    robo::imu_reading simulated_imu_reading()
    {
        // Compute world-frame acceleration (velocity change)
        double ax_world = (vx - vx_prev) / delta_time;
        double ay_world = (vy - vy_prev) / delta_time;

        // Rotate to body frame
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);

        double ax_body = cos_theta * ax_world + sin_theta * ay_world;
        double ay_body = -sin_theta * ax_world + cos_theta * ay_world;
        double az_body = 9.81f;  // Gravity (stationary reading)

        // Add noise
        ax_body += accel_noise(rng);
        ay_body += accel_noise(rng);
        az_body += accel_noise(rng) * 0.1f;

        // Gyroscope: angular velocity change (discrete time)
        double gx = 0.0f;
        double gy = 0.0f;
        double gz = (omega - omega_prev) / delta_time;  // Angular acceleration

        // Scale to discrete time step (change over dt)
        gx *= delta_time;
        gy *= delta_time;
        gz *= delta_time;

        // Add noise
        gx += gyro_noise(rng) * delta_time;
        gy += gyro_noise(rng) * delta_time;
        gz += gyro_noise(rng) * delta_time;

        //return robo::imu_reading{ 0 };
        return robo::imu_reading{ ax_body, ay_body, az_body, gx, gy, gz };
    }

    robo::encoder_reading simulated_encoder_reading()
    {
        double v_forward = std::sqrt(vx * vx + vy * vy);
        if (std::cos(theta) * vx + std::sin(theta) * vy < 0)
            v_forward = -v_forward;

        double v_left = v_forward - (omega * wheel_base_y / 2.0f);
        double v_right = v_forward + (omega * wheel_base_y / 2.0f);

        // Convert to angular velocities (rad/s)
        double left_vel = v_left / wheel_radius;
        double right_vel = v_right / wheel_radius;

        // Add noise
        left_vel += encoder_noise(rng);
        right_vel += encoder_noise(rng);

        //return robo::encoder_reading{ 0 };
        return robo::encoder_reading{ left_vel, right_vel };
    }
};

int main()
{
    InitWindow(1200, 800, "UKF Robot Tracking");
    SetTargetFPS(fps);

    agent agent;
    robo::ukf ukf(0.05f, 0.3f);

    std::vector<Vector2> ground_truth_trail;
    std::vector<Vector2> estimated_trail;

    double scale = 50.0f;
    Vector2 origin = { GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };

    int frame_count = 0;

    while (!WindowShouldClose())
    {
        double accel_input = 0.0f;
        double angular_accel_input = 0.0f;

        if (IsKeyDown(KEY_W))
            accel_input = 2.0f;
        else if (IsKeyDown(KEY_S))
            accel_input = -2.0f;
        else if (IsKeyDown(KEY_A))
            angular_accel_input = 2.0f;
        else if (IsKeyDown(KEY_D))
            angular_accel_input = -2.0f;

        agent.apply_control(accel_input, angular_accel_input);

        auto imu_reading = agent.simulated_imu_reading();
        ukf.predict(imu_reading);

        // Update with encoders at 30 Hz
        if (frame_count % 2 == 0)
        {
            auto encoder_reading = agent.simulated_encoder_reading();
            ukf.update_encoders(encoder_reading);
        }

        // Get positions
        const auto& transform = ukf.get_transform();
        Vector2 true_position = {
            origin.x + agent.x * scale,
            origin.y - agent.y * scale
        };

        Vector2 estimated_position = {
            origin.x + transform.translation().x() * scale,
            origin.y - transform.translation().y() * scale
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
            double pos = origin.x + i * scale;
            DrawLine(pos, 0, pos, GetScreenHeight(), i == 0 ? DARKGRAY : LIGHTGRAY);
        }
        for (int i = -20; i <= 20; i++)
        {
            double pos = origin.y + i * scale;
            DrawLine(0, pos, GetScreenWidth(), pos, i == 0 ? DARKGRAY : LIGHTGRAY);
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
            true_position.x + 20 * std::cos(agent.theta),
            true_position.y - 20 * std::sin(agent.theta)
        };
        DrawLineEx(true_position, true_direction, 3, DARKBLUE);

        // Draw estimated robot
        DrawCircleV(estimated_position, 10, RED);
        Eigen::Vector3d euler = transform.rotation().eulerAngles(0, 1, 2);
        double estimated_yaw = euler(2);
        Vector2 estimated_direction = {
            estimated_position.x + 20 * std::cos(estimated_yaw),
            estimated_position.y - 20 * std::sin(estimated_yaw)
        };
        DrawLineEx(estimated_position, estimated_direction, 3, MAROON);

        DrawText("BLUE = Ground Truth", 10, 130, 16, BLUE);
        DrawText("RED = UKF Estimate", 10, 150, 16, RED);

        // Position error
        double error = std::sqrt(
            std::pow(transform.translation().x() - true_position.x, 2) +
            std::pow(transform.translation().y() - true_position.y, 2)
        );

        DrawText(TextFormat("Position Error: %.3f m", error), 10, 190, 16, BLACK);
        DrawText(TextFormat("True Pos: (%.2f, %.2f)", agent.x, agent.y), 10, 210, 16, BLUE);
        DrawText(TextFormat("Est Pos: (%.2f, %.2f)", transform.translation().x(), transform.translation().y()), 10, 230, 16, RED);
        DrawText(TextFormat("True Yaw: %.2f°", agent.theta * 180.0 / PI), 10, 250, 16, BLUE);
        DrawText(TextFormat("Est Yaw: %.2f°", estimated_yaw * 180.0 / PI), 10, 270, 16, RED);

        DrawFPS(10, GetScreenHeight() - 30);

        EndDrawing();

        ++frame_count;
    }

    CloseWindow();
    return 0;
}