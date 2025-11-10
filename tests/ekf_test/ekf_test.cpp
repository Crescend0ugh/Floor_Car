#include "ekf.h"
#include "raylib.h"

#include <iostream>
#include <random>
#include <chrono>

int main()
{
    // Noise simulation
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<float> noise(0, 1);

    robo::ukf ukf(0.03f, 0.3f);

    float imu_noise_std = 0.05f;
    float delta_time = 0.05;

    for (int i = 0; i < 100; ++i)
    {
        robo::imu_reading imu = { 0 };
        imu.ax = 0.05f;
        imu.ay = 0.06f;
        imu.az = 0.00f;
        imu.gx = 0.00f;
        imu.gy = 0.00f;
        imu.gz = 0.03f;

        robo::encoder_reading encoder = { 0 };

        ukf.predict(imu);
        auto pose = ukf.get_transform();
    }

    InitWindow(800, 450, "UKF");

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 10.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        DrawGrid(10, 1.0f);

        EndMode3D();
        DrawFPS(10, 10);

        EndDrawing();
    }

    CloseWindow();

	return 0;
}