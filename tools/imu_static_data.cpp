/*
    Collect static IMU data for Allan variance analysis
    Collects 3+ hours of stationary IMU data and saves to a CSV
    This should only need to be done once per IMU
    https://www.mathworks.com/help/nav/ug/inertial-sensor-noise-analysis-using-allan-variance.html
*/

#include "MPU6050.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>

volatile bool running = true;

constexpr uint32_t duration_hours = 3;
constexpr uint32_t sample_rate_hz = 100;
constexpr uint64_t total_samples = duration_hours * 3600LL * sample_rate_hz;
constexpr uint32_t sleep_time_ms = 1000 / sample_rate_hz;

std::string output_file = "imu_static_data.csv";

static void signal_handler(int signum) 
{
    std::cout << "\nInterrupt signal received. Stopping data collection...\n";
    running = false;
}

int main() {
    std::cout << "=== IMU Static Data Collection for Allan Variance ===" << std::endl;
    std::cout << "Make sure IMU is stationary and on a stable surface!" << std::endl;
    std::cout << "Press Ctrl+C to stop early." << std::endl;
    std::cout << "\nStarting in 5 seconds...\n" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Register signal handler
    signal(SIGINT, signal_handler);

    // Initialize MPU6050 with manual polling
    MPU6050 mpu(0x68, false);

    // Open output file
    std::ofstream file(output_file);
    if (!file.is_open()) 
    {
        std::cerr << "Error: Could not open output file " << output_file << std::endl;
        return 1;
    }

    // Write CSV header
    file << "timestamp_ns,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z" << std::endl;

    std::cout << "Expected total samples: " << total_samples << std::endl;
    std::cout << "Collection started at: " << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;

    uint64_t sample_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_print_time = start_time;

    while (running && sample_count < total_samples) 
    {
        // Get current timestamp and read raw IMU data
        auto now = std::chrono::high_resolution_clock::now();
        auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

        float ax, ay, az, gx, gy, gz;
        mpu.getAccelRaw(&ax, &ay, &az);
        mpu.getGyroRaw(&gx, &gy, &gz);

        // Write to file
        file << timestamp_ns << ","
            << ax << "," << ay << "," << az << ","
            << gx << "," << gy << "," << gz << std::endl;

        sample_count++;

        // Print progress every 10 seconds
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 10) 
        {
            double progress = (double)sample_count / total_samples * 100.0;
            
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

            std::cout << "Progress: " << std::fixed << std::setprecision(2) << progress << "% "
                << "(" << sample_count << "/" << total_samples << " samples, "
                << "elapsed: " << elapsed << "s)" << std::endl;

            last_print_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }

    file.close();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    std::cout << "\n=== Collection Complete ===" << std::endl;
    std::cout << "Total samples collected: " << sample_count << std::endl;
    std::cout << "Total duration: " << total_duration << " seconds (" << total_duration / 3600.0 << " hours)" << std::endl;
    std::cout << "Average sample rate: " << (double)sample_count / total_duration << " Hz" << std::endl;
    std::cout << "Data saved to: " << output_file << std::endl;

    return 0;
}