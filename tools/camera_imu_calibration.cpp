/*
    Collect IMU-Camera calibration data
    Captures synchronized camera images and IMU measurements with checkerboard
    https://www.mathworks.com/help/nav/ug/estimate-camera-to-imu-transformation-using-extrinsic-calibration.html
*/

#include "MPU6050.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sys/stat.h>
#include <csignal>

volatile bool running = true;

constexpr uint32_t sample_rate_hz = 100;
constexpr uint32_t sleep_time_ms = 1000 / sample_rate_hz;

std::string output_dir = "imu_camera_calib_data";
std::string images_dir = output_dir + "/images";
std::string imu_file = output_dir + "/imu_data.csv";

static void signal_handler(int signum) 
{
    std::cout << "\nInterrupt signal received. Stopping collection...\n";
    running = false;
}

static bool create_directory(const std::string& path) 
{
    struct stat info;

    if (stat(path.c_str(), &info) != 0) 
    {
        return mkdir(path.c_str(), 0777) == 0;
    }

    return true;
}

int main() 
{
    std::cout << "=== IMU-Camera Calibration Data Collection ===" << std::endl;
    std::cout << "\nInstructions:" << std::endl;
    std::cout << "1. Place a checkerboard in view of the camera" << std::endl;
    std::cout << "2. Move the sensor rig (camera + IMU) smoothly:" << std::endl;
    std::cout << "   - Rotate around ALL IMU axes (roll, pitch, yaw)" << std::endl;
    std::cout << "   - Translate along ALL IMU axes (x, y, z)" << std::endl;
    std::cout << "   - Move at MODERATE speed (avoid motion blur)" << std::endl;
    std::cout << "   - NO 180-degree flips!" << std::endl;
    std::cout << "3. Capture 50-100 images at different poses" << std::endl;
    std::cout << "4. Press 's' to save an image + IMU pair" << std::endl;
    std::cout << "5. Press 'q' to quit" << std::endl;
    std::cout << "\nStarting in 3 seconds...\n" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Register signal handler
    signal(SIGINT, signal_handler);

    // Create output directories
    create_directory(output_dir);
    create_directory(images_dir);

    // Initialize camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) 
    {
        std::cerr << "Error: Could not open camera!" << std::endl;
        return 1;
    }

    // Set camera properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    // Initialize IMU
    MPU6050 mpu(0x68, false);

    // Open IMU data file
    std::ofstream imu_csv(imu_file);
    if (!imu_csv.is_open()) 
    {
        std::cerr << "Error: Could not open IMU data file!" << std::endl;
        return 1;
    }

    // Write CSV header
    imu_csv << "image_id,timestamp_ns,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z" << std::endl;

    uint32_t image_count = 0;
    cv::Mat frame;

    std::cout << "Camera initialized. Press 's' to save, 'q' to quit." << std::endl;

    // Collect IMU data in a separate thread
    std::vector<std::tuple<uint64_t, float, float, float, float, float, float>> imu_buffer;
    std::mutex imu_mutex;

    std::thread imu_thread(
        [&]() 
        {
            auto start_time = std::chrono::high_resolution_clock::now();

            while (running) 
            {
                auto now = std::chrono::high_resolution_clock::now();
                auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

                float ax, ay, az, gx, gy, gz;
                mpu.getAccel(&ax, &ay, &az);
                mpu.getGyro(&gx, &gy, &gz);

                {
                    std::lock_guard<std::mutex> lock(imu_mutex);
                    imu_buffer.push_back(std::make_tuple(timestamp_ns, ax, ay, az, gx, gy, gz));
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            }
        }
    );

    while (running) 
    {
        cap >> frame;
        if (frame.empty()) 
        {
            std::cerr << "Error: Could not read frame from camera!" << std::endl;
            break;
        }

        // cv::imshow("IMU-Camera Calibration (Press 's' to save, 'q' to quit)", frame);

        int key = cv::waitKey(30);

        if (key == 'q' || key == 27) // 'q' or ESC
        {
            running = false;
            break;
        }
        else if (key == 's' || key == 'S') 
        {
            // Save image
            std::string image_filename = images_dir + "/image_" + std::to_string(image_count) + ".jpg";
            cv::imwrite(image_filename, frame);

            // Get timestamp
            auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()
            ).count();

            // Match the image with the latest IMU reading
            float ax, ay, az, gx, gy, gz;
            {
                std::lock_guard<std::mutex> lock(imu_mutex);
                if (!imu_buffer.empty()) 
                {
                    auto& imu_data = imu_buffer.back();
                    ax = std::get<1>(imu_data);
                    ay = std::get<2>(imu_data);
                    az = std::get<3>(imu_data);
                    gx = std::get<4>(imu_data);
                    gy = std::get<5>(imu_data);
                    gz = std::get<6>(imu_data);
                }
            }

            // Associate the IMU reading with the image
            imu_csv << image_count << "," << timestamp_ns << ","
                << ax << "," << ay << "," << az << ","
                << gx << "," << gy << "," << gz << std::endl;

            std::cout << "Saved pair " << image_count << " (total: " << (image_count + 1) << ")" << std::endl;
            image_count++;

            // Brief pause to avoid duplicate captures
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    running = false;
    imu_thread.join();

    // Save all buffered IMU data for continuous trajectory
    std::ofstream imu_continuous(output_dir + "/imu_continuous.csv");
    imu_continuous << "timestamp_ns,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z" << std::endl;

    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        for (const auto& data : imu_buffer) 
        {
            imu_continuous << std::get<0>(data) << "," << std::get<1>(data) << "," << std::get<2>(data) << ","
                << std::get<3>(data) << "," << std::get<4>(data) << "," << std::get<5>(data) << ","
                << std::get<6>(data) << std::endl;
        }
    }

    imu_csv.close();
    imu_continuous.close();
    cap.release();
    cv::destroyAllWindows();

    std::cout << "\n=== Collection Complete ===" << std::endl;
    std::cout << "Total image-IMU pairs collected: " << image_count << std::endl;
    std::cout << "Data saved to: " << output_dir << "/" << std::endl;
    std::cout << "  - Images: " << images_dir << "/" << std::endl;
    std::cout << "  - IMU data at image times: imu_data.csv" << std::endl;
    std::cout << "  - Continuous IMU data: imu_continuous.csv" << std::endl;

    return 0;
}