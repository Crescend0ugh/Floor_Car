/*
    Collect LiDAR-Camera calibration data
    Captures synchronized camera images and LiDAR point clouds with checkerboard
    https://www.mathworks.com/help/lidar/ug/lidar-and-camera-calibration.html
*/

// TODO: Include LiDAR driver

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sys/stat.h>
#include <csignal>

volatile bool running = true;

std::string output_dir = "lidar_camera_calib_data";
std::string images_dir = output_dir + "/images";
std::string pcd_dir = output_dir + "/pointclouds";

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

static void save_pcd()
{

}

int main()
{
    std::cout << "=== LiDAR-Camera Calibration Data Collection ===" << std::endl;
    std::cout << "\nInstructions:" << std::endl;
    std::cout << "1. Place a large checkerboard visible to BOTH camera and LiDAR" << std::endl;
    std::cout << "2. Checkerboard should be:" << std::endl;
    std::cout << "   - NOT parallel to LiDAR scanning plane" << std::endl;
    std::cout << "   - Pointed toward camera (z-axis) and LiDAR (x-axis)" << std::endl;
    std::cout << "   - At various distances and angles" << std::endl;
    std::cout << "3. Capture 20-30 synchronized pairs" << std::endl;
    std::cout << "4. Avoid motion blur - keep setup still when capturing" << std::endl;
    std::cout << "5. Press 's' to save a pair" << std::endl;
    std::cout << "6. Press 'q' to quit" << std::endl;
    std::cout << "\nStarting in 3 seconds...\n" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Register signal handler
    signal(SIGINT, signal_handler);

    // Create output directories
    create_directory(output_dir);
    create_directory(images_dir);
    create_directory(pcd_dir);;

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) 
    {
        std::cerr << "Error: Could not open camera!" << std::endl;
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    uint32_t pair_count = 0;
    cv::Mat frame;

    // Create metadata file
    std::ofstream metadata(output_dir + "/metadata.txt");
    metadata << "# LiDAR-Camera Calibration Data\n";
    metadata << "# Generated: " << std::chrono::system_clock::now().time_since_epoch().count() << "\n";
    metadata << "# Format: pair_id,timestamp_ns,image_file,pcd_file\n";
    metadata.close();

    while (running)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Error: Could not read frame from camera!" << std::endl;
            break;
        }

        int key = cv::waitKey(30);

        if (key == 'q' || key == 27) // 'q' or ESC
        {
            running = false;
            break;
        }
        else if (key == 's' || key == 'S')
        {
            // Get timestamp
            auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();

            // Save image
            std::string image_filename = "image_" + std::to_string(pair_count) + ".jpg";
            std::string image_path = images_dir + "/" + image_filename;
            cv::imwrite(image_path, frame);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    cap.release();
    cv::destroyAllWindows();

    std::cout << "\n=== Collection Complete ===" << std::endl;
    std::cout << "Total image-LiDAR pairs collected: " << pair_count << std::endl;
    std::cout << "Data saved to: " << output_dir << "/" << std::endl;
    std::cout << "  - Images: " << images_dir << "/" << std::endl;
    std::cout << "  - Point clouds: " << pcd_dir << "/" << std::endl;
    std::cout << "  - Metadata: metadata.txt" << std::endl;

    return 0;
}