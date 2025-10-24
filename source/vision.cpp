#include "vision.h"

#include <cstdlib>

vision::vision()
{
    left_camera_detections = new std::vector<detection>();

    left_camera = cv::VideoCapture(0);
    if (!left_camera.isOpened())
    {
        std::cerr << "Error: Could not find or open camera 0 (left)" << std::endl;
        exit(1);
    }

    right_camera_detections = new std::vector<detection>();

    /*
    right_camera = cv::VideoCapture(1);
    if (!right_camera.isOpened())
    {
        std::cerr << "Error: Could not find or open camera 1 (right)" << std::endl;
        exit(1);
    }
    */
}

detection_results vision::detect_from_camera(uint8_t camera_id)
{
    cv::VideoCapture& camera = camera_id ? right_camera : left_camera;
    cv::Mat& frame = camera_id ? right_camera_frame : left_camera_frame;
    std::vector<detection>& detections = camera_id ? *right_camera_detections : *left_camera_detections;
    detections.clear();

    detection_results results;
    results.camera_id = camera_id;
    results.detections = &detections;

    camera >> frame;
    if (frame.empty()) 
    {
        std::cerr << "Warning: Captured empty frame from camera " << camera_id << std::endl;
        return results;
    }

    // These timestamps are totally unneeded if running without a client
    // But their effects on performance are probably negligible
    auto start_time = std::chrono::high_resolution_clock::now();

    yolo.detect(frame, detections);

    auto end_time = std::chrono::high_resolution_clock::now();

    // Don't do this extra work if no clients are connected
    if (is_client_connected)
    {
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        cv::Mat annotated = annotate_detections(
            frame, 
            detections,
            processing_time
        );

        results.annotated_image = annotated;
        results.processing_time = processing_time;
    }

    return results;
}