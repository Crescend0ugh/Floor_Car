#include "vision.h"

#include <cstdlib>

vision::vision()
{
    left_camera_detections = new std::vector<detection>();
    right_camera_detections = new std::vector<detection>();

    initialize_cameras();
}

bool vision::initialize_cameras()
{
    left_camera = cv::VideoCapture(0);
    
    if (!left_camera.value().isOpened())
    {
        std::cerr << "Warning: Could not find or open camera 0 (left)." << std::endl;
        left_camera = std::nullopt;
    }
   
    // TODO: Figure out which numbers to use for the constructor
#ifdef RPI_UBUNTU
    right_camera = cv::VideoCapture(2);
    if (!right_camera.value().isOpened())
    {
        std::cerr << "Warning: Could not find or open camera 1 (right)" << std::endl;
        right_camera = std::nullopt;
    }
#endif

    if (!left_camera && !right_camera)
    {
        std::cerr << "Warning: Could not find or open both left and right cameras. Vision will be disabled." << std::endl;
        is_enabled = false;
    }
    else
    { // We can run vision with one or two cameras active
        is_enabled = true;
    }

    return is_enabled;
}

detection_results vision::detect_from_camera(int camera_id)
{
    cv::Mat& frame = camera_id ? right_camera_frame : left_camera_frame;
    std::vector<detection>& detections = camera_id ? *right_camera_detections : *left_camera_detections;
    detections.clear();

    detection_results results;
    results.camera_id = camera_id;
    results.detections = &detections;

    std::optional<cv::VideoCapture> which_camera = camera_id ? right_camera : left_camera;

    // This camera is inactive
    if (!which_camera.has_value())
    {
        std::cerr << "Warning: Tried to capture from inactive camera (" << camera_id << ")" << std::endl;
        return results;
    }

    cv::VideoCapture& camera = which_camera.value();

    camera >> frame;
    if (frame.empty()) 
    {
        std::cerr << "Warning: Captured empty frame from camera (" << camera_id << ")" << std::endl;
        which_camera = std::nullopt; // Disable the camera because it's not working?
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