#include "vision.h"

#include <cstdlib>
#include <filesystem>

vision::vision()
{
    left_camera_detections = new std::vector<detection>();
    right_camera_detections = new std::vector<detection>();

    initialize_cameras();
}

bool vision::load_camera_calibration_info()
{
    std::string file_path{ __FILE__ };
    std::filesystem::path source_path(file_path);
    std::filesystem::path camera_matrices_path = source_path.parent_path().parent_path().parent_path().append("content").append("camera_matrices");

    if (!std::filesystem::exists(camera_matrices_path))
    {
        std::cerr << "No camera_matrices directory could be found." << std::endl;
        return false;
    }

    cv::FileStorage extrinsics_file((camera_matrices_path / "extrinsics.yml").string(), cv::FileStorage::READ);
    if (extrinsics_file.isOpened())
    {
        extrinsics_file["P1"] >> left_proj_mat;
        extrinsics_file["P2"] >> right_proj_mat;
    }
    else
    {
        std::cerr << "Could not open extrinsics.yml for stereo calibration parameters" << std::endl;
        return false;
    }

    cv::FileStorage left_intrinsics_file((camera_matrices_path / "intrinsics_0.yml").string(), cv::FileStorage::READ);
    if (left_intrinsics_file.isOpened())
    {
        left_intrinsics_file["M"] >> left_camera_mat;
        left_intrinsics_file["D"] >> left_dist_coeffs;
    }
    else
    {
        std::cerr << "Could not open intrinsics_0.yml to get the left camera's intrinsic matrix" << std::endl;
        return false;
    }

    cv::FileStorage right_intrinsics_file((camera_matrices_path / "intrinsics_2.yml").string(), cv::FileStorage::READ);
    if (right_intrinsics_file.isOpened())
    {
        right_intrinsics_file["M"] >> right_camera_mat;
        right_intrinsics_file["D"] >> right_dist_coeffs;
    }
    else
    {
        std::cerr << "Could not open intrinsics_2.yml to get the right camera's intrinsic matrix" << std::endl;
        return false;
    }

    return true;
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

#ifdef RPI_UBUNTU
    calibration_info_loaded = load_camera_calibration_info();
#endif

    return is_enabled;
}

void vision::estimate_3d_positions(
    const std::vector<cv::Vec2d>& left_image_points, 
    const std::vector<cv::Vec2d>& right_image_points,
    std::vector<maid::vector3d>& results
)
{
    if (!calibration_info_loaded)
    {
        std::cerr << "Warning: Unable to estimate 3D positions without calibration info" << std::endl;
        return;
    }

    std::vector<cv::Point2f> left_undistorted_points, right_undistorted_points;
    cv::undistortPoints(left_image_points, left_undistorted_points, left_camera_mat, left_dist_coeffs);
    cv::undistortPoints(right_image_points, right_undistorted_points, right_camera_mat, right_dist_coeffs);

    // TODO: If P1 and P2 don't work as projection matrices, compute them manually with extrinsic R and T
    cv::Mat homogeneous_points;
    cv::triangulatePoints(left_proj_mat, right_proj_mat, left_undistorted_points, right_undistorted_points, homogeneous_points);

    for (int i = 0; i < homogeneous_points.cols; ++i) {
        cv::Vec4d point = homogeneous_points.col(i);
        double x = point[0] / point[3];
        double y = point[1] / point[3];
        double z = point[2] / point[3];

        // std::cout << "3D Point " << i << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
        results.emplace_back(x, y, z);
    }
}

bool vision::grab_frame_from_camera(int camera_id)
{
    std::optional<cv::VideoCapture> which_camera = camera_id ? right_camera : left_camera;

    // This camera is inactive
    if (!which_camera.has_value())
    {
        std::cerr << "Warning: Tried to capture from inactive camera (" << camera_id << ")" << std::endl;
        return false;
    }

    return which_camera.value().grab();
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

    cv::VideoCapture& camera = which_camera.value();

    camera.retrieve(frame);

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