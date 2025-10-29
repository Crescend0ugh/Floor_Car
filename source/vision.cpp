#include "vision.h"

#include <cstdlib>
#include <filesystem>

vision::vision()
{
    initialize_camera();
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

    // TODO: Don't know if these will be in YML
    cv::FileStorage extrinsics((camera_matrices_path / "extrinsics.yml").string(), cv::FileStorage::READ);
    if (extrinsics.isOpened())
    {
        // TODO
        // extrinsics["RT"] >> lidar_to_camera_transform;
        // MAKE THIS A 4x4 MATRIX IF IT ISN'T !!!
    }
    else
    {
        std::cerr << "Could not open extrinsics.yml" << std::endl;
        return false;
    }

    cv::FileStorage intrinsics((camera_matrices_path / "intrinsics_0.yml").string(), cv::FileStorage::READ);
    if (intrinsics.isOpened())
    {
        intrinsics["M"] >> camera_mat;
        intrinsics["D"] >> dist_coeffs;
    }
    else
    {
        std::cerr << "Could not open intrinsics_0.yml to get the left camera's intrinsic matrix" << std::endl;
        return false;
    }

    return true;
}

bool vision::initialize_camera()
{
    capture = cv::VideoCapture(0);
    
    if (!capture.isOpened())
    {
        std::cerr << "Warning: Could not find or open camera 0." << std::endl;
        is_enabled = false;
    }

#ifdef RPI_UBUNTU
    calibration_info_loaded = load_camera_calibration_info();

    if (!calibration_info_loaded)
    {
        std::cerr << "Warning: Unable to load necessary calibration data." << std::endl;
        is_enabled = false;
    }
#endif

    return is_enabled;
}

void vision::estimate_detection_3d_bounds(const std::vector<cv::Vec3d>& lidar_point_cloud)
{
    std::vector<cv::Point2d> corresponding_image_points;
    corresponding_image_points.resize(lidar_point_cloud.size());

    // Create mapping from 3D lidar points to 2D image points
    for (int i = 0; i < lidar_point_cloud.size(); ++i)
    {
        auto& point = lidar_point_cloud[i];

        // Convert to 4D row vector
        cv::Mat homogeneous_point = (cv::Mat_<double>(4, 1) << point[0], point[1], point[2], 1.0);

        // Transform world (lidar) space to camera space
        cv::Mat camera_space_homogeneous_point = homogeneous_point * lidar_to_camera_transform;

        // Perspective divide into 3D
        double w = camera_space_homogeneous_point.at<double>(3, 0);

        cv::Mat camera_space_point = (cv::Mat_<double>(3, 1) << 
            camera_space_homogeneous_point.at<double>(0, 0) / w, 
            camera_space_homogeneous_point.at<double>(1, 0) / w,
            camera_space_homogeneous_point.at<double>(2, 0) / w
        );

        // Camera space to image plane
        cv::Mat image_space_homogeneous_point = camera_space_point * camera_mat;

        // Perspective divide into 2D
        w = image_space_homogeneous_point.at<double>(2, 0);
        
        cv::Mat image_space_point = (cv::Mat_<double>(2, 1) <<
            image_space_homogeneous_point.at<double>(0, 0) / w,
            image_space_homogeneous_point.at<double>(1, 0) / w
        );

        corresponding_image_points[i] = image_space_point.reshape(1, 1).at<cv::Point2d>(0, 0);
    }

    std::vector<std::vector<cv::Vec3d>> lidar_points_in_bboxes;
    lidar_points_in_bboxes.resize(detections.size());

    for (int detection_id = 0; detection_id < detections.size(); ++detection_id)
    {
        cv::Rect bounds = detections[detection_id].rect;

        for (int i = 0; i < corresponding_image_points.size(); ++i)
        {
            if (!bounds.contains(corresponding_image_points[i]))
            {
                continue;
            }

            lidar_points_in_bboxes[i].push_back(lidar_point_cloud[i]);
        }
    }

    for (int i = 0; i < lidar_points_in_bboxes.size(); ++i)
    {

    }
}

bool vision::grab_frame()
{
    if (!is_enabled)
    {
        std::cerr << "Warning: Tried to grab a frame while camera is disabled." << std::endl;
        return false;
    }

    return capture.grab();
}

detection_results vision::detect_from_camera()
{
    detections.clear();

    detection_results results;
    results.camera_id = 0;
    results.detections = &detections;

    capture.retrieve(camera_frame);

    if (camera_frame.empty())
    {
        std::cerr << "Warning: Captured empty frame from camera" << std::endl;
        return results;
    }

    // These timestamps are totally unneeded if running without a client
    // But their effects on performance are probably negligible
    auto start_time = std::chrono::high_resolution_clock::now();
    yolo.detect(camera_frame, detections);
    auto end_time = std::chrono::high_resolution_clock::now();

    // Don't do this extra work if no clients are connected
    if (is_client_connected)
    {
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        cv::Mat annotated = annotate_detections(
            camera_frame,
            detections,
            processing_time
        );

        results.annotated_image = annotated;
        results.processing_time = processing_time;
    }

    return results;
}