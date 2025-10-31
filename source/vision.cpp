#include "vision.h"

#include <cstdlib>
#include <filesystem>
#include <cmath>

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

    cv::FileStorage intrinsics((camera_matrices_path / "intrinsics.yml").string(), cv::FileStorage::READ);
    if (intrinsics.isOpened())
    {
        intrinsics["K"] >> camera_mat;
        intrinsics["D"] >> dist_coeffs;
        intrinsics["image_size"] >> image_size;
    }
    else
    {
        std::cerr << "Could not open intrinsics.yml to get the camera's intrinsic matrix" << std::endl;
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

void vision::estimate_detection_3d_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr front_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Discard points that have an x < 0.0 and are thus behind the camera
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(lidar_point_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, 100.0);
        pass.filter(*front_point_cloud);
    }

    cv::Mat camera_to_lidar_transform = lidar_to_camera_transform.inv();

    float near_plane_distance = 0.1;

    for (int detection_id = 0; detection_id < detections.size(); ++detection_id)
    {
        // Convert OpenCV matrix to Eigen matrix (OpenCV uses row major)
        Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> camera_pose(camera_to_lidar_transform.ptr<float>());

        cv::Rect bounds = detections[detection_id].rect;

        cv::Point2f center = (bounds.br() + bounds.tl()) * 0.5;

        // Translate camera origin so that it projects onto the center of the bbox
        // +x axis is right, +y axis is down
        camera_pose(0, 3) += (center.x - image_size.width);
        camera_pose(1, 3) += (center.y - image_size.height);

        // FOV = atan( dimension / (2 * focal length) )
        float fov_x = std::atan(bounds.width / (2 * camera_mat.at<float>(0, 0)));
        float fov_y = std::atan(bounds.height / (2 * camera_mat.at<float>(1, 1)));

        pcl::FrustumCulling<pcl::PointXYZ> frustum_culling;
        frustum_culling.setInputCloud(lidar_point_cloud);
        frustum_culling.setVerticalFOV(fov_y);
        frustum_culling.setHorizontalFOV(fov_x);
        frustum_culling.setNearPlaneDistance(near_plane_distance); // Arbitrary
        frustum_culling.setFarPlaneDistance(12.0); // Arbitrary

        frustum_culling.setCameraPose(camera_pose);

        pcl::PointCloud<pcl::PointXYZ> filtered_point_cloud;
        frustum_culling.filter(filtered_point_cloud);

        /*
        for (int i = 0; i < corresponding_image_points.size(); ++i)
        {
            if (!bounds.contains(corresponding_image_points[i]))
            {
                continue;
            }

            lidar_points_in_bboxes[i].push_back(lidar_point_cloud[i]);
        }
        */
    }

#if 0
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
#endif
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