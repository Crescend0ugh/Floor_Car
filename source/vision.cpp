#include "vision.h"

#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <cstdlib>
#include <filesystem>
#include <cmath>

static const float near_plane_distance_m = 0.1f;
static const float euclidean_clustering_tolerance_m = 1.0f;
static const int euclidean_clustering_min_cluster_size = 10;

static pcl::PointCloud<pcl::PointXYZ>::Ptr cull_points_behind_camera(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr front_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Discard points that have a x < 0.0 and are thus behind the camera
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, std::numeric_limits<float>::max());
    pass.filter(*front_point_cloud);

    return front_point_cloud;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr get_biggest_cluster(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(euclidean_clustering_tolerance_m);
    ec.setMinClusterSize(euclidean_clustering_min_cluster_size);
    ec.setMaxClusterSize(15000); // Doesn't matter
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Find the biggest cluster
    int most_indices = -1;
    int biggest_cluster_index = 0;
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        const auto& indices = cluster_indices[i];
        if (indices.indices.size() > most_indices)
        {
            most_indices = indices.indices.size();
            biggest_cluster_index = i;
        }
    }

    // Copy over the points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point_index : cluster_indices[biggest_cluster_index].indices)
    {
        cluster_cloud->push_back((*cloud)[point_index]);
    }
    cluster_cloud->width = cluster_cloud->size();
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = true;

    return cluster_cloud;
}

static robo::detection_obb estimate_OBB(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const yolo::detection& detection)
{
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    robo::detection_obb obb;

    feature_extractor.getOBB(obb.min_point, obb.max_point, obb.center, obb.rotation_matrix);
    obb.confidence = detection.prob;
    obb.label = detection.label;

    pcl::compute3DCentroid(*cloud, obb.centroid);

    return obb;
}

robo::vision::vision()
{
    initialize_camera();
}

void robo::vision::load_camera_calibration_info()
{
    std::filesystem::path source_path(std::string(__FILE__));

    std::filesystem::path calibration_results_path = source_path.parent_path().parent_path()
        .append("content").append("calibration_results");

    if (!std::filesystem::exists(calibration_results_path))
    {
        std::cerr << "No 'calibration_results' directory could be found." << std::endl;

        is_extrinsics_loaded = false;
        is_intrinsics_loaded = false;

        return;
    }

    cv::FileStorage extrinsics((calibration_results_path / "extrinsics.yml").string(), cv::FileStorage::READ);
    if (extrinsics.isOpened())
    {
        extrinsics["T"] >> lidar_to_camera_transform; // This should be column major
        camera_to_lidar_transform = lidar_to_camera_transform.inv();
        // MAKE THIS A 4x4 MATRIX IF IT ISN'T !!!

        is_extrinsics_loaded = true;
    }
    else
    {
        std::cerr << "Could not open extrinsics.yml" << std::endl;
        is_extrinsics_loaded = false;
    }

    cv::FileStorage intrinsics((calibration_results_path / "intrinsics.yml").string(), cv::FileStorage::READ);
    if (intrinsics.isOpened())
    {
        intrinsics["K"] >> camera_matrix;
        intrinsics["D"] >> dist_coeffs;
        intrinsics["image_size"] >> image_size;

        // Apply distortion coefficients
        cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 1.0);

        is_intrinsics_loaded = true;
    }
    else
    {
        std::cerr << "Could not open intrinsics.yml to get the camera's intrinsic matrix" << std::endl;
        is_intrinsics_loaded = false;
    }
}

bool robo::vision::initialize_camera()
{
    capture = cv::VideoCapture(0);
    
    if (!capture.isOpened())
    {
        std::cerr << "Warning: Could not find or open camera 0." << std::endl;
        is_enabled = false;
    }

    load_camera_calibration_info();

    if (!is_intrinsics_loaded && !is_extrinsics_loaded)
    {
        std::cerr << "Warning: Unable to load necessary calibration data." << std::endl;
        is_enabled = false;
    }

    return is_enabled;
}

std::vector<robo::detection_obb> robo::vision::estimate_detection_3d_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr front_point_cloud = cull_points_behind_camera(lidar_point_cloud);
    std::vector<detection_obb> obbs;

    if (!is_intrinsics_loaded || !is_extrinsics_loaded)
    {
        return obbs;
    }

    for (int detection_id = 0; detection_id < detections.size(); ++detection_id)
    {
        cv::Rect bounds = detections[detection_id].rect;
        cv::Point2f center = (bounds.br() + bounds.tl()) * 0.5;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& point : front_point_cloud->points)
        {
            cv::Mat homogeneous_point = (cv::Mat_<float>(4, 1) << point.x, point.y, point.z, 1.0f);
            cv::Mat camera_space_homogeneous_point = lidar_to_camera_transform * homogeneous_point;

            // Perform 3D perspective divide
            float w_3d = camera_space_homogeneous_point.at<float>(3, 0);
            if (w_3d == 0)
            {
                continue;
            }

            cv::Mat camera_space_point = (cv::Mat_<float>(3, 1) <<
                camera_space_homogeneous_point.at<float>(0, 0) / w_3d,
                camera_space_homogeneous_point.at<float>(1, 0) / w_3d,
                camera_space_homogeneous_point.at<float>(2, 0) / w_3d
            );

            // Camera space to image plane
            cv::Mat image_space_homogeneous_point = camera_matrix * camera_space_point;

            // Perspective divide to get 2D image coordinates
            float w_2d = image_space_homogeneous_point.at<float>(2, 0);

            // Near plane clipping
            if (w_2d <= near_plane_distance_m)
            {
                continue;
            }

            // Calculate final (u, v) pixel coordinates
            cv::Point2f image_space_point(
                image_space_homogeneous_point.at<float>(0, 0) / w_2d,
                image_space_homogeneous_point.at<float>(1, 0) / w_2d
            );

            // If the point is inside the bounding box, add it to the cloud
            if (bounds.contains(image_space_point))
            {
                filtered_point_cloud->points.push_back(point);
                // Draw point
                //cv::circle(annotated, image_space_point, 1, cv::Scalar(0, 0, 255));
            }
        }

        auto cluster_cloud = get_biggest_cluster(filtered_point_cloud);
        obbs.push_back(estimate_OBB(cluster_cloud, detections[detection_id]));
    }

    return obbs;
}

bool robo::vision::grab_frame()
{
    if (!is_enabled)
    {
        std::cerr << "Warning: Tried to grab a frame while camera is disabled." << std::endl;
        return false;
    }

    return capture.grab();
}

void robo::vision::detect_from_camera()
{
    detections.clear();

    capture.retrieve(camera_frame);

    if (camera_frame.empty())
    {
        std::cerr << "Warning: Captured empty frame from camera" << std::endl;
        return;
    }

    cv::Mat& final_image = camera_frame;

    // Undistort the capture if we loaded intrinsics and run object detection on the undistorted result instead
    if (is_intrinsics_loaded)
    {
        undistorted_camera_frame = cv::Mat();
        cv::undistort(camera_frame, undistorted_camera_frame, camera_matrix, dist_coeffs);
        final_image = undistorted_camera_frame;
    }

    // Get processing time for the client
    auto start_time = std::chrono::high_resolution_clock::now();
    yolo.detect(final_image, detections);
    auto end_time = std::chrono::high_resolution_clock::now();

    yolo_processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
}

robo::network::camera_frame robo::vision::serialize_detection_results()
{
    cv::Mat image = yolo::annotate_detections(
        (is_intrinsics_loaded) ? undistorted_camera_frame : camera_frame,
        detections,
        yolo_processing_time
    );

    std::vector<uchar> pixels;

    if (image.isContinuous()) // If the matrix is continuous (no padding between rows), copy all data at once
    {
        pixels.assign(image.data, image.data + image.total() * image.elemSize());
    }
    else  // If the matrix is not continuous, copy row by row
    {
        for (int i = 0; i < image.rows; ++i)
        {
            pixels.insert(pixels.end(), image.ptr<uchar>(i), image.ptr<uchar>(i) + image.cols * image.elemSize());
        }
    }

    return robo::network::camera_frame
    {
        .camera_id = 0,
        .frame_height = image.rows,
        .frame_width = image.cols,
        .bgr_pixels = pixels,
        .processing_time = static_cast<uint16_t>(yolo_processing_time.count()),
    };
}

float robo::vision::compute_delta_yaw_to_detection_center(const yolo::detection& detection) const
{
    if (!is_intrinsics_loaded)
    {
        return 0.0f;
    }

    float detection_center_x = ((detection.rect.br() + detection.rect.tl()) / 2).x;

    // Subtract by principal point x-coordinate (pixels)
    double delta_x = detection_center_x - camera_matrix.at<double>(0, 2);

    // Get the angle, given x focal length
    double angle_rad = std::atan(delta_x / camera_matrix.at<double>(0, 0));

    return angle_rad * (180.0f / M_PI);
}