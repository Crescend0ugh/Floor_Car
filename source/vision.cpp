#include "vision.h"

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

static detection_obb estimate_OBB(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    detection_obb obb;

    feature_extractor.getOBB(obb.min_point, obb.max_point, obb.center, obb.rotation_matrix);

    return obb;
}

vision::vision()
{
    initialize_camera();
}

bool vision::load_camera_calibration_info()
{
    std::string file_path{ __FILE__ };
    std::filesystem::path source_path(file_path);
    std::filesystem::path calibration_results_path = source_path.parent_path().parent_path().parent_path().append("content").append("calibration_results");

    if (!std::filesystem::exists(calibration_results_path))
    {
        std::cerr << "No 'calibration_results' directory could be found." << std::endl;
        return false;
    }

    cv::FileStorage extrinsics((calibration_results_path / "extrinsics.yml").string(), cv::FileStorage::READ);
    if (extrinsics.isOpened())
    {
        extrinsics["T"] >> lidar_to_camera_transform; // This should be column major
        camera_to_lidar_transform = lidar_to_camera_transform.inv();
        // MAKE THIS A 4x4 MATRIX IF IT ISN'T !!!
    }
    else
    {
        std::cerr << "Could not open extrinsics.yml" << std::endl;
        return false;
    }

    cv::FileStorage intrinsics((calibration_results_path / "intrinsics.yml").string(), cv::FileStorage::READ);
    if (intrinsics.isOpened())
    {
        intrinsics["K"] >> camera_matrix;
        intrinsics["D"] >> dist_coeffs;
        intrinsics["image_size"] >> image_size;

        // Apply distortion coefficients
        cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 1.0);
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

std::vector<detection_obb> vision::estimate_detection_3d_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr front_point_cloud = cull_points_behind_camera(lidar_point_cloud);
    std::vector<detection_obb> obbs;

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
        obbs.push_back(estimate_OBB(cluster_cloud));
    }

    return obbs;
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