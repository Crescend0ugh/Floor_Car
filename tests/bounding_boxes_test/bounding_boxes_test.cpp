#include "yolo_model.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <filesystem>
#include <limits>

const std::filesystem::path data_path = std::filesystem::path(std::string(__FILE__))
	.parent_path().parent_path().parent_path().append("tests").append("bounding_boxes_test").append("data");

const float near_plane_distance = 0.1f;

void log_point_cloud_size(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
	std::cout << "[POINT CLOUD SIZE]: " << point_cloud->points.size() << std::endl;
}

struct detection_obb
{
    pcl::PointXYZ center; // In world coordinates
    Eigen::Matrix3f rotation_matrix; // From world to OBB

    // These are of the underlying AABB
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
};

detection_obb estimate_OBB(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    detection_obb obb;

    feature_extractor.getOBB(obb.min_point, obb.max_point, obb.center, obb.rotation_matrix);

    return obb;
}

int main()
{
	cv::FileStorage intrinsics_file((data_path / "intrinsics.yml").string(), cv::FileStorage::READ);

    cv::Size2f image_size;
	cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
	intrinsics_file["K"] >> camera_matrix;
    intrinsics_file["D"] >> dist_coeffs;
    intrinsics_file["image_size"] >> image_size;

	cv::FileStorage extrinsics_file((data_path / "extrinsics.yml").string(), cv::FileStorage::READ);

	cv::Mat lidar_to_camera_transform;
    extrinsics_file["T"] >> lidar_to_camera_transform;
    cv::Mat camera_to_lidar_transform = lidar_to_camera_transform.inv();

	cv::Mat input_matrix = cv::imread((data_path / "cars.jpg").string(), 1);
    cv::Mat undistorted_matrix;
    cv::undistort(input_matrix, undistorted_matrix, camera_matrix, dist_coeffs, camera_matrix);

	yolo_model yolo;
	std::vector<detection> detections;
	yolo.detect(undistorted_matrix, detections);
	cv::Mat annotated =  annotate_detections(undistorted_matrix, detections);

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>((data_path / "point_cloud.pcd").string(), *point_cloud);

	// Begin algorithm

	pcl::PointCloud<pcl::PointXYZ>::Ptr front_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Discard points that have a x < 0.0 and are thus behind the camera
	{
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(point_cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(0.0, std::numeric_limits<float>::max());
		pass.filter(*front_point_cloud);
	}

	log_point_cloud_size(front_point_cloud);

    for (int detection_id = 0; detection_id < detections.size(); ++detection_id)
    {
        // Convert OpenCV matrix to Eigen matrix
        Eigen::Matrix4f camera_pose;
        cv::cv2eigen(camera_to_lidar_transform, camera_pose);

        // Frustum culling coordinate axes are different
        Eigen::Matrix4f cam2robot;
        cam2robot << 0, 0, 1, 0,
                0, -1, 0, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;

        Eigen::Matrix4f converted_pose = camera_pose * cam2robot;

        cv::Rect bounds = detections[detection_id].rect;
        cv::Point2f center = (bounds.tl() + bounds.br()) * 0.5;

        // Project all points and check against the 2D bounding box
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
            if (w_2d <= near_plane_distance)
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
                cv::circle(annotated, image_space_point, 1, cv::Scalar(0, 0, 255));
            }
        }

        log_point_cloud_size(filtered_point_cloud);

        std::cout << "Detection " << detection_id 
            << " of class " << get_detection_class_name(detections[detection_id].label) 
            << " with probability " << detections[detection_id].prob << std::endl;

        // Cluster the points
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(filtered_point_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(1); // 1m
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(15000); // Doesn't matter
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered_point_cloud);
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
            cluster_cloud->push_back((*filtered_point_cloud)[point_index]);
        }
        cluster_cloud->width = cluster_cloud->size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        auto obb = estimate_OBB(cluster_cloud);
        std::cout << "OBB Center: " <<  obb.center << std::endl;
    }

	cv::imshow("image", annotated);
	cv::waitKey(0);

	return 0;
}