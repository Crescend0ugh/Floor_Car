/*
	vision.h

	Opens and captures images from the robot's camera.
	Runs the YOLOv11 and ncnn object inference model on camera captures.
	Stores the camera's intrinsic matrix and LiDAR sensor's extrinsic matrix with respect to the camera.
	Provides tools for identifying which LiDAR point cloud points fall within a 2D bounding box object detection.

	Object detection is run asynchronously due to long processing times.
*/

#pragma once

#include "yolo_model.h"
#include "vector.h"
#include "network_data.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/eigen.hpp>

#include <optional>

namespace robo
{
	struct detection_obb
	{
		pcl::PointXYZ center; // In world coordinates
		Eigen::Matrix3f rotation_matrix; // From world to OBB

		// These are of the underlying AABB
		pcl::PointXYZ min_point;
		pcl::PointXYZ max_point;

		int label;
		float confidence;
	};

	class vision
	{
	private:
		yolo::yolo_model yolo;

		cv::Mat camera_matrix; // Camera's intrinsic matrix
		cv::Mat dist_coeffs; // Camera's distortion coefficients
		cv::Mat lidar_to_camera_transform; // Obtained from calibration (4x4)
		cv::Mat camera_to_lidar_transform; // Inverse of lidar_to_camera_transform

		cv::VideoCapture capture;
		cv::Mat camera_frame; // Raw camera frame
		cv::Mat undistorted_camera_frame; // Camera frame after undistorting based on intrinsics and distortion coefficients

		cv::Size image_size = cv::Size{ 640, 480 };

		std::chrono::milliseconds yolo_processing_time; // For sending to client

		bool is_intrinsics_loaded = false;
		bool is_extrinsics_loaded = false;

		void load_camera_calibration_info();

	public:
		bool is_client_connected = false;
		bool is_enabled = true;

		std::vector<yolo::detection> detections; // Can be accessed at anytime, anywhere. Contains 2D detection data.

		vision();
		bool initialize_camera();

		std::vector<detection_obb> estimate_detection_3d_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud) const;

		bool grab_frame();
		void detect_from_camera();

		robo::network::camera_frame serialize_detection_results();

		// Makeshift
		float compute_delta_yaw_to_detection_center(const yolo::detection& detection) const;
	};
}
