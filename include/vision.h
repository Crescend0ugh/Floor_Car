/*
	vision.h

	Opens and captures images from the robot's camera.
	Runs the YOLOv11 and ncnn object inference model on camera captures.
	Stores the camera's intrinsic matrix and LiDAR sensor's extrinsic matrix with respect to the camera.
	Provides tools for identifying which LiDAR point cloud points fall within a 2D bounding box object detection.

	Object detection is run asynchronously due to long processing times.
*/

#pragma once

#include "async_processor.h"
#include "yolo_model.h"
#include "vector.h"
#include "network_data.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/eigen.hpp>

#include <asio.hpp>

#include <optional>
#include <thread>
#include <functional>
#include <mutex>
#include <atomic>

namespace robo
{
	struct detection_obb
	{
		Eigen::Vector4f centroid; // Center of mass of point cluster

		pcl::PointXYZ center; // In world coordinates
		Eigen::Matrix3f rotation_matrix; // From world to OBB

		// These are of the underlying AABB
		pcl::PointXYZ min_point;
		pcl::PointXYZ max_point;

		int label = 0;
		float confidence = 0.0f;
	};

	// Result type for vision processing
	struct vision_result
	{
		std::vector<yolo::detection> detections;
		cv::Mat frame;
		std::chrono::milliseconds processing_time;
		bool success = false;
	};

	class vision : public async_processor<vision_result>
	{
	private:
		yolo::yolo_model yolo;

		cv::Mat camera_matrix; // Camera's intrinsic matrix
		cv::Mat dist_coeffs; // Camera's distortion coefficients
		cv::Mat lidar_to_camera_transform; // Obtained from extrinsic calibration (4x4)
		cv::Mat camera_to_lidar_transform; // Inverse of lidar_to_camera_transform

		cv::VideoCapture capture;

		bool is_intrinsics_loaded = false;
		bool is_extrinsics_loaded = false;

		// Thread safety
		mutable std::mutex frame_mutex;
		mutable std::mutex detection_mutex;

		void load_camera_calibration_info();

		// Base class implementations
		vision_result process_impl() override;
		bool on_init() override;
		void on_shutdown() override;

	public:
		bool is_client_connected = false;
		bool is_enabled = true;

		cv::Mat camera_frame; // Raw camera frame
		cv::Mat undistorted_camera_frame; // Camera frame after undistorting based on intrinsics and distortion coefficients

		std::vector<yolo::detection> detections; // Can be accessed at anytime, anywhere. Contains 2D detection data.

		cv::Size image_size = cv::Size{ 640, 480 };

		vision();

		bool initialize_camera();

		std::vector<detection_obb> estimate_detection_3d_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud) const;

		bool grab_frame();
		const cv::Mat& capture_frame();

		void clear_detections();

		// Synchronous detection
		void detect_from_camera();

		// Thread-safe accessors
		std::vector<yolo::detection> get_detections() const;
		cv::Mat get_latest_frame() const;

		// Network serialization
		robo::network::camera_frame serialize_detection_results(const vision_result& results) const;

		// Makeshift
		float compute_delta_yaw_to_bbox_center(const cv::Rect& bbox) const;
	};
}
