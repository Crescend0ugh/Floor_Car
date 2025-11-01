#pragma once

#include "yolo_model.h"
#include "vector.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/eigen.hpp>

#include <optional>

// If a client is connected, annotated_image will contain the annotated camera frame
struct detection_results
{
	uint8_t camera_id = 0;
	std::vector<detection>* detections = nullptr;
	std::optional<cv::Mat> annotated_image = std::nullopt;
	std::optional<std::chrono::milliseconds> processing_time = std::nullopt;
};

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
	yolo_model yolo;

	cv::Mat camera_matrix; // Camera's intrinsic matrix
	cv::Mat dist_coeffs; // Camera's distortion coefficients
	cv::Mat lidar_to_camera_transform; // Obtained from calibration (4x4)
	cv::Mat camera_to_lidar_transform;

	cv::VideoCapture capture;
	cv::Mat camera_frame;

	cv::Size image_size = cv::Size{ 640, 480 };

	bool calibration_info_loaded = false;

	bool load_camera_calibration_info();

public:
	bool is_client_connected = false;
	bool is_enabled = true;

	std::vector<detection> detections;

	vision();
	bool initialize_camera();

	std::vector<detection_obb> estimate_detection_3d_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud);

	bool grab_frame();

	// Call AFTER grab_frame_from_camera
	detection_results detect_from_camera();
};