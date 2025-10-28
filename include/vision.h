#pragma once

#include "yolo_model.h"
#include "vector.h"

#include <optional>

// If a client is connected, annotated_image will contain the annotated camera frame
struct detection_results
{
	uint8_t camera_id = 0;
	std::vector<detection>* detections = nullptr;
	std::optional<cv::Mat> annotated_image = std::nullopt;
	std::optional<std::chrono::milliseconds> processing_time = std::nullopt;
};


struct detection_results_3d
{
	maid::vector3d center_position; // Coordinates (the origin is the left camera!)

	// TODO
	double height; // Estimated height of the bounding box
	double width; // Estimated width of the bounding box
	double depth; // About how far the center of the object is from the left camera

	int label; // What type of object was detected
	float confidence; // Confidence rating
};

class vision
{
private:
	std::optional<cv::VideoCapture> left_camera = std::nullopt;
	cv::Mat left_camera_frame;

	std::optional<cv::VideoCapture> right_camera = std::nullopt;
	cv::Mat right_camera_frame;

	yolo_model yolo;

	// Intrinsic matrices for left and right cameras
	cv::Mat left_camera_mat;
	cv::Mat right_camera_mat;

	// Distortion coefficients for left and right cameras
	cv::Mat left_dist_coeffs;
	cv::Mat right_dist_coeffs;

	// Projection matrices for left and right cameras
	cv::Mat left_proj_mat; 
	cv::Mat right_proj_mat;
	bool calibration_info_loaded = false;

	bool load_camera_calibration_info();

public:
	bool is_client_connected = false;
	bool is_enabled = true;

	std::vector<detection>* left_camera_detections;
	std::vector<detection>* right_camera_detections;
	std::vector<detection_results_3d> detections_3d;

	vision();
	bool initialize_cameras();
	void estimate_3d_positions(
		const std::vector<cv::Vec2d>& left_image_points,
		const std::vector<cv::Vec2d>& right_image_points,
		std::vector<maid::vector3d>& results
	);

	// 0 = left camera, 1 = right camera

	bool grab_frame_from_camera(int camera_id);

	// Call AFTER grab_frame_from_camera
	detection_results detect_from_camera(int camera_id);
};