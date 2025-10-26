#pragma once

#include "yolo_model.h"

#include <optional>

// If a client is connected, annotated_image will contain the annotated camera frame
struct detection_results
{
	uint8_t camera_id;
	std::vector<detection>* detections = nullptr;
	std::optional<cv::Mat> annotated_image = std::nullopt;
	std::optional<std::chrono::milliseconds> processing_time = std::nullopt;
};

class vision
{
private:
	std::optional<cv::VideoCapture> left_camera = std::nullopt;
	cv::Mat left_camera_frame;

	std::optional<cv::VideoCapture> right_camera = std::nullopt;
	cv::Mat right_camera_frame;

	yolo_model yolo;

public:
	bool is_client_connected = false;
	bool is_enabled = true;
	std::vector<detection>* left_camera_detections;
	std::vector<detection>* right_camera_detections;

	vision();
	bool initialize_cameras();

	// 0 = left camera, 1 = right camera
	detection_results detect_from_camera(int camera_id);
};