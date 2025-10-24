#pragma once

#include "layer.h"
#include "net.h"

#include "opencv2/opencv.hpp"

#include <vector>

struct detection
{
	cv::Rect_<float> rect;
	int label;
	float prob;
};

class yolo_model 
{
private:
	ncnn::Net model;
	const int target_size;
	const float prob_threshold;
	const float nms_threshold;

	static float softmax(const float* src, float* dst, int length);

	void non_max_suppression(
		std::vector<detection>& proposals,
		std::vector<detection>& results,
		int orin_h,
		int orin_w,
		float dh = 0,
		float dw = 0,
		float ratio_h = 1.0f,
		float ratio_w = 1.0f
	);

	void generate_proposals(
		int stride,
		const ncnn::Mat& feat_blob,
		std::vector<detection>& objects
	);

public:
	yolo_model();
	void detect(const cv::Mat& bgr, std::vector<detection>& detections);
};

const char* get_detection_class_name(int id);

cv::Mat annotate_detections(const cv::Mat& bgr, const std::vector<detection>& detections, std::chrono::milliseconds processing_time);