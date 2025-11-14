#pragma once

#include "yolo_model.h"
#include "network_data.h"

#include <opencv2/opencv.hpp>

namespace robo
{
	struct tracking_result
	{
		bool is_located = false;
		cv::Rect bbox;
		float confidence = 0.0f;
		int32_t id;
		int32_t class_id;
	};

	struct single_tracker
	{
		cv::Ptr<cv::TrackerVit> model;
		int32_t id;
		int32_t class_id;
	};

	class object_tracker
	{
	private:
		cv::TrackerVit::Params params;
		std::vector<single_tracker> trackers;
		int32_t next_id = 0;

	public:
		object_tracker();

		int32_t init(const cv::Mat& image, const yolo::detection& detection);

		// Returns list of tracking results and true if none of the trackers could find their assigned object
		std::pair<std::vector<robo::tracking_result>, bool> infer(const cv::Mat& image);
		void clear_trackers();
		bool is_empty() const;

		static cv::Mat annotate_tracker_results(
			const cv::Mat& bgr,
			const std::vector<robo::tracking_result>& results, 
			std::chrono::milliseconds processing_time = -1ms
		);

		robo::network::camera_frame serialize_tracker_results(
			const cv::Mat& bgr,
			const std::vector<robo::tracking_result>& results
		) const;
	};
}