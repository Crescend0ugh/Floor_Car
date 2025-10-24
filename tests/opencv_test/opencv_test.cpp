#include "opencv2/opencv.hpp"

#include <vector>
#include <iostream>

#include "yolo_model.h"

static void draw_results(const cv::Mat& bgr, const std::vector<detection>& objects, std::chrono::milliseconds runtime, bool is_camera_feed)
{
	cv::Mat image = bgr.clone();

	for (size_t i = 0; i < objects.size(); i++)
	{
		const detection& obj = objects[i];

		cv::rectangle(image, obj.rect, cv::Scalar(255, 0, 0));

		std::string text = std::format("{}: {:.1f}%", get_detection_class_name(obj.label), obj.prob * 100);

		int base_line = 0;
		cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);

		int x = obj.rect.x;
		int y = obj.rect.y - label_size.height - base_line;
		if (y < 0)
			y = 0;
		if (x + label_size.width > image.cols)
			x = image.cols - label_size.width;

		cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + base_line)),
			cv::Scalar(255, 255, 255), -1);

		cv::putText(image, text, cv::Point(x, y + label_size.height),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
	}

	std::string duration_text = std::format("Took: {}", runtime);
	cv::putText(image, duration_text, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

	 cv::imshow("image", image);

	 if (!is_camera_feed) {
		 cv::waitKey(0);
	 }
}

cv::Mat run_model(yolo_model& model, cv::Mat& m)
{
	std::vector<detection> objects;

	auto start_time = std::chrono::high_resolution_clock::now();
	model.detect(m, objects);
	auto end_time = std::chrono::high_resolution_clock::now();

	auto runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

	return annotate_detections(m, objects, runtime);
}

void run_on_static_image(const char* path)
{
	const char* imagepath = "content/images/apple.jpg";

	cv::Mat m = cv::imread(imagepath, 1);

	if (m.empty())
	{
		std::cerr << "cv::imread " << imagepath << " failed" << std::endl;
		return;
	}

	yolo_model yolo;
	cv::Mat results = run_model(yolo, m);
	cv::imshow("image", results);
	cv::waitKey(0);
}

void run_on_camera_feed()
{
	// Change 0 or 1 depending on camera 
	cv::VideoCapture cap(0);

	if (!cap.isOpened()) {
		std::cerr << "Error: Could not open camera." << std::endl;
		return;
	}

	cv::Mat frame;

	yolo_model yolo;

	auto current_time = std::chrono::high_resolution_clock::now();
	auto next_capture_time = current_time + std::chrono::seconds(1);

	while (true) {
		current_time = std::chrono::high_resolution_clock::now();

		if (current_time < next_capture_time)
		{
			continue;
		}

		next_capture_time += std::chrono::seconds(1);

		cap >> frame; // Capture new frame

		if (frame.empty()) {
			std::cerr << "Error: Captured empty frame." << std::endl;
			break;
		}

		cv::Mat results = run_model(yolo, frame);
		cv::imshow("image", results);

		// Press q to quit
		if (cv::waitKey(1) == 'q') {
			break;
		}
	}

	cap.release();
	cv::destroyAllWindows();
}

int main()
{
	//run_on_static_image("content/images/apple.jpg");

	run_on_camera_feed();

	return 0;
}