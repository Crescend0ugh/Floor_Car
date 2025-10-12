#include <fstream>
#include "opencv2/opencv.hpp"
#include "net.h"

struct detection
{
	int class_id;
	float confidence;
	cv::Rect box;
};

// Normally, YOLO uses 640x640. But since we're running on a Pi, halving each dimension is likely necessary for performance
const float input_width = 320.0;
const float input_height = 320.0;
const float score_threshold = 0.5;
const float nms_threshold = 0.5;
const float confidence_threshold = 0.5;

// List of objects we're detecting
std::vector<std::string> load_class_list()
{
	std::vector<std::string> class_list;

	std::ifstream stream("../content/models/classes.txt");

	std::string line;
	while (getline(stream, line)) {
		class_list.push_back(line);
	}

	return class_list;
}

int main()
{
	std::vector<std::string> class_list = load_class_list();

	cv::Mat image = cv::imread("../content/images/apple.jpg");

	cv::namedWindow("opencv test", cv::WINDOW_AUTOSIZE);
	cv::waitKey(0);

	return 0;
}