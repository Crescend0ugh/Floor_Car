#include <fstream>
#include <chrono>

#include "opencv2/opencv.hpp"
#include "net.h"

struct detection
{
	int class_id;
	float confidence;
	cv::Rect rectangle;
};

// Normally, YOLO uses 640x640. But since we're running on a Pi, halving each dimension is likely necessary for performance
const float target_size = 320.0;
const float score_threshold = 0.5;
const float nms_threshold = 0.4;
const float confidence_threshold = 0.25;

class object_detector
{
private:
	ncnn::Net model;
	static void quick_sort_descent_inplace(std::vector<detection>& detections, int left, int right);
	static void quick_sort_descent_inplace(std::vector<detection>& detections);
	static inline float intersection_area(const detection& a, const detection& b);
	static void nms_sorted_bboxes(const std::vector<detection>& detections, std::vector<int>& picked, float nms_threshold, bool agnostic = false);

public:
	object_detector();
	~object_detector();
	void detect(const cv::Mat& input, std::vector<detection>& detection);
};

void object_detector::quick_sort_descent_inplace(std::vector<detection>& detections, int left, int right)
{
	int i = left;
	int j = right;
	float p = detections[(left + right) / 2].confidence;

	while (i <= j)
	{
		while (detections[i].confidence > p)
			i++;

		while (detections[j].confidence < p)
			j--;

		if (i <= j)
		{
			// swap
			std::swap(detections[i], detections[j]);

			i++;
			j--;
		}
	}

#pragma omp parallel sections
	{
#pragma omp section
		{
			if (left < j) quick_sort_descent_inplace(detections, left, j);
		}
#pragma omp section
		{
			if (i < right) quick_sort_descent_inplace(detections, i, right);
		}
	}
}

void object_detector::quick_sort_descent_inplace(std::vector<detection>& detections)
{
	if (detections.empty())
		return;

	quick_sort_descent_inplace(detections, 0, detections.size() - 1);
}

inline float object_detector::intersection_area(const detection& a, const detection& b)
{
	cv::Rect_<float> inter = a.rectangle & b.rectangle;
	return inter.area();
}

void object_detector::nms_sorted_bboxes(const std::vector<detection>& detections, std::vector<int>& picked, float nms_threshold, bool agnostic)
{
	picked.clear();

	const int n = detections.size();

	std::vector<float> areas(n);
	for (int i = 0; i < n; i++)
	{
		areas[i] = detections[i].rectangle.area();
	}

	for (int i = 0; i < n; i++)
	{
		const detection& a = detections[i];

		int keep = 1;
		for (int j = 0; j < (int)picked.size(); j++)
		{
			const detection& b = detections[picked[j]];

			if (!agnostic && a.class_id != b.class_id)
				continue;

			// intersection over union
			float inter_area = intersection_area(a, b);
			float union_area = areas[i] + areas[picked[j]] - inter_area;
			// float IoU = inter_area / union_area
			if (inter_area / union_area > nms_threshold)
				keep = 0;
		}

		if (keep)
			picked.push_back(i);
	}
}

object_detector::object_detector()
{
	model.load_param("content/models/yolo11n_ncnn_model/model.ncnn.param");
	model.load_model("content/models/yolo11n_ncnn_model/model.ncnn.bin");
}

object_detector::~object_detector()
{
	model.clear();
}

void object_detector::detect(const cv::Mat& input, std::vector<detection>& detections)
{
	// load image, resize and pad to 640x640
	const int img_w = input.cols;
	const int img_h = input.rows;

	// solve resize scale
	int w = img_w;
	int h = img_h;
	float scale = 1.f;
	if (w > h)
	{
		scale = (float)target_size / w;
		w = target_size;
		h = h * scale;
	}
	else
	{
		scale = (float)target_size / h;
		h = target_size;
		w = w * scale;
	}

	// construct ncnn::Mat from image pixel data, swap order from bgr to rgb
	ncnn::Mat in = ncnn::Mat::from_pixels_resize(input.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h, w, h);

	// pad to target_size rectangle
	const int wpad = target_size - w;
	const int hpad = target_size - h;

	ncnn::Mat in_pad;
	ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT, 114.f);

	// apply yolov5 pre process, that is to normalize 0~255 to 0~1
	const float preprocess_values[3] = { 1 / 255.f, 1 / 255.f, 1 / 255.f };
	in_pad.substract_mean_normalize(0, preprocess_values);

	ncnn::Extractor extractor = model.create_extractor();
	extractor.input("in0", in_pad);
	ncnn::Mat out;
	extractor.extract("out0", out);

	std::vector<detection> proposals;

	// enumerate all boxes
	for (int i = 0; i < out.h; i++)
	{
		const float* ptr = out.row(i);

		const int num_class = 80; // CHANGE THIS !!!

		const float cx = ptr[0];
		const float cy = ptr[1];
		const float bw = ptr[2];
		const float bh = ptr[3];
		const float box_score = ptr[4];
		const float* class_scores = ptr + 5;

		// find class index with the biggest class score among all classes
		int class_index = 0;
		float class_score = -FLT_MAX;
		for (int j = 0; j < num_class; j++)
		{
			if (class_scores[j] > class_score)
			{
				class_score = class_scores[j];
				class_index = j;
			}
		}

		// combined score = box score * class score
		float confidence = box_score * class_score;

		// filter candidate boxes with combined score >= prob_threshold
		if (confidence < confidence_threshold)
			continue;

		// transform candidate box (center-x,center-y,w,h) to (x0,y0,x1,y1)
		float x0 = cx - bw * 0.5f;
		float y0 = cy - bh * 0.5f;
		float x1 = cx + bw * 0.5f;
		float y1 = cy + bh * 0.5f;

		// collect candidates
		detection candidate;
		candidate.rectangle.x = x0;
		candidate.rectangle.y = y0;
		candidate.rectangle.width = x1 - x0;
		candidate.rectangle.height = y1 - y0;
		candidate.class_id = class_index;
		candidate.confidence = confidence;

		proposals.push_back(candidate);
	}

	// sort all candidates by score from highest to lowest
	quick_sort_descent_inplace(proposals);

	// apply non max suppression
	std::vector<int> picked;
	nms_sorted_bboxes(proposals, picked, nms_threshold);

	// collect final result after nms
	const int count = picked.size();
	detections.resize(count);

	for (int i = 0; i < count; i++)
	{
		detections[i] = proposals[picked[i]];

		// adjust offset to original unpadded
		float x0 = (detections[i].rectangle.x - (wpad / 2)) / scale;
		float y0 = (detections[i].rectangle.y - (hpad / 2)) / scale;
		float x1 = (detections[i].rectangle.x + detections[i].rectangle.width - (wpad / 2)) / scale;
		float y1 = (detections[i].rectangle.y + detections[i].rectangle.height - (hpad / 2)) / scale;

		// clip
		x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
		y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
		x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
		y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

		detections[i].rectangle.x = x0;
		detections[i].rectangle.y = y0;
		detections[i].rectangle.width = x1 - x0;
		detections[i].rectangle.height = y1 - y0;
	}
}

// List of objects we're detecting
std::vector<std::string> load_class_list()
{
	std::vector<std::string> class_list;

	std::ifstream stream("content/models/classes.txt");

	std::string line;
	while (getline(stream, line)) {
		class_list.push_back(line);
	}

	return class_list;
}

void draw_detections(const cv::Mat& input, const std::vector<detection>& objects)
{
	std::vector<std::string> class_list = load_class_list();
	cv::Mat image = input.clone();

	for (size_t i = 0; i < objects.size(); i++)
	{
		const detection& obj = objects[i];

		cv::rectangle(image, obj.rectangle, cv::Scalar(255, 0, 0));

		std::cout << obj.confidence << std::endl;

		std::string text = std::format("{} {:.1f}%", class_list[obj.class_id], obj.confidence * 100);

		int baseLine = 0;
		cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

		int x = obj.rectangle.x;
		int y = obj.rectangle.y - label_size.height - baseLine;
		if (y < 0)
			y = 0;
		if (x + label_size.width > image.cols)
			x = image.cols - label_size.width;

		cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
			cv::Scalar(255, 255, 255), -1);

		cv::putText(image, text, cv::Point(x, y + label_size.height),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
	}

	cv::imshow("image", image);
	cv::waitKey(0);
}

int main()
{
	cv::Mat image = cv::imread("content/images/apple.jpg");

	std::vector<detection> detections;
	object_detector detector;

	auto start = std::chrono::high_resolution_clock::now();
	detector.detect(image, detections);
	auto end = std::chrono::high_resolution_clock::now();
	auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	std::cout << "Took " << ms_int << std::endl;
	std::cout << "Detection count: " << detections.size() << std::endl;

	draw_detections(image, detections);

	std::cout << "done" << std::endl;

	return 0;
}