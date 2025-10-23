#include "yolo_model.h"

#include <float.h>

static const char* class_names[] = {
	"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
	"fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
	"elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
	"skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
	"tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
	"sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
	"potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
	"microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
	"hair drier", "toothbrush"
};

static const float norm_vals[3] = { 1 / 255.f, 1 / 255.f, 1 / 255.f };

static float clamp(float val, float min = 0.f, float max = 1280.f)
{
	return val > min ? (val < max ? val : max) : min;
}

float yolo_model::softmax(const float* src, float* dst, int length)
{
	float alpha = -FLT_MAX;
	for (int c = 0; c < length; c++)
	{
		float score = src[c];
		if (score > alpha)
		{
			alpha = score;
		}
	}

	float denominator = 0;
	float dis_sum = 0;
	for (int i = 0; i < length; ++i)
	{
		dst[i] = expf(src[i] - alpha);
		denominator += dst[i];
	}
	for (int i = 0; i < length; ++i)
	{
		dst[i] /= denominator;
		dis_sum += i * dst[i];
	}
	return dis_sum;
}

yolo_model::yolo_model():
	target_size(640),
	prob_threshold(0.5),
	nms_threshold(0.45)
{
	model.opt.use_vulkan_compute = false;
	// model.opt.use_bf16_storage = true;

	// original pretrained model from https://github.com/ultralytics/ultralytics
	// the ncnn model https://github.com/nihui/ncnn-assets/tree/master/models
	if (model.load_param("content/models/yolo11n_ncnn_model/model.ncnn.param"))
	{
		exit(-1);
	}

	if (model.load_model("content/models/yolo11n_ncnn_model/model.ncnn.bin"))
	{
		exit(-1);
	}

}

void yolo_model::non_max_suppression(
	std::vector<detection>& proposals,
	std::vector<detection>& results,
	int orin_h,
	int orin_w,
	float dh,
	float dw,
	float ratio_h,
	float ratio_w
)
{
	results.clear();
	std::vector<cv::Rect> bboxes;
	std::vector<float> scores;
	std::vector<int> labels;
	std::vector<int> indices;

	for (auto& pro : proposals)
	{
		bboxes.push_back(pro.rect);
		scores.push_back(pro.prob);
		labels.push_back(pro.label);
	}

	cv::dnn::NMSBoxes(
		bboxes,
		scores,
		prob_threshold,
		nms_threshold,
		indices
	);

	for (auto i : indices)
	{
		auto& bbox = bboxes[i];
		float x0 = bbox.x;
		float y0 = bbox.y;
		float x1 = bbox.x + bbox.width;
		float y1 = bbox.y + bbox.height;
		float& score = scores[i];
		int& label = labels[i];

		x0 = (x0 - dw) / ratio_w;
		y0 = (y0 - dh) / ratio_h;
		x1 = (x1 - dw) / ratio_w;
		y1 = (y1 - dh) / ratio_h;

		x0 = clamp(x0, 0.f, orin_w);
		y0 = clamp(y0, 0.f, orin_h);
		x1 = clamp(x1, 0.f, orin_w);
		y1 = clamp(y1, 0.f, orin_h);

		detection obj;
		obj.rect.x = x0;
		obj.rect.y = y0;
		obj.rect.width = x1 - x0;
		obj.rect.height = y1 - y0;
		obj.prob = score;
		obj.label = label;
		results.push_back(obj);
	}
}

void yolo_model::generate_proposals(
	int stride,
	const ncnn::Mat& feat_blob,
	std::vector<detection>& objects
)
{
	const int reg_max = 16;
	float dst[16];
	const int num_w = feat_blob.w;
	const int num_grid_y = feat_blob.c;
	const int num_grid_x = feat_blob.h;

	const int num_class = num_w - 4 * reg_max;

	for (int i = 0; i < num_grid_y; i++)
	{
		for (int j = 0; j < num_grid_x; j++)
		{

			const float* matat = feat_blob.channel(i).row(j);

			int class_index = 0;
			float class_score = -FLT_MAX;
			for (int c = 0; c < num_class; c++)
			{
				float score = matat[4 * reg_max + c];
				if (score > class_score)
				{
					class_index = c;
					class_score = score;
				}
			}

			if (class_score >= prob_threshold)
			{

				float x0 = j + 0.5f - softmax(matat, dst, 16);
				float y0 = i + 0.5f - softmax(matat + 16, dst, 16);
				float x1 = j + 0.5f + softmax(matat + 2 * 16, dst, 16);
				float y1 = i + 0.5f + softmax(matat + 3 * 16, dst, 16);

				x0 *= stride;
				y0 *= stride;
				x1 *= stride;
				y1 *= stride;

				detection obj;
				obj.rect.x = x0;
				obj.rect.y = y0;
				obj.rect.width = x1 - x0;
				obj.rect.height = y1 - y0;
				obj.label = class_index;
				obj.prob = class_score;
				objects.push_back(obj);

			}
		}
	}
}

void yolo_model::detect(const cv::Mat& bgr, std::vector<detection>& detections)
{
	int img_w = bgr.cols;
	int img_h = bgr.rows;

	// letterbox pad to multiple of MAX_STRIDE
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

	ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h, w, h);

	int wpad = target_size - w;
	int hpad = target_size - h;

	int top = hpad / 2;
	int bottom = hpad - hpad / 2;
	int left = wpad / 2;
	int right = wpad - wpad / 2;

	ncnn::Mat in_pad;
	ncnn::copy_make_border(in,
		in_pad,
		top,
		bottom,
		left,
		right,
		ncnn::BORDER_CONSTANT,
		114.f); // What is 114? I have no idea

	in_pad.substract_mean_normalize(0, norm_vals);

	ncnn::Extractor ex = model.create_extractor();
	ex.input("in0", in_pad);

	std::vector<detection> proposals;

	// stride 8 
	{
		ncnn::Mat out;
		ex.extract("out0", out);

		std::vector<detection> objects8;
		generate_proposals(8, out, objects8);

		proposals.insert(proposals.end(), objects8.begin(), objects8.end());
	}

	// stride 16 
	{
		ncnn::Mat out;

		ex.extract("out1", out);

		std::vector<detection> objects16;
		generate_proposals(16, out, objects16);

		proposals.insert(proposals.end(), objects16.begin(), objects16.end());
	}

	// stride 32 
	{
		ncnn::Mat out;

		ex.extract("out2", out);

		std::vector<detection> objects32;
		generate_proposals(32, out, objects32);

		proposals.insert(proposals.end(), objects32.begin(), objects32.end());
	}

	// objects = proposals;
	for (auto& pro : proposals)
	{
		float x0 = pro.rect.x;
		float y0 = pro.rect.y;
		float x1 = pro.rect.x + pro.rect.width;
		float y1 = pro.rect.y + pro.rect.height;
		float& score = pro.prob;
		int& label = pro.label;

		x0 = (x0 - (wpad / 2)) / scale;
		y0 = (y0 - (hpad / 2)) / scale;
		x1 = (x1 - (wpad / 2)) / scale;
		y1 = (y1 - (hpad / 2)) / scale;

		x0 = clamp(x0, 0.f, img_w);
		y0 = clamp(y0, 0.f, img_h);
		x1 = clamp(x1, 0.f, img_w);
		y1 = clamp(y1, 0.f, img_h);

		detection obj;
		obj.rect.x = x0;
		obj.rect.y = y0;
		obj.rect.width = x1 - x0;
		obj.rect.height = y1 - y0;
		obj.prob = score;
		obj.label = label;
		detections.push_back(obj);
	}

	non_max_suppression(proposals, detections,
		img_h, img_w, hpad / 2, wpad / 2,
		scale, scale);
}

const char* get_detection_class_name(int id)
{
	return class_names[id];
}