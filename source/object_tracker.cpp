#include "object_tracker.h"

robo::object_tracker::object_tracker()
{
    params.net = "content/models/object_tracking_vittrack_2023sep.onnx";
    params.backend = cv::dnn::DNN_BACKEND_OPENCV;
    params.target = cv::dnn::DNN_TARGET_CPU;
}

int32_t robo::object_tracker::init(const cv::Mat& image, const yolo::detection& detection)
{
    single_tracker tracker;
    tracker.model = cv::TrackerVit::create(params);
    tracker.model->init(image, detection.rect);
    tracker.id = next_id;
    tracker.class_id = detection.label;

    trackers.push_back(std::move(tracker));

    return next_id++;
}

std::pair<std::vector<robo::tracking_result>, bool> robo::object_tracker::infer(const cv::Mat& image)
{
    std::vector<robo::tracking_result> results;
    bool located_something = false;

    std::vector<int32_t> trackers_to_remove;

    for (const auto& tracker : trackers)
    {
        tracking_result result;

        bool successful_update = tracker.model->update(image, result.bbox);
        bool is_bbox_reasonable = (result.bbox.x >= 0 && result.bbox.width <= image.cols && result.bbox.y >= 0 && result.bbox.width <= image.rows);

        result.is_located = successful_update && is_bbox_reasonable;
        result.confidence = tracker.model->getTrackingScore();

        result.id = tracker.id;
        result.class_id = tracker.class_id;

        if (result.is_located)
        {
            located_something = true;
        }
        else
        {
            trackers_to_remove.push_back(tracker.id);
        }

        results.push_back(std::move(result));
    }

    std::erase_if(trackers,
        [&](const robo::single_tracker& tracker) 
        {
            return std::find(trackers_to_remove.begin(), trackers_to_remove.end(), tracker.id) != trackers_to_remove.end();
        }
    );

    return std::make_pair(results, located_something);
}

void robo::object_tracker::clear_trackers()
{
    trackers.clear();
    next_id = 0;
}

bool robo::object_tracker::is_empty() const
{
    return trackers.size() == 0;
}

cv::Mat robo::object_tracker::annotate_tracker_results(
    const cv::Mat& bgr, 
    const std::vector<robo::tracking_result>& results, 
    std::chrono::milliseconds processing_time
)
{
    cv::Mat image = bgr.clone();

    for (const auto& result : results)
    {
        if (!result.is_located)
        {
            continue;
        }

        cv::rectangle(image, result.bbox, cv::Scalar(0, 255, 0), 2);

        std::string text = std::format(
            "[id: {}] {}: {:.1f}%",
            result.id,
            yolo::get_detection_class_name(result.class_id),
            result.confidence * 100
        );

        int base_line = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);

        int x = result.bbox.x;
        int y = result.bbox.y - label_size.height - base_line;
        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + base_line)),
            cv::Scalar(255, 255, 255), -1);

        cv::putText(image, text, cv::Point(x, y + label_size.height),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }

    using namespace std::chrono_literals;
    if (processing_time >= 0ms)
    {
        std::string processing_time_text = std::format("Took: {}", processing_time);
        cv::putText(image, processing_time_text, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }

    return image;
}

robo::network::camera_frame robo::object_tracker::serialize_tracker_results(
    const cv::Mat& bgr, 
    const std::vector<robo::tracking_result>& results
) const
{
    cv::Mat image = annotate_tracker_results(bgr, results);

    std::vector<uchar> pixels;

    if (image.isContinuous()) // If the matrix is continuous (no padding between rows), copy all data at once
    {
        pixels.assign(image.data, image.data + image.total() * image.elemSize());
    }
    else  // If the matrix is not continuous, copy row by row
    {
        for (int i = 0; i < image.rows; ++i)
        {
            pixels.insert(pixels.end(), image.ptr<uchar>(i), image.ptr<uchar>(i) + image.cols * image.elemSize());
        }
    }

    return robo::network::camera_frame
    {
        .camera_id = 0,
        .frame_height = image.rows,
        .frame_width = image.cols,
        .bgr_pixels = pixels,
        .processing_time = 0,
    };
}