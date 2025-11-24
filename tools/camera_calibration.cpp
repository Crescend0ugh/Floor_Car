#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <string>
#include <format>
#include <filesystem>
#include <fstream>
#include <stdlib.h>

static std::filesystem::path get_content_directory(std::string name)
{
	std::string file_path{ __FILE__ };
	std::filesystem::path source_path(file_path);
	std::filesystem::path content_path = source_path.parent_path().parent_path().parent_path().append("content");

	std::filesystem::path target_directory = content_path.append(name);
	if (!std::filesystem::exists(target_directory))
	{
		std::filesystem::create_directories(target_directory);
	}

	return target_directory;
}

static void remove_files_from_directory(std::filesystem::path path_to_directory)
{
	for (const auto& entry : std::filesystem::directory_iterator(path_to_directory))
	{
		if (std::filesystem::is_regular_file(entry.status())) {
			std::filesystem::remove(entry.path());
			std::cout << "Removed file: " << entry.path() << std::endl;
		}
	}
}

cv::Size board_size = cv::Size{ 9, 6 };
int square_size_mm = 1; // CHANGE TO REAL SIZE

// Get the intrinsic matrix of a single camera
void calibrate(int index)
{
	cv::VideoCapture capture(index);
	if (!capture.isOpened())
	{
		std::cerr << "Unable to open camera " << index << "!" << std::endl;
		exit(1);
	}

	std::filesystem::path calibration_inputs_path = get_content_directory("calibration_inputs");

	int images_saved = 0;
	
	cv::Mat frame;
	while (capture.isOpened())
	{
		capture >> frame;

		if (frame.empty())
		{
			std::cout << "Captured empty frame!" << std::endl;
			break;
		}

		int key = cv::waitKey(5);
		if (key == 27) // ESC to quit
		{
			break;
		}
		else if (key == 's' || key == 'S') // S to save image
		{
			std::string file_path = (calibration_inputs_path / std::format("cam{}_img{}.png", index, images_saved)).string();

			cv::imwrite(file_path, frame);
			std::cout << "Saved image: " << images_saved << std::endl;

			++images_saved;
		}

		cv::imshow(std::format("CAPTURING CALIBRATION INPUTS FOR CAMERA {}", index), frame);
	}

	capture.release();
	cv::destroyAllWindows();

	std::filesystem::path outputs_path = get_content_directory("calibration_outputs");

	// cam0 in cam0_img1
	std::string prefix = std::format("cam{}", index);

	std::vector<std::vector<cv::Point2f>> image_points;
	cv::Size image_size;

	for (const auto& entry : std::filesystem::directory_iterator(calibration_inputs_path)) {
		auto& path = entry.path();

		if (path.filename().string().find(prefix) == std::string::npos)
		{
			continue;
		}

		cv::Mat input = cv::imread(path.string());

		if (input.empty())
		{
			std::cerr << "Invalid image at path " << path.string() << std::endl;
			continue;
		}

		image_size = input.size();

		cv::Mat original_input = input; // Will keep its color
		cv::cvtColor(input, input, cv::COLOR_BGR2GRAY); // Grayscaled

		std::vector<cv::Point2f> corners;
		int found = cv::findChessboardCorners(input, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

		if (!found)
		{
			std::cout << "Could not find chessboard corners of " << path.filename().string() << std::endl;
			continue;
		}

		cv::drawChessboardCorners(original_input, board_size, corners, found);

		image_points.push_back(std::move(corners));

		// Save calibration results
		cv::imwrite((outputs_path / path.filename()).string(), original_input);
	}

	if (image_points.size() == 0)
	{
		std::cout << "No images could be used for calibration." << std::endl;
		return;
	}

	std::vector<std::vector<cv::Point3f>> object_points;
	object_points.resize(image_points.size());

	// 3D positions of the chessboard corners
	for (int i = 0; i < object_points.size(); ++i)
	{
		for (int j = 0; j < board_size.height; ++j)
		{
			for (int k = 0; k < board_size.width; ++k)
			{
				object_points[i].push_back(cv::Point3f(float(j * square_size_mm), float(k * square_size_mm), 0.0f));
			}
		}
	}

	cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat dist_coeffs, R, T;

	double rms = cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coeffs, R, T);

	std::cout << "Camera matrix:\n " << camera_matrix << std::endl;
	std::cout << "Distance coefficients:\n " << dist_coeffs << std::endl;
	std::cout << "RMS Reprojection Error: " << rms << std::endl;

	// Write instrinsic matrix and coefficients to file
	std::filesystem::path intrinsics_file_path = get_content_directory("calibration_results") / "intrinsics.yml";

	cv::FileStorage intrinsics_file(intrinsics_file_path.string(), cv::FileStorage::WRITE);
	if (intrinsics_file.isOpened())
	{
		intrinsics_file << "K" << camera_matrix << "D" << dist_coeffs << "image_size" << image_size;
		intrinsics_file.release();

		std::cout << "\nWritten results to " << intrinsics_file_path.string() << std::endl;
	}
}

int main(int argc, char** argv)
{
	if (argc > 1)
	{
		std::string arg = argv[1];
		if (arg == "clean")
		{
			remove_files_from_directory(get_content_directory("calibration_inputs"));
			remove_files_from_directory(get_content_directory("calibration_outputs"));
		}
		else if (arg == "cleanall")
		{
			remove_files_from_directory(get_content_directory("calibration_inputs"));
			remove_files_from_directory(get_content_directory("calibration_outputs"));
			remove_files_from_directory(get_content_directory("calibration_results"));
		}
		else if (arg == "calib")
		{
			if (argc < 3)
			{
				std::cerr << "Missing argument: camera_index" << std::endl;
				exit(1);
			}

			std::string camera_id = argv[2];
			calibrate(std::stoi(camera_id));
		}
	}
	else
	{
		std::cout << "[Usage]" << std::endl;
		std::cout << "clean: Removes all images in the calibration_inputs and calibration_results directories" << std::endl;
		std::cout << "cleanall: Removes all images and calibration result .yml files" << std::endl;
		std::cout << "calib [camera_index]: Find camera's intrinsic matrix using input images" << std::endl;
	}
	
	return 0;
}