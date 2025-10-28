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

std::filesystem::path get_content_directory(std::string name)
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

void remove_files_from_directory(std::filesystem::path path_to_directory)
{
	for (const auto& entry : std::filesystem::directory_iterator(path_to_directory)) {
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

	std::filesystem::path calibration_results_path = get_content_directory("calibration_results");

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
		cv::imwrite((calibration_results_path / path.filename()).string(), original_input);
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
	std::filesystem::path intrinsics_file_path = get_content_directory("camera_matrices") / std::format("intrinsics_{}.yml", index);

	cv::FileStorage intrinsics_file(intrinsics_file_path.string(), cv::FileStorage::WRITE);
	if (intrinsics_file.isOpened())
	{
		intrinsics_file << "M" << camera_matrix << "D" << dist_coeffs << "image_size" << image_size;
		intrinsics_file.release();

		std::cout << "\nWritten results to " << intrinsics_file_path.string() << std::endl;
	}
}

void read_intrinsics(
	int index,
	cv::Mat& intrinsic_matrix, 
	cv::Mat& dist_coeffs,
	cv::Size& image_size
)
{
	std::filesystem::path file_path = get_content_directory("camera_matrices") / std::format("intrinsics_{}.yml", index);
	cv::FileStorage file(file_path.string(), cv::FileStorage::READ);
	if (!file.isOpened())
	{
		std::cout << "Unable to open intrinsics file " << file_path << std::endl;
		exit(1);
	}

	file["M"] >> intrinsic_matrix;
	file["D"] >> dist_coeffs;
	file["image_size"] >> image_size;
}

// Get the extrinsic matrices for a stereo pair of cameras
void stereo_calibration(int left, int right, bool capture)
{
	std::filesystem::path synced_inputs_path = get_content_directory("stereo_calibration_inputs");
	std::filesystem::path synced_results_path = get_content_directory("stereo_calibration_results");

	if (capture)
	{
		cv::VideoCapture left_capture(left);
		cv::VideoCapture right_capture(right);
		if (!left_capture.isOpened() || !right_capture.isOpened())
		{
			std::cerr << "Unable to stereo calibrate: unable to open both cameras." << std::endl;
			exit(1);
		}

		remove_files_from_directory(synced_inputs_path);
		remove_files_from_directory(synced_results_path);

		int pairs_saved = 0;

		cv::Mat left_frame, right_frame;
		while (left_capture.isOpened() && right_capture.isOpened())
		{
			left_capture.grab();
			right_capture.grab();

			left_capture.retrieve(left_frame);
			right_capture.retrieve(right_frame);

			if (left_frame.empty() || right_frame.empty())
			{
				std::cout << "Captured empty frame!" << std::endl;
				break;
			}

			int key = cv::waitKey(10);
			if (key == 27) // ESC to quit
			{
				break;
			}
			else if (key == 's' || key == 'S') // S to save image
			{
				std::string left_path = (synced_inputs_path / std::format("pair{}_cam{}.png", pairs_saved, left)).string();
				std::string right_path = (synced_inputs_path / std::format("pair{}_cam{}.png", pairs_saved, right)).string();

				cv::imwrite(left_path, left_frame);
				cv::imwrite(right_path, right_frame);
				std::cout << "Saved image pair: " << pairs_saved << std::endl;

				++pairs_saved;
			}

			auto both_frames = std::vector<cv::Mat>{ left_frame, right_frame };
			cv::Mat concat_frame;
			cv::hconcat(both_frames, concat_frame);
			cv::imshow("Frames", concat_frame);
		}

		left_capture.release();
		right_capture.release();
		cv::destroyAllWindows();

		if (pairs_saved == 0)
		{
			std::cout << "No image pairs were captured. Quitting." << std::endl;
			return;
		}

		std::cout << "Done capturing inputs from left and right cameras." << std::endl;
	}
	
	int pair_count = 0;
	{
		int file_count = 0;
		for (const auto& entry : std::filesystem::directory_iterator(synced_inputs_path)) {
			if (std::filesystem::is_regular_file(entry.status())) {
				file_count++;
			}
		}
		pair_count = file_count / 2;
	}

	std::vector<std::vector<cv::Point2f>> left_image_points, right_image_points;

	for (int i = 0; i < pair_count; ++i)
	{
		std::string left_filename = std::format("pair{}_cam{}.png", i, left);
		std::string right_filename = std::format("pair{}_cam{}.png", i, right);
		auto left_image = synced_inputs_path / left_filename;
		auto right_image = synced_inputs_path / right_filename;

		cv::Mat left_input = cv::imread(left_image.string());
		cv::Mat right_input = cv::imread(right_image.string());

		if (left_input.empty() || right_input.empty())
		{
			std::cerr << "Invalid image pair " << i << std::endl;
			continue;
		}

		cv::Mat original_left = left_input; // Will keep its color
		cv::cvtColor(left_input, left_input, cv::COLOR_BGR2GRAY); // Grayscaled

		std::vector<cv::Point2f> left_corners;
		int found_left = cv::findChessboardCorners(left_input, board_size, left_corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

		cv::Mat original_right = right_input; // Will keep its color
		cv::cvtColor(right_input, right_input, cv::COLOR_BGR2GRAY); // Grayscaled

		std::vector<cv::Point2f> right_corners;
		int found_right = cv::findChessboardCorners(right_input, board_size, right_corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

		if (!found_left || !found_right)
		{
			std::cout << "Could not find chessboard corners in both images of pair " << i << std::endl;
			continue;
		}

		cv::drawChessboardCorners(original_left, board_size, left_corners, found_left);
		cv::drawChessboardCorners(original_right, board_size, right_corners, found_right);

		left_image_points.push_back(std::move(left_corners));
		right_image_points.push_back(std::move(right_corners));

		// Save calibration results
		cv::imwrite((synced_results_path / left_filename).string(), original_left);
		cv::imwrite((synced_results_path / right_filename).string(), original_left);
	}

	std::vector<std::vector<cv::Point3f>> object_points;
	object_points.resize(left_image_points.size());

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

	// Assumed to be the same for each camera	
	cv::Size image_size;

	// Load data gathered from individual calibrations
	cv::Mat left_intrinsics, left_dist_coeffs;
	read_intrinsics(left, left_intrinsics, left_dist_coeffs, image_size);

	cv::Mat right_intrinsics, right_dist_coeffs;
	read_intrinsics(right, right_intrinsics, right_dist_coeffs, image_size);
	
	cv::Mat R, T, E, F;
	double rms = cv::stereoCalibrate(
		object_points,
		left_image_points,
		right_image_points,
		left_intrinsics,
		left_dist_coeffs,
		right_intrinsics,
		right_dist_coeffs,
		image_size,
		R, T, E, F,
		cv::CALIB_FIX_INTRINSIC + cv::CALIB_SAME_FOCAL_LENGTH + cv::CALIB_FIX_ASPECT_RATIO,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5)
	);

	{
		double err = 0;
		int npoints = 0;

		std::vector<std::vector<cv::Point2f>> image_points[2] = { left_image_points, right_image_points };
		cv::Mat camera_matrices[2] = { left_intrinsics, right_intrinsics };
		cv::Mat dist_coeffs[2] = { left_dist_coeffs, right_dist_coeffs };
		std::vector<cv::Vec3f> lines[2];

		for (int i = 0; i < object_points.size(); ++i)
		{
			int npt = (int)image_points[0][i].size();
			cv::Mat imgpt[2];

			for (int j = 0; j < 2; ++j)
			{
				imgpt[j] = cv::Mat(image_points[j][i]);
				cv::undistortPoints(imgpt[j], imgpt[j], camera_matrices[j], dist_coeffs[j], cv::Mat(), camera_matrices[j]);
				cv::computeCorrespondEpilines(imgpt[j], j + 1, F, lines[j]);
			}

			for (int j = 0; j < npt; ++j)
			{
				double errij = fabs(
					image_points[0][i][j].x * lines[1][j][0] + image_points[0][i][j].y * lines[1][j][1] + lines[1][j][2]
				);

				errij += fabs(
					image_points[1][i][j].x * lines[0][j][0] + image_points[1][i][j].y * lines[0][j][1] + lines[0][j][2]
				);

				err += errij;
			}
			npoints += npt;
		}

		std::cout << "Average Reprojection Error: " << err / npoints << std::endl;
	}

	cv::Mat R1, R2, P1, P2, Q;
	//cv::Rect valid_ROI[2];
	cv::stereoRectify(
		left_intrinsics,
		left_dist_coeffs,
		right_intrinsics,
		right_dist_coeffs,
		image_size,
		R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY,
		1,
		image_size
	);

	cv::Mat left_map_x, left_map_y;
	cv::initUndistortRectifyMap(left_intrinsics, left_dist_coeffs, R1, P1, image_size, CV_16SC2, left_map_x, left_map_y);

	cv::Mat right_map_x, right_map_y;
	cv::initUndistortRectifyMap(right_intrinsics, right_dist_coeffs, R2, P2, image_size, CV_16SC2, right_map_x, right_map_y);

	std::filesystem::path extrinsics_file_path = get_content_directory("camera_matrices") / "extrinsics.yml";
	cv::FileStorage extrinsics_file(extrinsics_file_path.string(), cv::FileStorage::WRITE);
	if (extrinsics_file.isOpened())
	{
		extrinsics_file << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q 
			<< "MX1" << left_map_x << "MY1" << left_map_y << "MX2" << right_map_x << "MY2" << right_map_y;

		extrinsics_file.release();

		std::cout << "\nWritten results to " << extrinsics_file_path.string() << std::endl;
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
			remove_files_from_directory(get_content_directory("calibration_results"));
			remove_files_from_directory(get_content_directory("stereo_calibration_inputs"));
			remove_files_from_directory(get_content_directory("stereo_calibration_results"));
		}
		else if (arg == "cleanall")
		{
			remove_files_from_directory(get_content_directory("calibration_inputs"));
			remove_files_from_directory(get_content_directory("calibration_results"));
			remove_files_from_directory(get_content_directory("stereo_calibration_inputs"));
			remove_files_from_directory(get_content_directory("stereo_calibration_results"));
			remove_files_from_directory(get_content_directory("camera_matrices"));
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
		else if (arg == "scalib")
		{
			if (argc < 4)
			{
				std::cerr << "Need arguments: camera_index_1, camera_index_2" << std::endl;
				exit(1);
			}

			std::string camera_id_1 = argv[2];
			std::string camera_id_2 = argv[3];
			stereo_calibration(std::stoi(camera_id_1), std::stoi(camera_id_2), false);
		}
		else if (arg == "scalibc")
		{
			if (argc < 4)
			{
				std::cerr << "Need arguments: camera_index_1, camera_index_2" << std::endl;
				exit(1);
			}

			std::string camera_id_1 = argv[2];
			std::string camera_id_2 = argv[3];
			stereo_calibration(std::stoi(camera_id_1), std::stoi(camera_id_2), true);
		}
	}
	else
	{
		std::cout << "[Usage]" << std::endl;
		std::cout << "clean: Removes all images in the calibration_inputs and calibration_results directories" << std::endl;
		std::cout << "cleanall: Removes all images and calibration result .yml files" << std::endl;
		std::cout << "calib [camera_index]: Find camera's intrinsic matrix using input images" << std::endl;
		std::cout << "scalib [left_camera_index] [right_camera_index]: Stereo calibrate two cameras with saved captures" << std::endl;
		std::cout << "scalibc [left_camera_index] [right_camera_index]: Take synchronized captures and stereo calibrate two cameras" << std::endl;
	}
	
	return 0;
}