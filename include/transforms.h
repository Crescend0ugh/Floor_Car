/*
	transforms.h

	Stores the extrinsic matrices and their inverses

	1)	LiDAR to camera: from calibration
		See https://www.mathworks.com/help/lidar/ug/lidar-and-camera-calibration.html

		LiDAR: X = forward, Y = left, Z = up
		Camera: X = right, Y = down, -Z = foward

	2)	Camera to IMU: from calibration
		See https://www.mathworks.com/help/nav/ug/estimate-camera-to-imu-transformation-using-extrinsic-calibration.html

		Camera: X = right, Y = down, -Z = foward
		IMU: X = forward, Y = right, Z = up

	For compatibility with Recast Navigation, the world coordinate system is right-handed, Y up
		World: X = right, Y = up, Z = forward

	So an additional transform is needed to convert from IMU to world
*/

#pragma once

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>

namespace robo
{
	// Singleton
	class transforms
	{
	private:
		static std::unique_ptr<transforms> instance;
		static std::mutex instance_mutex;

		static Eigen::Matrix4f cv_to_eigen(const cv::Mat& cv_mat);
		static cv::Mat eigen_to_cv(const Eigen::Matrix4f& eigen_mat);

		transforms();

	public:
		// OpenCV matrices (row-major, as loaded from file)
		cv::Mat lidar_to_camera_cv;
		cv::Mat camera_to_lidar_cv;
		cv::Mat camera_to_imu_cv;
		cv::Mat imu_to_camera_cv;
		cv::Mat lidar_to_imu_cv;
		cv::Mat imu_to_lidar_cv;

		// Eigen matrices (column-major by default)
		Eigen::Matrix4f lidar_to_camera_eigen;
		Eigen::Matrix4f camera_to_lidar_eigen;
		Eigen::Matrix4f camera_to_imu_eigen;
		Eigen::Matrix4f imu_to_camera_eigen;
		Eigen::Matrix4f lidar_to_imu_eigen;
		Eigen::Matrix4f imu_to_lidar_eigen;

		// IMU to World axis rotation
		Eigen::Matrix4f imu_to_world_axis_rotation_eigen;

		bool is_lidar_to_camera_loaded = false;
		bool is_camera_to_imu_loaded = false;

		transforms(const transforms&) = delete;
		transforms& operator=(const transforms&) = delete;

		// Singleton getter
		static transforms& get();

		// Load calibration matrices from YAML files
		// Should be called once at startup
		void load(const std::string& calibration_dir = "");

		Eigen::Vector3f transform_lidar_to_camera(const Eigen::Vector3f& point) const;
		Eigen::Vector3f transform_camera_to_imu(const Eigen::Vector3f& point) const;
		Eigen::Vector3f transform_lidar_to_imu(const Eigen::Vector3f& point) const;
	};
}