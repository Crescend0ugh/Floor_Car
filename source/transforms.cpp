#include "transforms.h"

namespace robo
{
	std::unique_ptr<transforms> transforms::instance = nullptr;
	std::mutex transforms::instance_mutex;
}

Eigen::Matrix4f robo::transforms::cv_to_eigen(const cv::Mat& cv_mat)
{
	Eigen::Matrix4f eigen_mat;

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			eigen_mat(i, j) = cv_mat.at<float>(i, j);
		}
	}

	return eigen_mat;
}

cv::Mat robo::transforms::eigen_to_cv(const Eigen::Matrix4f& eigen_mat)
{
	cv::Mat cv_mat;

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			cv_mat.at<float>(i, j) = eigen_mat(i, j);
		}
	}

	return cv_mat;
}

robo::transforms::transforms()
{
	// Initialize all matrices to identity
	lidar_to_camera_cv = cv::Mat::eye(4, 4, CV_32F);
	camera_to_lidar_cv = cv::Mat::eye(4, 4, CV_32F);
	camera_to_imu_cv = cv::Mat::eye(4, 4, CV_32F);
	imu_to_camera_cv = cv::Mat::eye(4, 4, CV_32F);
	lidar_to_imu_cv = cv::Mat::eye(4, 4, CV_32F);
	imu_to_lidar_cv = cv::Mat::eye(4, 4, CV_32F);

	lidar_to_camera_eigen = Eigen::Matrix4f::Identity();
	camera_to_lidar_eigen = Eigen::Matrix4f::Identity();
	camera_to_imu_eigen = Eigen::Matrix4f::Identity();
	imu_to_camera_eigen = Eigen::Matrix4f::Identity();
	lidar_to_imu_eigen = Eigen::Matrix4f::Identity();
	imu_to_lidar_eigen = Eigen::Matrix4f::Identity();

	// IMU: X = forward, Y = right, Z = up
	// World: X = right, Y = up, Z = forward
	imu_to_world_axis_rotation_eigen <<
		0, 1, 0, 0,   // World X = IMU Y
		0, 0, 1, 0,   // World Y = IMU Z
		1, 0, 0, 0,   // World Z = IMU X
		0, 0, 0, 1;
}

robo::transforms& robo::transforms::get()
{
	std::lock_guard<std::mutex> lock(instance_mutex);
	if (!instance)
	{
		instance = std::unique_ptr<transforms>(new transforms());
	}
	return *instance;
}

void robo::transforms::load(const std::string& calibration_dir)
{
	std::filesystem::path calibration_path;

	if (!calibration_dir.empty())
	{
		calibration_path = calibration_dir;
	}
	else
	{
		// Default: look for calibration_results in parent directories
		std::filesystem::path source_path(std::string(__FILE__));
		calibration_path = source_path.parent_path().parent_path().append("content").append("calibration_results");
	}

	if (!std::filesystem::exists(calibration_path)) 
	{
		std::cerr << "[Transforms] Calibration directory not found: " << calibration_path << std::endl;
		return;
	}

	std::cout << "[Transforms] Loading calibrations from: " << calibration_path << std::endl;

	// Load LiDAR to Camera transform
	std::filesystem::path lidar_camera_file = calibration_path / "lidar_camera.yml";
	cv::FileStorage lidar_camera_fs(lidar_camera_file.string(), cv::FileStorage::READ);

	if (lidar_camera_fs.isOpened()) 
	{
		lidar_camera_fs["T"] >> lidar_to_camera_cv;
		camera_to_lidar_cv = lidar_to_camera_cv.inv();

		// Convert to Eigen
		lidar_to_camera_eigen = cv_to_eigen(lidar_to_camera_cv);
		camera_to_lidar_eigen = cv_to_eigen(camera_to_lidar_cv);

		is_lidar_to_camera_loaded = true;
		std::cout << "[Transforms] Loaded LiDAR-Camera calibration" << std::endl;
	}
	else 
	{
		std::cerr << "[Transforms] Could not open lidar_camera.yml" << std::endl;
	}

	// Load Camera to IMU transform
	std::filesystem::path camera_imu_file = calibration_path / "camera_imu.yml";
	cv::FileStorage camera_imu_fs(camera_imu_file.string(), cv::FileStorage::READ);

	if (camera_imu_fs.isOpened()) 
	{
		camera_imu_fs["T"] >> camera_to_imu_cv;
		imu_to_camera_cv = camera_to_imu_cv.inv();

		// Convert to Eigen
		camera_to_imu_eigen = cv_to_eigen(camera_to_imu_cv);
		imu_to_camera_eigen = cv_to_eigen(imu_to_camera_cv);

		is_camera_to_imu_loaded = true;
		std::cout << "[Transforms] Loaded Camera-IMU calibration" << std::endl;
	}
	else 
	{
		std::cerr << "[Transforms] Could not open camera_imu.yml" << std::endl;
	}

	// Compute LiDAR to IMU
	lidar_to_imu_cv = camera_to_imu_cv * lidar_to_camera_cv;
	imu_to_lidar_cv = lidar_to_imu_cv.inv();

	lidar_to_imu_eigen = cv_to_eigen(lidar_to_imu_cv);
	imu_to_lidar_eigen = cv_to_eigen(imu_to_lidar_cv);

	std::cout << "[Transforms] All calibration matrices loaded!" << std::endl;
}

Eigen::Vector3f robo::transforms::transform_lidar_to_camera(const Eigen::Vector3f& point) const
{
	Eigen::Vector4f homogeneous(point.x(), point.y(), point.z(), 1.0f);
	Eigen::Vector4f transformed = lidar_to_camera_eigen * homogeneous;
	return Eigen::Vector3f(transformed.x(), transformed.y(), transformed.z());
}

Eigen::Vector3f robo::transforms::transform_camera_to_imu(const Eigen::Vector3f& point) const
{
	Eigen::Vector4f homogeneous(point.x(), point.y(), point.z(), 1.0f);
	Eigen::Vector4f transformed = camera_to_imu_eigen * homogeneous;
	return Eigen::Vector3f(transformed.x(), transformed.y(), transformed.z());
}

Eigen::Vector3f robo::transforms::transform_lidar_to_imu(const Eigen::Vector3f& point) const
{
	Eigen::Vector4f homogeneous(point.x(), point.y(), point.z(), 1.0f);
	Eigen::Vector4f transformed = lidar_to_imu_eigen * homogeneous;
	return Eigen::Vector3f(transformed.x(), transformed.y(), transformed.z());
}