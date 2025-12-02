/*
	world.h

	Handles robot navigation, meshing and pose estimation
*/

#pragma once

#include "async_processor.h"
#include "kalman_filter.h"
#include "mapgen.h"
#include "navmesh.h"
#include "navgeometry.h"
#include "path.h"
#include "transforms.h"

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>

namespace robo
{
	struct pose
	{
		Eigen::Affine3f transform;
		std::chrono::steady_clock::time_point timestamp;
	};

	struct meshing_result
	{
		bool success = false;
		size_t vertex_count = 0;
		size_t triangle_count = 0;
		std::chrono::milliseconds processing_time;
	};

	class world : public async_processor<meshing_result>
	{
	private:
		class navmesh navmesh;
		class navgeometry navgeometry;

		mutable std::mutex pose_mutex;
		mutable std::mutex point_cloud_mutex;
		mutable std::mutex navmesh_mutex;

		pose current_pose; // Current world pose
		std::deque<pose> pose_history; // Previous poses stored for interpolation

		ukf filter;

		meshing_result process_impl() override;
		bool on_init() override;
		void on_shutdown() override;
		
		void update_pose();

	public:
		class path path;

		// The cumulative point cloud
		point_cloud world_point_cloud;

		std::atomic<bool> navmesh_ready{ false };
		std::atomic<bool> is_returning{ false };

		world();

		pose get_pose_at_timestamp(std::chrono::steady_clock::time_point target_time);

		void update_pose_from_imu(const imu_reading& imu, double delta_time = 0.01);
		void update_pose_from_encoders(const encoder_reading& encoders);

		pose get_current_pose() const;
		size_t get_point_cloud_size() const;
		void clear_point_cloud();
	};
}
