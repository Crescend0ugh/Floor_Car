#include "transforms.h"
#include "world.h"

static Eigen::Quaternionf slerp(const Eigen::Quaternionf& q1, const Eigen::Quaternionf& q2, float alpha) 
{
    Eigen::Quaternionf q2_adjusted = q2;

    // If dot product is negative, negate one quaternion to take shorter path
    float dot = q1.dot(q2);
    if (dot < 0.0f) 
    {
        q2_adjusted.coeffs() = -q2.coeffs();
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation
    constexpr float dot_threshold = 0.9995f;
    if (dot > dot_threshold)
    {
        Eigen::Quaternionf result;
        result.coeffs() = q1.coeffs() + alpha * (q2_adjusted.coeffs() - q1.coeffs());
        result.normalize();
        return result;
    }

    float theta_0 = std::acos(dot); // Angle between input quaternions
    float theta = theta_0 * alpha; // Angle between q1 and result
    float sin_theta = std::sin(theta);
    float sin_theta_0 = std::sin(theta_0);

    float s1 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    float s2 = sin_theta / sin_theta_0;

    Eigen::Quaternionf result;
    result.coeffs() = s1 * q1.coeffs() + s2 * q2_adjusted.coeffs();
    result.normalize();

    return result;
}

robo::world::world():
	navmesh(robo::navigation_params
        {
            .agent_radius = 0.3f,
            .agent_height = 0.5f,
            .max_slope = 30.0f,
            .max_climb = 0.1f
        }
    ),
    filter(0.05, 0.1) // TODO: Find actual wheel measurements
{
    current_pose.transform = Eigen::Affine3f::Identity();
    current_pose.timestamp = std::chrono::steady_clock::now();
}

bool robo::world::on_init()
{
    return true;
}

void robo::world::on_shutdown()
{
}

robo::meshing_result robo::world::process_impl()
{
    meshing_result result;
    auto start_time = std::chrono::high_resolution_clock::now();

    // Copy point cloud for processing
    point_cloud cloud_copy;
    {
        std::lock_guard<std::mutex> lock(point_cloud_mutex);

        if (world_point_cloud.points->points.empty())
        {
            result.success = false;
            return result;
        }

        cloud_copy = world_point_cloud;
        std::cout << "[World] Meshing " << cloud_copy.points->points.size() << " points" << std::endl;
    }

    if (is_returning.load())
    {
        // More aggressive downsampling on return trip
        cloud_copy.points = cloud_copy.voxel_grid_downsample(0.15);
        std::cout << "[World] Downsampled to " << cloud_copy.points->points.size() << " points" << std::endl;
    }
    else
    {
        // Downsample
        cloud_copy.points = cloud_copy.voxel_grid_downsample();
        std::cout << "[World] Downsampled to " << cloud_copy.points->points.size() << " points" << std::endl;

        // Downsample again, keeping points in front
        cloud_copy.points = cloud_copy.filter_navigation_box(
            get_current_pose().transform,
            10.0f,
            1.5f, // Keep points 1.5 meters behind to ensure the mesh is continuous
            5.0f,
            5.0f,
            3.0f,
            0.5f
        );

        std::cout << "[World] Filtered to " << cloud_copy.points->points.size() << " relevant points" << std::endl;

        // Only mesh if we have enough points
        if (cloud_copy.points->points.size() < 100)
        {
            result.success = false;
            return result;
        }
    }
    
    // Reconstruct mesh
    auto reconstructed_mesh = cloud_copy.reconstruct_mesh_from_points();

    // Extract mesh components
    auto components = cloud_copy.extract_mesh_components(reconstructed_mesh);
    result.vertex_count = components.vertices.size();
    result.triangle_count = components.triangles.size();

    auto end_time = std::chrono::high_resolution_clock::now();

    result.processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "[World] Mesh reconstruction completed: "
        << result.vertex_count << " vertices, "
        << result.triangle_count << " triangles, "
        << result.processing_time.count() << "ms" << std::endl;

    // Update navigation mesh
    {
        std::lock_guard<std::mutex> lock(navmesh_mutex);

        navmesh_ready.store(false);
        // NOTE: These get MOVED!
        navgeometry.load(components.vertices, components.triangles);
        navmesh.set_geometry(navgeometry);
        navmesh.build();
        path.init(&navmesh);

        navmesh_ready.store(true);
    }

    end_time = std::chrono::high_resolution_clock::now();
    result.processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.success = true;

    std::cout << "[World] Navigation mesh generated: "
        << result.vertex_count << " vertices, "
        << result.triangle_count << " triangles, "
        << result.processing_time.count() << "ms" << std::endl;

    return result;
}

robo::pose robo::world::get_pose_at_timestamp(std::chrono::steady_clock::time_point target_time)
{
    std::lock_guard<std::mutex> lock(pose_mutex);

    // No pose -> Use current pose
    if (pose_history.empty()) 
    {
        return current_pose;
    }

    if (pose_history.size() == 1)
    {
        return pose_history.front();
    }

    // Find the two poses that bracket the target time
    auto it_after = std::lower_bound(pose_history.begin(), pose_history.end(), target_time,
        [](const pose& pose, std::chrono::steady_clock::time_point time)
        {
            return pose.timestamp < time;
        }
    );

    // Target time is before all recorded poses
    if (it_after == pose_history.begin()) 
    {
        return pose_history.front();
    }

    // Target time is after all recorded poses
    if (it_after == pose_history.end()) 
    {
        return pose_history.back();
    }

    // Get two bracketing poses
    auto it_before = std::prev(it_after);
    const pose& pose_before = *it_before;
    const pose& pose_after = *it_after;

    // Calculate alpha
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(pose_after.timestamp - pose_before.timestamp).count();
    auto elapsed_duration = std::chrono::duration_cast<std::chrono::microseconds>(target_time - pose_before.timestamp).count();

    // Avoid division by zero
    if (total_duration <= 0) 
    {
        return pose_before;
    }

    float alpha = static_cast<float>(elapsed_duration) / static_cast<float>(total_duration);
    alpha = std::max(0.0f, std::min(1.0f, alpha));

    pose interpolated;
    interpolated.timestamp = target_time;

    // Lerp position
    Eigen::Vector3f pos_before = pose_before.transform.translation();
    Eigen::Vector3f pos_after = pose_after.transform.translation();
    Eigen::Vector3f interpolated_pos(
        std::lerp(pos_before.x(), pos_after.x(), alpha),
        std::lerp(pos_before.y(), pos_after.y(), alpha),
        std::lerp(pos_before.z(), pos_after.z(), alpha)
    );

    // Slerp rotation
    Eigen::Quaternionf rot_before(pose_before.transform.rotation());
    Eigen::Quaternionf rot_after(pose_after.transform.rotation());
    Eigen::Quaternionf interpolated_rot = slerp(rot_before, rot_after, alpha);

    interpolated.transform = Eigen::Affine3f::Identity();
    interpolated.transform.translate(interpolated_pos);
    interpolated.transform.rotate(interpolated_rot);

    return interpolated;
}

void robo::world::update_pose()
{
    // In IMU space
    Eigen::Affine3f imu_transform = filter.get_transform().cast<float>();

    Eigen::Affine3f world_transform;
    world_transform.matrix() = transforms::get().imu_to_world_axis_rotation_eigen * imu_transform.matrix();

    pose new_pose;

    // Convert from IMU to World space
    new_pose.transform = world_transform;
    new_pose.timestamp = std::chrono::steady_clock::now();

    // Add to history
    pose_history.push_back(new_pose);
    constexpr size_t max_pose_history = 300; // 300 Hz = 3 seconds of history
    if (pose_history.size() > max_pose_history)
    {
        pose_history.pop_front();
    }

    current_pose = new_pose;
}

void robo::world::update_pose_from_imu(const robo::imu_reading& imu, double delta_time)
{
    std::lock_guard<std::mutex> lock(pose_mutex);

    filter.predict(imu, delta_time);
    update_pose();
}

void robo::world::update_pose_from_encoders(const robo::encoder_reading& encoders)
{
    std::lock_guard<std::mutex> lock(pose_mutex);

    filter.update_encoders(encoders);
    update_pose();
}

robo::pose robo::world::get_current_pose() const
{
    std::lock_guard<std::mutex> lock(pose_mutex);
    return current_pose;
}

size_t robo::world::get_point_cloud_size() const
{
    std::lock_guard<std::mutex> lock(point_cloud_mutex);
    return world_point_cloud.points->points.size();
}

void robo::world::clear_point_cloud()
{
    std::lock_guard<std::mutex> lock(point_cloud_mutex);
    world_point_cloud.points->clear();
    navmesh_ready.store(false);
}