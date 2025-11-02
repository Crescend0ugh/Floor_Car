#include "mapgen.h"
#include "navmesh.h"
#include "navgeometry.h"
#include "path.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <chrono>
#include <iostream>
#include <filesystem>

const std::filesystem::path data_path = std::filesystem::path(std::string(__FILE__)).parent_path().append("data");

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;

	float voxel_size = 0.1f; // Edge length of the 3D grid cube.
	voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
	voxel_grid.setInputCloud(cloud);
	voxel_grid.filter(*downsampled_cloud);

	return downsampled_cloud;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>((data_path / "point_cloud.pcd").string(), *input_cloud);

	std::cout << "Input point cloud size is " << input_cloud->size() << " points." << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = downsample_point_cloud(input_cloud);
	std::cout << "After voxel grid filtering: " << downsampled->size() << " points." << std::endl;

	robo::point_cloud cloud;

	for (const auto& point : downsampled->points)
	{
		cloud.add_point({ point.x, point.y, point.z });
	}
	std::cout << "Populated point cloud with " << cloud.points->size() << " points." << std::endl;

	auto start_time = std::chrono::steady_clock::now();
	pcl::PolygonMesh mesh = cloud.reconstruct_mesh_from_points();
	auto end_time = std::chrono::steady_clock::now();
	
	std::cout << "Reconstruction took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time) << std::endl;

	pcl::io::savePLYFile((data_path / "output.ply").string(), mesh);

	auto components = cloud.extract_mesh_components(mesh);

	std::cout << "Vertices count: " << components.vertices.size() / 3 << std::endl;
	std::cout << "First vert: " << components.vertices[0] << ", " << components.vertices[1] << ", " << components.vertices[2] << std::endl;
	std::cout << "Triangles count: " << components.triangles.size() / 3 << std::endl;

	robo::navgeometry geometry;
	geometry.load(components.vertices, components.triangles);

	std::cout << "BOUNDS: " << std::endl;
	std::cout << geometry.min_bounds[0] << ", " << geometry.min_bounds[1] << ", " << geometry.min_bounds[2] << std::endl;
	std::cout << geometry.max_bounds[0] << ", " << geometry.max_bounds[1] << ", " << geometry.max_bounds[2] << std::endl;

	robo::navigation_params params;
	params.cell_height = 0.2f;
	params.cell_size = 0.3f;
	params.agent_height = 1.0f;
	params.agent_radius = 0.5f;
	params.max_slope = 20.0f;
	params.tile_size = 30.0f;

	robo::navmesh navmesh(params);
	navmesh.on_mesh_changed(&geometry);
	navmesh.build();

	robo::path path;
	path.init(&navmesh);

	float start[3] = { 18.603f, -5.5495f, 1.5548f };
	path.set_start(start);

	float end[3] = { -29.433f, -19.446f, -3.8896f };
	path.set_end(end);

	const int path_waypoint_count = path.get_waypoint_count();
	std::cout << path_waypoint_count << std::endl;

	for (int i = 0; i < path_waypoint_count; ++i)
	{
		const float* next_position = path.get_waypoint_from_id(i);
		std::cout << "[path " << i << "]: " << next_position[0] << ", " << next_position[1] << ", " << next_position[2] << std::endl;
	}

	return 0;
}