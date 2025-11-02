#include "mapgen.h"
#include "navmesh.h"

#include <pcl/io/pcd_io.h>

#include <raylib.h>

#include <chrono>
#include <iostream>
#include <filesystem>

const std::filesystem::path data_path = std::filesystem::path(std::string(__FILE__))
	.parent_path().parent_path().parent_path().append("tests").append("bounding_boxes_test").append("data");

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>((data_path / "point_cloud.pcd").string(), *input_cloud);

	robo::point_cloud cloud;

	for (const auto& point : input_cloud->points)
	{
		cloud.add_point({ point.x, point.y, point.z });
	}
	std::cout << "Populated point cloud with " << cloud.points->size() << " points." << std::endl;

	auto start_time = std::chrono::steady_clock::now();
	pcl::PolygonMesh mesh = cloud.reconstruct_mesh_from_points();
	auto end_time = std::chrono::steady_clock::now();
	
	std::cout << "Reconstruction took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time) << std::endl;

	const int screen_width = 2048;
	const int screen_height = 1024;
	InitWindow(screen_width, screen_height, "NAVMESH VISUALIZER");

	Camera3D camera = { 0 };
	camera.position = { 0.0f, 50.0f, -5.0f };
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
	camera.fovy = 90.0f;                                // Camera field-of-view Y
	camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

	while (!WindowShouldClose())
	{
		UpdateCamera(&camera, CAMERA_FREE);

		if (IsKeyPressed(KEY_Z))
		{
			camera.target = { 0.0f, 0.0f, 0.0f };
		}

		BeginDrawing();
		ClearBackground(RAYWHITE);

		BeginMode3D(camera);
		EndMode3D();

		EndDrawing();
	}

	CloseWindow();

	return 0;
}