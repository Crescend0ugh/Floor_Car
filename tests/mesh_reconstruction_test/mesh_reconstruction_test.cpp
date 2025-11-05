#if defined(_WIN32)           
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif

#include "mapgen.h"
#include "navmesh.h"
#include "navgeometry.h"
#include "path.h"

#include "raylib.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

#include <chrono>
#include <iostream>
#include <filesystem>

const std::filesystem::path data_path = std::filesystem::path(std::string(__FILE__)).parent_path().append("data");

robo::navigation_params agent = {
	.agent_radius = 0.5f,
	.agent_height = 1.0f,
	.max_slope = 45.0f,
	.max_climb = 0.5f,
	.cell_size = 1.0f,
	.cell_height = 1.0f,

	.tile_size = 50.0f,

	.border_offset = 1,
	.detail_sample_dist = 6.0f,
	.detail_sample_max_error = 1.0f,
};

void draw_path_points(robo::path& path)
{
	Vector3 start{ path.start.x, path.start.y, path.start.z };
	Vector3 end{ path.end.x, path.end.y, path.end.z };
	DrawSphere(start, 0.5f, RED);
	DrawSphere(end, 0.75f, GREEN);

	Vector3 previous_position_vector = start;
	for (int i = 1; i < path.waypoints.size() - 1; ++i)
	{
		const robo::vector3f& next_position = path.waypoints[i];
		Vector3 next_position_vector = { next_position.x, next_position.y, next_position.z };

		DrawSphere(next_position_vector, 0.3f, BLUE);
		DrawLine3D(previous_position_vector, next_position_vector, BLUE);
		previous_position_vector = next_position_vector;
	}

	if (path.waypoints.size() > 0)
	{
		DrawLine3D(previous_position_vector, end, BLUE);
	}
}

Model create_point_cloud_model(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
	Mesh mesh = { 0 };
	mesh.vertexCount = point_cloud->points.size();
	mesh.triangleCount = 0; // No triangles for a point cloud

	mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));

	for (int i = 0; i < point_cloud->points.size(); ++i)
	{
		mesh.vertices[3 * i] = point_cloud->points[i].x;
		mesh.vertices[3 * i + 1] = point_cloud->points[i].y;
		mesh.vertices[3 * i + 2] = point_cloud->points[i].z;
	}

	UploadMesh(&mesh, false);

	// UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * sizeof(float), 0);

	return LoadModelFromMesh(mesh);
}

Model create_reconstructed_mesh(const robo::navgeometry& geometry)
{
	Mesh mesh = { 0 };
	mesh.vertexCount = geometry.vertices.size() / 3;
	mesh.triangleCount = geometry.triangles.size() / 3;
	
	mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
	mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));
	mesh.normals = (float*)MemAlloc(geometry.normals.size() * sizeof(float));

	for (int i = 0; i < geometry.vertices.size(); ++i)
	{
		mesh.vertices[i] = geometry.vertices[i];
	}

	for (int i = 0; i < geometry.triangles.size(); ++i)
	{
		mesh.indices[i] = geometry.triangles[i];
	}

	for (int i = 0; i < geometry.normals.size(); ++i)
	{
		mesh.normals[i] = geometry.normals[i];
	}

	UploadMesh(&mesh, false);

	return LoadModelFromMesh(mesh);
}

Mesh draw_navmesh_tile(const dtMeshTile* tile)
{
	std::vector<float> vertices;

	int triangle_count = 0;

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
			continue;

		const dtPolyDetail* pd = &tile->detailMeshes[i];

		for (int j = 0; j < pd->triCount; ++j)
		{
			++triangle_count;

			const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < p->vertCount)
				{
					float* vert = &tile->verts[p->verts[t[k]] * 3];
					vertices.push_back(vert[0]);
					vertices.push_back(vert[1]);
					vertices.push_back(vert[2]);
				}
				else
				{
					float* vert = &tile->detailVerts[(pd->vertBase + t[k] - p->vertCount) * 3];
					vertices.push_back(vert[0]);
					vertices.push_back(vert[1]);
					vertices.push_back(vert[2]);
				}
			}
		}
	}

	Mesh mesh = { 0 };
	mesh.vertices = new float[vertices.size()];
	for (int i = 0; i < vertices.size(); ++i)
	{
		mesh.vertices[i] = vertices[i];
	}
	mesh.vertexCount = triangle_count * 3;
	mesh.triangleCount = triangle_count;

	return mesh;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;

	float voxel_size = 0.2f; // Edge length of the 3D grid cube.
	voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
	voxel_grid.setInputCloud(cloud);
	voxel_grid.filter(*downsampled_cloud);

	return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transform_point_cloud_to_camera_coordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// LiDAR: X = forward, Y = left, Z = up
	// Camera: X = left, Y = up, -Z = foward
	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

	// New X = -Old Y
	transform_matrix(0, 1) = -1.0f;
	transform_matrix(0, 0) = 0.0f;

	// New Y = +Old Z
	transform_matrix(1, 2) = 1.0f;
	transform_matrix(1, 1) = 0.0f;

	// New Z = -Old X
	transform_matrix(2, 0) = -1.0f;
	transform_matrix(2, 2) = 0.0f;

	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix);

	return transformed_cloud;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>((data_path / "point_cloud.pcd").string(), *input_cloud);

	std::cout << "Input point cloud size is " << input_cloud->size() << " points." << std::endl;

	// Downsampling

	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = downsample_point_cloud(input_cloud);
	std::cout << "After voxel grid filtering: " << downsampled->size() << " points." << std::endl;

	downsampled = transform_point_cloud_to_camera_coordinates(downsampled);

	robo::point_cloud cloud;

	for (const auto& point : downsampled->points)
	{
		cloud.add_point({ point.x, point.y, point.z });
	}
	std::cout << "Populated point cloud with " << cloud.points->size() << " points." << std::endl;

	// Reconstruction

	auto start_time = std::chrono::steady_clock::now();
	pcl::PolygonMesh mesh = cloud.reconstruct_mesh_from_points();
	auto end_time = std::chrono::steady_clock::now();
	
	std::cout << "Reconstruction took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time) << std::endl;

	pcl::io::savePLYFile((data_path / "output.ply").string(), mesh);

	auto components = cloud.extract_mesh_components(mesh);

	std::cout << "Vertices count: " << components.vertices.size() / 3 << std::endl;
	std::cout << "Triangles count: " << components.triangles.size() / 3 << std::endl;

	// Navmesh construction

	robo::navgeometry geometry;
	geometry.load(components.vertices, components.triangles);

	std::cout << "Navmesh bounds: " << std::endl;
	std::cout << geometry.min_bounds[0] << ", " << geometry.min_bounds[1] << ", " << geometry.min_bounds[2] << std::endl;
	std::cout << geometry.max_bounds[0] << ", " << geometry.max_bounds[1] << ", " << geometry.max_bounds[2] << std::endl;

	geometry.set_navmesh_min_bounds(robo::vector3f(-30.0f, -5.0f, -30.0f));
	geometry.set_navmesh_max_bounds(robo::vector3f(50.0f, 20.0f, 50.0f));

	BoundingBox navmesh_bbox = { 0 };
	navmesh_bbox.min = Vector3(geometry.get_min_bounds()[0], geometry.get_min_bounds()[1], geometry.get_min_bounds()[2]);
	navmesh_bbox.max = Vector3(geometry.get_max_bounds()[0], geometry.get_max_bounds()[1], geometry.get_max_bounds()[2]);

	robo::navmesh navmesh(agent);
	navmesh.set_geometry(geometry);
	navmesh.build();

	robo::path path;
	path.init(&navmesh);
	path.set_start(robo::vector3f { -8.3472f, -2.7779f, 12.257f });
	path.set_end(robo::vector3f { -4.5572f, -1.0036f, -14.273f });

	std::cout << "Path waypoint count: " << path.waypoints.size() << std::endl;

	/////////////////////////////////////////////////

	const int screen_width = 2048;
	const int screen_height = 1024;

	InitWindow(screen_width, screen_height, "NAVMESH VISUALIZER");

	Model point_cloud_model = create_point_cloud_model(downsampled);
	Model reconstructed_mesh_model = create_reconstructed_mesh(geometry);

	std::vector<Model> navmesh_models;
	const dtNavMesh* nmesh = navmesh.navmesh_internal;
	for (int i = 0; i < nmesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nmesh->getTile(i);
		if (!tile->header)
		{
			continue;
		}

		Mesh mesh = draw_navmesh_tile(tile);
		UploadMesh(&mesh, false);

		Model model = LoadModelFromMesh(mesh);
		navmesh_models.push_back(model);
	}

	Camera3D camera = { 0 };
	camera.position = { -10.0f, 0.0f, 5.0f };
	camera.target = { 50.0f, 0.0f, 0.0f };
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

		ClearBackground(BLACK);

		BeginMode3D(camera);

		DrawBoundingBox(navmesh_bbox, RED);

		DrawModelWires(
			reconstructed_mesh_model,
			Vector3{ 0.0f, 0.0f, 0.0f }, 
			1.0f,
			ColorAlpha(ORANGE, 0.2f)
		);

		DrawModelPoints(point_cloud_model, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, SKYBLUE);

		for (const auto& model : navmesh_models)
		{
			DrawModelWires(model, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, PURPLE);
		}
		draw_path_points(path);

		EndMode3D();

		EndDrawing();
	}

	CloseWindow();

	return 0;
}