#include "client/point_cloud_visualizer.h"

#include "rlgl.h"

void ui::point_cloud_visualizer::update_mesh()
{
    if (!needs_update)
    {
        return;
    }

    int new_point_count = points.size();
    // No points => clean up mesh
    if (new_point_count == 0)
    {
        UnloadModel(point_cloud_model);
        point_cloud_model = { 0 };
        point_cloud_mesh = { 0 };
        needs_update = false;
        return;
    }

    // If the number of vertices has in any way changed, we have to destroy and re-initialize the mesh.
    bool needs_resize = (new_point_count != point_cloud_mesh.vertexCount);
    if (needs_resize)
    {
        UnloadModel(point_cloud_model);

        point_cloud_mesh = { 0 };
        point_cloud_mesh.vertexCount = new_point_count;
        point_cloud_mesh.triangleCount = 0;

        point_cloud_mesh.vertices = (float*)MemAlloc(new_point_count * 3 * sizeof(float));
    }

    // Update vertices
    for (int i = 0; i < new_point_count; i++)
    {
        point_cloud_mesh.vertices[3 * i] = points[i].x;
        point_cloud_mesh.vertices[3 * i + 1] = points[i].y;
        point_cloud_mesh.vertices[3 * i + 2] = points[i].z;
    }

    if (needs_resize)
    {
        // Upload new mesh to GPU
        UploadMesh(&point_cloud_mesh, true);
        point_cloud_model = LoadModelFromMesh(point_cloud_mesh);
    }
    else
    {
        // The VBOs already exist and are the right size, so just update the data within them, which is much faster.
        UpdateMeshBuffer(point_cloud_mesh, 0, point_cloud_mesh.vertices, new_point_count * 3 * sizeof(float), 0);
    }

    // Reset flag
    needs_update = false;
}

ui::point_cloud_visualizer::point_cloud_visualizer():
	point_cloud_mesh({ 0 }),
	reconstructed_mesh({ 0 }),
    point_cloud_model({ 0 })
{
	point_cloud_mesh.triangleCount = 0; // No triangles
}

ui::point_cloud_visualizer::~point_cloud_visualizer()
{
    UnloadModel(point_cloud_model);
}

void ui::point_cloud_visualizer::draw()
{
    update_mesh();

    if (point_cloud_mesh.vertexCount == 0)
    {
        return;
    }

    rlSetPointSize(10.0f);

    DrawModelPoints(point_cloud_model, Vector3(0, 0, 0), 1.0f, BLACK);

    rlSetPointSize(1.0f);
}

void ui::point_cloud_visualizer::clear()
{
	points.clear();
	needs_update = true;
}

void ui::point_cloud_visualizer::set_points(const std::vector<robo::vector3f>& new_points)
{
	points = new_points;
	needs_update = true;
}

void ui::point_cloud_visualizer::add_points(const std::vector<robo::vector3f>& delta_points)
{
	points.insert(points.end(), delta_points.begin(), delta_points.end());
	needs_update = true;
}

void ui::point_cloud_visualizer::set_triangles(const std::vector<uint32_t>& triangle_indices)
{

}