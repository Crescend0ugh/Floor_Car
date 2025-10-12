#include "navmesh.h"
#include "path.h"
#include "raylib.h"

#include <iostream>
#include <vector>

void draw_path_points(path* path) 
{
    const int path_waypoint_count = path->get_waypoint_count();
    Vector3 previous_position_vector = { 0.0f };
    
    for (int i = 0; i < path_waypoint_count; ++i) {
        const float* next_position = path->get_waypoint_from_id(i);
        Vector3 next_position_vector = { next_position[0], next_position[1], next_position[2] };

        // The start position is red, and the end is blue
        if (i == 0) {
            previous_position_vector = next_position_vector;
            DrawSphere(next_position_vector, 0.5f, RED);
        }
        else if (i == path_waypoint_count - 1) {
            DrawSphere(next_position_vector, 0.75f, GREEN);
            DrawLine3D(previous_position_vector, next_position_vector, BLUE);
        }
        else {
            DrawSphere(next_position_vector, 0.3f, BLUE);
            DrawLine3D(previous_position_vector, next_position_vector, BLUE);
            previous_position_vector = next_position_vector;
        }
    }
}

// To be used externally
class navmesh_visualizer {
public:
    navmesh_visualizer(const dtNavMesh* navmesh);
    ~navmesh_visualizer();

    void load_navmesh();
    void draw_wireframe();

private:
    std::vector<Mesh*> meshes;
    std::vector<Model> models;

    const dtNavMesh* navmesh = nullptr;

    void draw_navmesh_tile(const dtMeshTile* tile, Mesh* mesh);
    void cleanup();
};

navmesh_visualizer::navmesh_visualizer(const dtNavMesh* navmesh)
{
    this->navmesh = navmesh;
    load_navmesh();
}

navmesh_visualizer::~navmesh_visualizer()
{
    cleanup();
    navmesh = nullptr;
}

// I think this crashes...
void navmesh_visualizer::cleanup() 
{
    for (Model& model : models) {
        UnloadModel(model);
    }
    models.clear();

    for (Mesh* mesh : meshes) {
        UnloadMesh(*mesh);
        delete mesh->vertices;
        delete mesh;
    }
    meshes.clear();
}

void navmesh_visualizer::load_navmesh() 
{
    cleanup();

    for (int i = 0; i < navmesh->getMaxTiles(); ++i) {
        const dtMeshTile* tile = navmesh->getTile(i);
        if (!tile->header) {
            continue;
        }

        Mesh* mesh = new Mesh();
        draw_navmesh_tile(tile, mesh);
        UploadMesh(mesh, false);

        Model model = LoadModelFromMesh(*mesh);

        meshes.push_back(mesh);
        models.push_back(model);
    }
}

void navmesh_visualizer::draw_navmesh_tile(const dtMeshTile* tile, Mesh* mesh)
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
                if (t[k] < p->vertCount) {
                    float* vert = &tile->verts[p->verts[t[k]] * 3];
                    vertices.push_back(vert[0]);
                    vertices.push_back(vert[1]);
                    vertices.push_back(vert[2]);
                }
                else {
                    float* vert = &tile->detailVerts[(pd->vertBase + t[k] - p->vertCount) * 3];
                    vertices.push_back(vert[0]);
                    vertices.push_back(vert[1]);
                    vertices.push_back(vert[2]);
                }
            }
        }
    }

    mesh->vertices = new float[vertices.size()];
    for (int i = 0; i < vertices.size(); ++i) {
        mesh->vertices[i] = vertices[i];
    }
    mesh->vertexCount = triangle_count * 3;
    mesh->triangleCount = triangle_count;
}

void navmesh_visualizer::draw_wireframe() 
{
    for (const Model& model : models) {
        DrawModelWires(model, Vector3{ 0.0f, 0.0f, 0.0f }, 1, DARKPURPLE);
    }

    for (Mesh* mesh : meshes) {
        for (int i = 0; i < mesh->vertexCount; ++i) {
            float* v = &mesh->vertices[3 * i];
            DrawSphere(Vector3{ v[0], v[1], v[2] }, 0.2, DARKPURPLE);
        }
    }
}

int main() 
{
    agent_params params;
    params.cell_height = 0.2f;
    params.cell_size = 0.3f;
    params.agent_height = 1.0f;
    params.agent_radius = 0.5f;
    params.max_slope = 20.0f;

    navmesh* nmesh = new navmesh(&params);
    navcontext* context = nmesh->get_context();
    InputGeom* geometry = new InputGeom();

    const char* path_to_mesh = "../content/meshes/dungeon.obj";

    geometry->load(context, path_to_mesh);
    context->dump_log("Geometry log:");

    nmesh->on_mesh_changed(geometry);
    nmesh->build();

    context->dump_log("Build log: ");

    ////////////////////////////////////////

    path* p = new path();
    p->init(nmesh);

    float start[3] = { 20.230976f, 9.998184f, 1.481956f };
    p->set_start(start);

    float end[3] = { -1.292282f, 9.998180f, -5.186989f };
    p->set_end(end);

    ///////////////////////////////////////

    const int screen_width = 2048;
    const int screen_height = 1024;

    InitWindow(screen_width, screen_height, "NAVMESH VISUALIZER");

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 50.0f, -5.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };    
    camera.up = { 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 90.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    const dtNavMesh* navmesh_internal = nmesh->get_navmesh_internal();
    navmesh_visualizer* visualizer = new navmesh_visualizer(navmesh_internal);

    //Mesh actual_mesh = { 0 }; This doesn't work???
    //actual_mesh.vertices = (float *)geometry->getMesh()->getVerts();
    //actual_mesh.normals = (float*)geometry->getMesh()->getNormals();
    //actual_mesh.vertexCount = geometry->getMesh()->getVertCount();
    //actual_mesh.indices = (unsigned short*)geometry->getMesh()->getTris();
    //actual_mesh.triangleCount = geometry->getMesh()->getTriCount();
    //UploadMesh(&actual_mesh, false);
    Model model = LoadModel(path_to_mesh);

    while (!WindowShouldClose())
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_FREE);

        if (IsKeyPressed(KEY_Z)) {
            camera.target = { 0.0f, 0.0f, 0.0f };
        }
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        DrawModel(model, Vector3{ 0.0f, 0.0f, 0.0f }, 1, Fade(GRAY, 0.25f));
        DrawModelWires(model, Vector3{ 0.0f, 0.0f, 0.0f }, 1, DARKGRAY);
        visualizer->draw_wireframe();
        draw_path_points(p);

        EndMode3D();
        
        DrawRectangle(10, 10, 320, 93, Fade(SKYBLUE, 0.5f));
        DrawRectangleLines(10, 10, 320, 93, BLUE);

        DrawText("Free camera default controls:", 20, 20, 10, BLACK);
        DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, DARKGRAY);
        DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, DARKGRAY);
        DrawText("- Z to zoom to (0, 0, 0)", 40, 80, 10, DARKGRAY);

        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}