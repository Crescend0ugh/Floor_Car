#pragma once

#include <Recast.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <InputGeom.h>

#include "navcontext.h"

struct agent_params 
{
    float radius;
    float height;
    float max_slope; // Maximum slope angle we can move up
    float max_climb = 0.0f; // Maximum height of obstacles we can step up (probably 0?)
    float edge_max_error = 1.3f; // Suggested by Docs

    float cell_size;
    float cell_height;
};

class navmesh 
{
    rcConfig config;
    navcontext* context = nullptr;

    rcHeightfield* height_field = nullptr;
    rcCompactHeightfield* compact_height_field = nullptr;
    rcContourSet* contour_set = nullptr;
    rcPolyMesh* poly_mesh = nullptr;
    rcPolyMeshDetail* poly_mesh_detail = nullptr;
    dtNavMesh* navmesh_internal  = nullptr;
    dtNavMeshQuery* nav_query = nullptr;

    InputGeom* geometry = nullptr;

    unsigned char* tri_areas = nullptr;

    float tile_size = 30.0f;
    int tile_tri_count = 0;
    int max_tiles = 0;
    int max_polys_per_tile = 0;

    float last_built_tile_bmin[3] = { 0.0f };
    float last_built_tile_bmax[3] = { 0.0f };

private:
    void cleanup();

    void build_tile(const float* position);
    void get_tile_pos(const float* position, int& tx, int& ty);
    void remove_tile(const float* position);
    void build_all_tiles();
    void remove_all_tiles();
    unsigned char* build_tile_mesh(const int tx, const int ty, const float* bmin, const float* bmax, int& data_size);

public:
    navmesh(agent_params* params);
    ~navmesh();

    bool build();
    void on_mesh_changed(InputGeom* new_geometry);
    navcontext* get_context() { return context; };
    dtNavMesh* get_navmesh_internal() { return navmesh_internal; };
    dtNavMeshQuery* get_nav_query() { return nav_query; };
};