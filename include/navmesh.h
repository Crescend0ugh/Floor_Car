#pragma once

#include <Recast.h>
#include <DetourNavMesh.h>

struct agent_params {
    float radius;
    float height;
    float max_slope; // Maximum slope angle we can move up
    float max_climb = 0.0f; // Maximum height of obstacles we can step up (probably 0?)
    float edge_max_error = 1.3f; // Suggested by Docs

    float cell_size;
    float cell_height;
};

class navmesh {
    private:
        rcConfig config;
        rcContext* context = nullptr;

        rcHeightfield* height_field = nullptr;
        rcCompactHeightfield* compact_height_field = nullptr;
        rcContourSet* contour_set = nullptr;
        rcPolyMesh* poly_mesh = nullptr;
        rcPolyMeshDetail* poly_mesh_detail = nullptr;

        dtNavMesh* final_navmesh = nullptr;

        float tile_size = 30.0f;

        void cleanup();

        void build_tile(const float* position);
        void get_tile_pos(const float* position, int& x, int& y);
        void remove_tile(const float* position);
        void build_all_tiles();
        void remove_all_tiles();
        unsigned char* build_tile_mesh(const int tile_x, const int tile_y, const float* min_bounds, const float* max_bounds, int& data_size);
    
    public:
        navmesh(agent_params*params);
        ~navmesh();

        bool build();
        void on_mesh_changed();
};