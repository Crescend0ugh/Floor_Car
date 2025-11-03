/*
    navmesh.h

    Interfaces Recast Navigation to construct a navigation mesh.
    Defines agent parameters.

    The class does not provide any tools or functionality for traversing the underlying navigation mesh. See path.h.

    Use navgeometry to pass in the vertices, triangles, and normals of the world mesh.
*/

#pragma once

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "InputGeom.h"

#include "navgeometry.h"

namespace robo
{
    // Default values for all fields were chosen arbitrarily and should definitely be set!
    struct navigation_params
    {
        // If the agent were a circle, this would be its radius, in meters
        float agent_radius = 0.25f;

        // Height in meters
        float agent_height = 0.25f;

        // Maximum slope angle (0, 90 deg) we can move up
        float max_slope = 0.0f;

        // Maximum height of obstacles we can step over (probably 0?)
        float max_climb = 0.0f;

        // x and z size of each cell, in meters (NEEDS TUNING)
        float cell_size = 20.0f;

        // y height of each cell, in meters (NEEDS TUNING)
        float cell_height = 10.0f;

        // Size of the tiles in voxels (one tile will cover tile_size squared voxels)
        float tile_size = 20.0f;
        
        // Edge max error in voxels
        float edge_max_error = 1.3f;

        // Edge max length in world units
        float edge_max_len = 0.5f;

        float border_offset = 3.0f;

        // Detail sample distance in voxels
        float detail_sample_dist = 6.0f;

        // Detail sample max error in voxel heights.
        float detail_sample_max_error = 1.0f;
    };

    class navmesh
    {
        rcConfig config;
        rcContext* context = nullptr;

        rcHeightfield* height_field = nullptr;
        rcCompactHeightfield* compact_height_field = nullptr;
        rcContourSet* contour_set = nullptr;
        rcPolyMesh* poly_mesh = nullptr;
        rcPolyMeshDetail* poly_mesh_detail = nullptr;
        dtNavMesh* navmesh_internal = nullptr;
        dtNavMeshQuery* nav_query = nullptr;

        navgeometry* geometry = nullptr;

        unsigned char* tri_areas = nullptr;

        float tile_size = 20.0f;
        int tile_tri_count = 0;
        int max_tiles = 0;
        int max_polys_per_tile = 0;

        float last_built_tile_bmin[3] = { 0.0f };
        float last_built_tile_bmax[3] = { 0.0f };

    private:
        void cleanup();

        void build_tile(const float* position);
        void remove_tile(const float* position);
        void build_all_tiles();
        void remove_all_tiles();
        unsigned char* build_tile_mesh(const int tx, const int ty, const float* bmin, const float* bmax, int& data_size);

    public:
        navmesh(const navigation_params& params);
        ~navmesh();

        bool build();
        void on_mesh_changed(navgeometry* new_geometry);
        void get_tile_pos(const float* position, int& tx, int& ty);

        rcContext* get_context() { return context; };
        dtNavMesh* get_navmesh_internal() { return navmesh_internal; };
        dtNavMeshQuery* get_nav_query() { return nav_query; };
    };
}
