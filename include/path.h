#pragma once

#include <DetourNavMeshQuery.h>
#include <DetourStatus.h>
#include <DetourPathCorridor.h>

#include "navmesh.h"

class path 
{
	static const int MAX_POLYS = 256;

	navmesh* mesh = nullptr;
	dtNavMeshQuery* query = nullptr;

	dtStatus pathfind_status = DT_FAILURE;

	dtQueryFilter filter;

	dtPolyRef start_poly_ref = 0; // Pplygon containing start
	dtPolyRef end_poly_ref = 0;	// Polygon containing end
	dtPolyRef polys[MAX_POLYS] = { 0 }; // Array of polygons in path
	int npolys = 0; // Number of polygons in path

	float start_pos[3] = { 0.0f }; // These can be replaced with vector3 probably
	float end_pos[3] = { 0.0f };
	float half_extents[3] = { 0.0f };

	float straight_path[MAX_POLYS * 3] = { 0 }; // I'm assuming these are the vector coordinates of the straight path positions
	unsigned char straight_path_flags[MAX_POLYS] = { 0 };
	dtPolyRef straight_path_polys[MAX_POLYS] = { 0 };
	int straight_path_count = 0;

public:
	path();
	~path();

	void init(navmesh* navmesh);
	void recalculate();
	void reset();
	void update();

	void set_half_extents(const float* extents) { rcVcopy(half_extents, extents); };
};