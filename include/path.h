/*
	path.h

	Generates a path (array of points) through a navmesh, given start and end positions.

	The path is automatically recalculated and updated when the start and end positions change.
*/

#pragma once

#include "DetourNavMeshQuery.h"
#include "DetourStatus.h"
#include "DetourPathCorridor.h"

#include "navmesh.h"
#include "vector.h"

#include <optional>

namespace robo
{
	class path
	{
	private:
		static const int MAX_POLYS = 256;

		class navmesh* navmesh = nullptr;
		dtNavMeshQuery* query = nullptr;

		dtStatus pathfind_status = DT_FAILURE;

		dtQueryFilter filter;

		dtPolyRef start_poly_ref = 0; // Pplygon containing start
		dtPolyRef end_poly_ref = 0;	// Polygon containing end
		dtPolyRef polys[MAX_POLYS] = { 0 }; // Array of polygons in path
		int polys_count = 0; // Number of polygons in path

		float half_extents[3] = { 0.0f };

		float path_waypoints[MAX_POLYS * 3] = { 0 }; // Vector coordinates of the straight path points
		unsigned char path_waypoint_flags[MAX_POLYS] = { 0 }; // Flags for each point (probably unused)
		dtPolyRef path_polys[MAX_POLYS] = { 0 }; // Polygon refs that are being entered at each straight path point
		int path_waypoints_count = 0;

		bool is_path_start_set = false;
		bool is_path_end_set = false;

		// Float arrays for Recast
		float path_start[3] = { 0.0f };
		float path_end[3] = { 0.0f };

		void populate_waypoints_vector();
		void recalculate();

	public:
		// Outward-facing for: path_waypoints, path_start, and path_end
		std::vector<vector3f> waypoints;
		vector3f start;
		vector3f end;
		int current_waypoint_id = 0; // For external incrementing

		path();

		void init(class navmesh* navmesh);
		void reset();

		void set_half_extents(const float* extents) { rcVcopy(half_extents, extents); };

		// These can be vector3's in the future
		void set_start(const vector3f& position);
		void set_end(const vector3f& position);

		std::optional<vector3f> get_next_waypoint() const;
		void increment_waypoint();
	};
}