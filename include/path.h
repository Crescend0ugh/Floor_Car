#pragma once

#include <DetourNavMeshQuery.h>
#include <DetourStatus.h>
#include <DetourPathCorridor.h>

#include "navmesh.h"

namespace robo
{
	class path
	{
		static const int MAX_POLYS = 256;

		class navmesh* navmesh = nullptr;
		dtNavMeshQuery* query = nullptr;

		dtStatus pathfind_status = DT_FAILURE;

		dtQueryFilter filter;

		dtPolyRef start_poly_ref = 0; // Pplygon containing start
		dtPolyRef end_poly_ref = 0;	// Polygon containing end
		dtPolyRef polys[MAX_POLYS] = { 0 }; // Array of polygons in path
		int polys_count = 0; // Number of polygons in path

		float start_pos[3] = { 0.0f };
		float end_pos[3] = { 0.0f };
		float half_extents[3] = { 0.0f };

		float path_waypoints[MAX_POLYS * 3] = { 0 }; // Vector coordinates of the straight path points
		unsigned char path_waypoint_flags[MAX_POLYS] = { 0 }; // Flags for each point (probably unused)
		dtPolyRef path_polys[MAX_POLYS] = { 0 }; // Polygon refs that are being entered at each straight path point
		int path_waypoints_count = 0;

		int current_waypoint_id = 0;

		bool is_start_pos_set = false;
		bool is_end_pos_set = false;

	private:
		void recalculate();

	public:
		path();
		~path();

		void init(class navmesh* navmesh);
		void reset();

		void set_half_extents(const float* extents) { rcVcopy(half_extents, extents); };

		// These can be vector3's in the future
		void set_start(const float* position);
		void set_end(const float* position);

		int get_waypoint_count() { return path_waypoints_count; };
		const float* get_next_waypoint(); // Can return a vector3
		void increment_waypoint();
		const float* get_waypoint_from_id(int id);
	};
}