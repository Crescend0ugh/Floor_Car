#include <DetourCommon.h>

#include "path.h"

path::path() 
{
	filter.setIncludeFlags(0xffff);
	filter.setExcludeFlags(0);

	// Not sure where these numbers come from
	half_extents[0] = 1.0f;
	half_extents[1] = 2.0f;
	half_extents[2] = 1.0f;
}

path::~path() 
{}

void path::init(class navmesh* mesh) 
{
	navmesh = mesh;
	query = navmesh->get_nav_query();
	
	recalculate();
}

void path::recalculate()
{ 
	if (!navmesh) return;

	if (is_start_pos_set) 
	{
		query->findNearestPoly(start_pos, half_extents, &filter, &start_poly_ref, nullptr);
	}
	else 
	{
		start_poly_ref = 0;
	}

	if (is_end_pos_set) 
	{
		query->findNearestPoly(end_pos, half_extents, &filter, &end_poly_ref, nullptr);
	}
	else 
	{
		end_poly_ref = 0;
	}
	
	pathfind_status = DT_FAILURE;

	if (!start_poly_ref || !end_poly_ref || !is_start_pos_set || !is_end_pos_set) 
	{
		polys_count = 0;
		path_waypoints_count = 0;
		return;
	}

	query->findPath(start_poly_ref, end_poly_ref, start_pos, end_pos, &filter, polys, &polys_count, MAX_POLYS);
	path_waypoints_count = 0;

	if (polys_count) 
	{
		float end_pos_copy[3];
		dtVcopy(end_pos_copy, end_pos);

		// In case of partial path, make sure the end point is clamped to the last polygon
		if (polys[polys_count - 1] != end_poly_ref) 
		{
			query->closestPointOnPoly(polys[polys_count - 1], end_pos, end_pos_copy, nullptr);
		}

		query->findStraightPath(start_pos, end_pos_copy, polys, polys_count,
			path_waypoints, path_waypoint_flags, path_polys, &path_waypoints_count, MAX_POLYS, DT_STRAIGHTPATH_ALL_CROSSINGS);
	}
}

void path::set_start(const float* position) 
{
	rcVcopy(start_pos, position);
	is_start_pos_set = true;
	recalculate();
}

void path::set_end(const float* position) 
{
	rcVcopy(end_pos, position);
	is_end_pos_set = true;
	recalculate();
}

const float* path::get_next_waypoint() 
{
	// We've finished traversing the path
	if (current_waypoint_id > path_waypoints_count - 1) 
	{
		return nullptr;
	}

	float goal[3];
	goal[0] = path_waypoints[3 * current_waypoint_id];
	goal[1] = path_waypoints[3 * current_waypoint_id + 1];
	goal[2] = path_waypoints[3 * current_waypoint_id + 2];
	return goal;
}

void path::increment_waypoint()
{
	if (current_waypoint_id > path_waypoints_count - 1) 
	{
		return;
	}

	current_waypoint_id++;
}

void path::reset()
{
	start_poly_ref = 0;
	end_poly_ref = 0;
	polys_count = 0;
	path_waypoints_count = 0;
	current_waypoint_id = 0;

	is_start_pos_set = false;
	is_end_pos_set = false;
}

const float* path::get_waypoint_from_id(int id) 
{
	if (id > path_waypoints_count - 1) 
	{
		return nullptr;
	}

	float waypoint[3];
	waypoint[0] = path_waypoints[3 * id];
	waypoint[1] = path_waypoints[3 * id + 1];
	waypoint[2] = path_waypoints[3 * id + 2];

	return waypoint;
}