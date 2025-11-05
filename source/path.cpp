#include <DetourCommon.h>

#include "path.h"

robo::path::path() 
{
	filter.setIncludeFlags(0xffff);
	filter.setExcludeFlags(0);

	// Not sure where these numbers come from
	half_extents[0] = 1.0f;
	half_extents[1] = 2.0f;
	half_extents[2] = 1.0f;
}

void robo::path::init(class navmesh* mesh)
{
	navmesh = mesh;
	query = navmesh->nav_query;
	
	recalculate();
}

void robo::path::populate_waypoints_vector()
{
	waypoints.reserve(path_waypoints_count);

	for (int i = 0; i < path_waypoints_count; ++i)
	{
		waypoints.emplace_back(path_waypoints[3 * i], path_waypoints[3 * i + 1], path_waypoints[3 * i + 2]);
	}
}

void robo::path::recalculate()
{ 
	if (!navmesh) return;

	if (is_path_start_set) 
	{
		query->findNearestPoly(path_start, half_extents, &filter, &start_poly_ref, nullptr);
	}
	else 
	{
		start_poly_ref = 0;
	}

	if (is_path_end_set) 
	{
		query->findNearestPoly(path_end, half_extents, &filter, &end_poly_ref, nullptr);
	}
	else 
	{
		end_poly_ref = 0;
	}
	
	pathfind_status = DT_FAILURE;

	if (!start_poly_ref || !end_poly_ref || !is_path_start_set || !is_path_end_set) 
	{
		polys_count = 0;
		path_waypoints_count = 0;
		waypoints.clear();

		return;
	}

	query->findPath(start_poly_ref, end_poly_ref, path_start, path_end, &filter, polys, &polys_count, MAX_POLYS);

	path_waypoints_count = 0;
	waypoints.clear();

	if (polys_count) 
	{
		float end_pos_copy[3];
		dtVcopy(end_pos_copy, path_end);

		// In case of partial path, make sure the end point is clamped to the last polygon
		if (polys[polys_count - 1] != end_poly_ref) 
		{
			query->closestPointOnPoly(polys[polys_count - 1], path_end, end_pos_copy, nullptr);
		}

		query->findStraightPath(path_start, end_pos_copy, polys, polys_count,
			path_waypoints, path_waypoint_flags, path_polys, &path_waypoints_count, MAX_POLYS, DT_STRAIGHTPATH_ALL_CROSSINGS);

		populate_waypoints_vector();
	}
}

void robo::path::set_start(const vector3f& position)
{
	path_start[0] = position.x;
	path_start[1] = position.y;
	path_start[2] = position.z;

	// Copy
	start = position;

	is_path_start_set = true;
	recalculate();
}

void robo::path::set_end(const vector3f& position)
{
	path_end[0] = position.x;
	path_end[1] = position.y;
	path_end[2] = position.z;

	// Copy
	end = position;

	is_path_end_set = true;
	recalculate();
}

std::optional<robo::vector3f> robo::path::get_next_waypoint() const
{
	// We've finished traversing the path
	if (current_waypoint_id > path_waypoints_count - 1) 
	{
		return std::nullopt;
	}

	return waypoints[current_waypoint_id];
}

void robo::path::increment_waypoint()
{
	if (current_waypoint_id > path_waypoints_count - 1) 
	{
		return;
	}

	current_waypoint_id++;
}

void robo::path::reset()
{
	start_poly_ref = 0;
	end_poly_ref = 0;
	polys_count = 0;
	path_waypoints_count = 0;
	current_waypoint_id = 0;

	is_path_start_set = false;
	is_path_end_set = false;

	waypoints.clear();
}