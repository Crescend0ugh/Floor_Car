#include <DetourCommon.h>

#include "path.h"

path::path() 
{
	filter.setExcludeFlags(0);

	// Not sure where these numbers come from
	half_extents[0] = 2.0f;
	half_extents[1] = 4.0f;
	half_extents[2] = 2.0f;
}

path::~path() 
{}

void path::init(navmesh* navmesh) 
{
	mesh = navmesh;
	query = navmesh->get_nav_query();

	recalculate();
}

// Using sliced paths, which are slow but try extra hard to find a way to the target
void path::recalculate()
{ 
	if (!mesh) return;

	query->findNearestPoly(start_pos, half_extents, &filter, &start_poly_ref, nullptr);
	query->findNearestPoly(end_pos, half_extents, &filter, &end_poly_ref, nullptr);
	
	pathfind_status = DT_FAILURE;

	npolys = 0;

	if (start_poly_ref && end_poly_ref) {
		pathfind_status = query->initSlicedFindPath(start_poly_ref, end_poly_ref, start_pos, end_pos, &filter, DT_FINDPATH_ANY_ANGLE);
	}
}

// To be called in an update loop
void path::update()
{
	if (dtStatusInProgress(pathfind_status)) {
		pathfind_status = query->updateSlicedFindPath(1, 0);
	}
	if (dtStatusSucceed(pathfind_status)) {
		query->finalizeSlicedFindPath(polys, &npolys, MAX_POLYS);
		straight_path_count = 0;
		
		if (npolys) {
			float end_pos_copy[3];
			dtVcopy(end_pos_copy, end_pos);

			// In case of partial path, make sure the end point is clamped to the last polygon
			if (polys[npolys - 1] != end_poly_ref) {
				query->closestPointOnPoly(polys[npolys - 1], end_pos, end_pos_copy, nullptr);
			}

			query->findStraightPath(start_pos, end_pos, polys, npolys,
				straight_path, straight_path_flags, straight_path_polys, &straight_path_count, MAX_POLYS, DT_STRAIGHTPATH_ALL_CROSSINGS);
		}

		pathfind_status = DT_FAILURE;
	}
}

void path::reset()
{
	start_poly_ref = 0;
	end_poly_ref = 0;
	npolys = 0;
	straight_path_count = 0;
}