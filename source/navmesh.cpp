#include "navmesh.h"
#include "math.h"

navmesh::navmesh(agent_params *params) {
	config.cs = params->cell_size;
	config.ch = params->cell_height;
	config.walkableSlopeAngle = params->max_slope;

	config.walkableHeight = (int)ceilf(params->height / config.ch);
	config.walkableClimb = (int)ceilf(params->max_climb / config.ch);
	config.walkableRadius = (int)ceilf(params->radius / config.cs);
	config.maxEdgeLen = config.walkableRadius * 8;
	config.maxSimplificationError = params->edge_max_error;

	config.minRegionArea = rcSqr(8.0f);
	config.mergeRegionArea = rcSqr(20.0f);
}

navmesh::~navmesh() {
	cleanup();

	dtFreeNavMesh(final_navmesh);
	final_navmesh = nullptr;
}

void navmesh::cleanup() {
	rcFreeHeightField(height_field);
	height_field = nullptr;

	rcFreeCompactHeightfield(compact_height_field);
	compact_height_field = nullptr;

	rcFreeContourSet(contour_set);
	contour_set = nullptr;

	rcFreePolyMesh(poly_mesh);
	poly_mesh = nullptr;

	rcFreePolyMeshDetail(poly_mesh_detail);
	poly_mesh_detail = nullptr;
}

bool navmesh::build() {
	// Check for geometry

	dtFreeNavMesh(final_navmesh);

	final_navmesh = dtAllocNavMesh();
	if (!final_navmesh)
	{
		context->log(RC_LOG_ERROR, "navmesh::build: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	dtStatus status;

	status = final_navmesh->init(&params);
	if (dtStatusFailed(status))
	{
		context->log(RC_LOG_ERROR, "navmesh::build: Could not init navmesh.");
		return false;
	}

	build_all_tiles();

	return true;
}

void navmesh::on_mesh_changed() {}

void navmesh::build_tile(const float* position) {
	if (!final_navmesh) {
		return;
	}

	// const float* min_bounds;
	// const float* max_bounds;

	const float size = tile_size * config.cs;
}

void navmesh::get_tile_pos(const float* position, int& x, int& y) {}

void navmesh::remove_tile(const float* position) {}

void navmesh::build_all_tiles() {}

void navmesh::remove_all_tiles() {}

unsigned char* navmesh::build_tile_mesh(const int tile_x, const int tile_y, const float* min_bounds, const float* max_bounds, int& data_size) {
	cleanup();

	/*
	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const int ntris = m_geom->getMesh()->getTriCount();
	const rcChunkyTriMesh* chunkyMesh = m_geom->getChunkyMesh();
	*/

	rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

	// Allocate voxel heightfield where we rasterize our input data to
	height_field = rcAllocHeightfield();
	if (!height_field) {
		context->log(RC_LOG_ERROR, "nav_mesh::build: Out of memory for height field");
		return false;
	}
	if (!rcCreateHeightfield(context, *height_field, config.width, config.height, config.bmin, config.bmax, config.cs, config.ch)) {
		context->log(RC_LOG_ERROR, "nav_mesh::build: Unable to create height field");
		return false;
	}
}