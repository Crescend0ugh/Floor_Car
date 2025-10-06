#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>

#include "navmesh.h"
#include "math.h"

inline unsigned int next_pow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

navmesh::navmesh(agent_params *params) 
{
	context = new navcontext();
	nav_query = dtAllocNavMeshQuery();

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

navmesh::~navmesh() 
{
	cleanup();

	dtFreeNavMeshQuery(nav_query);
	dtFreeNavMesh(mesh);
	mesh = nullptr;
}

void navmesh::cleanup()
{
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

bool navmesh::build() 
{
	if (!geometry || !geometry->getMesh())
	{
		context->log(RC_LOG_ERROR, "navmesh::build: No vertices and triangles.");
		return false;
	}

	dtFreeNavMesh(mesh);
	mesh = dtAllocNavMesh();
	if (!mesh)
	{
		context->log(RC_LOG_ERROR, "navmesh::build: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, geometry->getNavMeshBoundsMin());
	params.tileWidth = tile_size * config.cs;
	params.tileHeight = tile_size * config.cs;
	params.maxTiles = max_tiles;
	params.maxPolys = max_polys_per_tile;

	dtStatus status;

	status = mesh->init(&params);
	if (dtStatusFailed(status))
	{
		context->log(RC_LOG_ERROR, "navmesh::build: Could not init navmesh.");
		return false;
	}

	build_all_tiles();

	return true;
}

void navmesh::on_mesh_changed(InputGeom* new_geometry) 
{
	geometry = new_geometry;

	if (geometry) {
		int width = 0, height = 0;
		const float* bmin = geometry->getNavMeshBoundsMin();
		const float* bmax = geometry->getNavMeshBoundsMax();

		rcCalcGridSize(bmin, bmax, config.cs, &width, &height);

		const int ts = (int)tile_size;
		const int tile_width = (width + ts - 1) / ts;
		const int tile_height = (height + ts - 1) / ts;

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tile_bits = rcMin((int)ilog2(next_pow2(tile_width * tile_height)), 14);
		if (tile_bits > 14) tile_bits = 14;
		int poly_bits = 22 - tile_bits;

		max_tiles = 1 << tile_bits;
		max_polys_per_tile = 1 << poly_bits;
	}
	else {
		max_tiles = 0;
		max_polys_per_tile = 0;
	}

	cleanup();

	dtFreeNavMesh(mesh);
	mesh = 0;
}

void navmesh::build_tile(const float* position) 
{
	if (!geometry) return;
	if (!mesh) return;

	const float* bmin = geometry->getNavMeshBoundsMin();
	const float* bmax = geometry->getNavMeshBoundsMax();

	const float ts = tile_size * config.cs;
	const int tx = (int)((position[0] - bmin[0]) / ts);
	const int ty = (int)((position[2] - bmin[2]) / ts);

	last_built_tile_bmin[0] = bmin[0] + tx * ts;
	last_built_tile_bmin[1] = bmin[1];
	last_built_tile_bmin[2] = bmin[2] + ty * ts;

	last_built_tile_bmax[0] = bmin[0] + (tx + 1) * ts;
	last_built_tile_bmax[1] = bmax[1];
	last_built_tile_bmax[2] = bmin[2] + (ty + 1) * ts;

	context->resetLog();

	int data_size = 0;
	unsigned char* data = build_tile_mesh(tx, ty, last_built_tile_bmin, last_built_tile_bmax, data_size);

	// Remove any previous data (navmesh owns and deletes the data).
	mesh->removeTile(mesh->getTileRefAt(tx, ty, 0), 0, 0);

	// Add tile, or leave the location empty.
	if (data)
	{
		// Let the navmesh own the data.
		dtStatus status = mesh->addTile(data, data_size, DT_TILE_FREE_DATA, 0, 0);
		if (dtStatusFailed(status))
			dtFree(data);
	}
}

void navmesh::get_tile_pos(const float* position, int& tx, int& ty) 
{
	if (!geometry) return;

	const float* bmin = geometry->getNavMeshBoundsMin();

	const float ts = tile_size * config.cs;
	tx = (int)((position[0] - bmin[0]) / ts);
	ty = (int)((position[2] - bmin[2]) / ts);
}

void navmesh::remove_tile(const float* position)
{
	if (!geometry) return;
	if (!mesh) return;

	const float* bmin = geometry->getNavMeshBoundsMin();
	const float* bmax = geometry->getNavMeshBoundsMax();

	const float ts = tile_size * config.cs;
	const int tx = (int)((position[0] - bmin[0]) / ts);
	const int ty = (int)((position[2] - bmin[2]) / ts);

	last_built_tile_bmin[0] = bmin[0] + tx * ts;
	last_built_tile_bmin[1] = bmin[1];
	last_built_tile_bmin[2] = bmin[2] + ty * ts;

	last_built_tile_bmax[0] = bmin[0] + (tx + 1) * ts;
	last_built_tile_bmax[1] = bmax[1];
	last_built_tile_bmax[2] = bmin[2] + (ty + 1) * ts;

	mesh->removeTile(mesh->getTileRefAt(tx, ty, 0), 0, 0);
}

void navmesh::build_all_tiles()
{
	if (!geometry) return;
	if (!mesh) return;

	const float* bmin = geometry->getNavMeshBoundsMin();
	const float* bmax = geometry->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, config.cs, &gw, &gh);
	const int ts = (int)tile_size;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;
	const float tcs = tile_size * config.cs;

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			last_built_tile_bmin[0] = bmin[0] + x * tcs;
			last_built_tile_bmin[1] = bmin[1];
			last_built_tile_bmin[2] = bmin[2] + y * tcs;

			last_built_tile_bmax[0] = bmin[0] + (x + 1) * tcs;
			last_built_tile_bmax[1] = bmax[1];
			last_built_tile_bmax[2] = bmin[2] + (y + 1) * tcs;

			int data_size = 0;
			unsigned char* data = build_tile_mesh(x, y, last_built_tile_bmin, last_built_tile_bmax, data_size);
			if (data)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				mesh->removeTile(mesh->getTileRefAt(x, y, 0), 0, 0);
				// Let the navmesh own the data.
				dtStatus status = mesh->addTile(data, data_size, DT_TILE_FREE_DATA, 0, 0);
				if (dtStatusFailed(status))
					dtFree(data);
			}
		}
	}
}

void navmesh::remove_all_tiles()
{
	if (!geometry || !mesh)
		return;

	const float* bmin = geometry->getNavMeshBoundsMin();
	const float* bmax = geometry->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, config.cs, &gw, &gh);
	const int ts = (int)tile_size;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			mesh->removeTile(mesh->getTileRefAt(x, y, 0), 0, 0);
}

unsigned char* navmesh::build_tile_mesh(const int tx, const int ty, const float* bmin, const float* bmax, int& data_size) 
{
	if (!geometry || !geometry->getMesh() || !geometry->getChunkyMesh())
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Input mesh is not specified.");
		return nullptr;
	}

	cleanup();

	const float* verts = geometry->getMesh()->getVerts();
	const int nverts = geometry->getMesh()->getVertCount();
	const int ntris = geometry->getMesh()->getTriCount();
	const rcChunkyTriMesh* chunky_mesh = geometry->getChunkyMesh();


	// TODO: Adjust these as needed. Also, most of these should be set in the constructor, probably.
	config.maxVertsPerPoly = 6.0f;
	config.tileSize = tile_size;
	config.borderSize = config.walkableRadius + 3;
	config.width = config.tileSize + config.borderSize * 2;
	config.height = config.tileSize + config.borderSize * 2;
	config.detailSampleDist = 6.0f;
	config.detailSampleMaxError = 1.0f;

	rcVcopy(config.bmin, bmin);
	rcVcopy(config.bmax, bmax);
	config.bmin[0] -= config.borderSize * config.cs;
	config.bmin[2] -= config.borderSize * config.cs;
	config.bmax[0] += config.borderSize * config.cs;
	config.bmax[2] += config.borderSize * config.cs;

	height_field = rcAllocHeightfield();
	if (!height_field) {
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Out of memory 'height_field'.");
		return 0;
	}
	if (!rcCreateHeightfield(context, *height_field, config.width, config.height, config.bmin, config.bmax, config.cs, config.ch))
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Could not create solid heightfield.");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	tri_areas = new unsigned char[chunky_mesh->maxTrisPerChunk];
	if (!tri_areas)
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Out of memory 'tri_areas' (%d).", chunky_mesh->maxTrisPerChunk);
		return nullptr;
	}

	float tbmin[2], tbmax[2];
	tbmin[0] = config.bmin[0];
	tbmin[1] = config.bmin[2];
	tbmax[0] = config.bmax[0];
	tbmax[1] = config.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunky_mesh, tbmin, tbmax, cid, 512);
	if (!ncid)
		return nullptr;

	tile_tri_count = 0;

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunky_mesh->nodes[cid[i]];
		const int* ctris = &chunky_mesh->tris[node.i * 3];
		const int nctris = node.n;

		tile_tri_count += nctris;

		memset(tri_areas, 0, nctris * sizeof(unsigned char));
		rcMarkWalkableTriangles(context, config.walkableSlopeAngle,
			verts, nverts, ctris, nctris, tri_areas);

		if (!rcRasterizeTriangles(context, verts, nverts, ctris, tri_areas, nctris, *height_field, config.walkableClimb))
			return nullptr;
	}

	delete[] tri_areas;
	tri_areas = nullptr;

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(context, config.walkableClimb, *height_field);
	rcFilterLedgeSpans(context, config.walkableHeight, config.walkableClimb, *height_field);
	rcFilterWalkableLowHeightSpans(context, config.walkableHeight, *height_field);


	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	compact_height_field = rcAllocCompactHeightfield();
	if (!compact_height_field)
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Out of memory 'chf'.");
		return nullptr;
	}
	if (!rcBuildCompactHeightfield(context, config.walkableHeight, config.walkableClimb, *height_field, *compact_height_field))
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Could not build compact data.");
		return nullptr;
	}

	rcFreeHeightField(height_field);
	height_field = nullptr;

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(context, config.walkableRadius, *compact_height_field))
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Could not erode.");
		return nullptr;
	}

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	// Test: Monotone

	// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
	if (!rcBuildRegionsMonotone(context, *compact_height_field, config.borderSize, config.minRegionArea, config.mergeRegionArea))
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Could not build monotone regions.");
		return nullptr;
	}

	// Create contours.
	contour_set = rcAllocContourSet();
	if (!contour_set)
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Out of memory 'contour_set'.");
		return nullptr;
	}
	if (!rcBuildContours(context, *compact_height_field, config.maxSimplificationError, config.maxEdgeLen, *contour_set))
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Could not create contours.");
		return nullptr;
	}

	if (contour_set->nconts == 0)
	{
		return nullptr;
	}

	// Build polygon navmesh from the contours.
	poly_mesh = rcAllocPolyMesh();
	if (!poly_mesh)
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Out of memory 'poly_mesh'.");
		return nullptr;
	}
	if (!rcBuildPolyMesh(context, *contour_set, config.maxVertsPerPoly, *poly_mesh))
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Could not triangulate contours.");
		return nullptr;
	}

	// Build detail mesh.
	poly_mesh_detail = rcAllocPolyMeshDetail();
	if (!poly_mesh_detail)
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Out of memory 'dmesh'.");
		return nullptr;
	}

	if (!rcBuildPolyMeshDetail(context, *poly_mesh, *compact_height_field,
		config.detailSampleDist, config.detailSampleMaxError,
		*poly_mesh_detail))
	{
		context->log(RC_LOG_ERROR, "navmesh::build_tile_mesh: Could build polymesh detail.");
		return nullptr;
	}

	rcFreeCompactHeightfield(compact_height_field);
	compact_height_field = nullptr;
	rcFreeContourSet(contour_set);
	contour_set = nullptr;


	unsigned char* nav_data = nullptr;
	int nav_data_size = 0;
	if (config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (poly_mesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			context->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", poly_mesh->nverts, 0xffff);
			return 0;
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = poly_mesh->verts;
		params.vertCount = poly_mesh->nverts;
		params.polys = poly_mesh->polys;
		params.polyAreas = poly_mesh->areas;
		params.polyFlags = poly_mesh->flags;
		params.polyCount = poly_mesh->npolys;
		params.nvp = poly_mesh->nvp;
		params.detailMeshes = poly_mesh_detail->meshes;
		params.detailVerts = poly_mesh_detail->verts;
		params.detailVertsCount = poly_mesh_detail->nverts;
		params.detailTris = poly_mesh_detail->tris;
		params.detailTriCount = poly_mesh_detail->ntris;
		params.offMeshConVerts = geometry->getOffMeshConnectionVerts();
		params.offMeshConRad = geometry->getOffMeshConnectionRads();
		params.offMeshConDir = geometry->getOffMeshConnectionDirs();
		params.offMeshConAreas = geometry->getOffMeshConnectionAreas();
		params.offMeshConFlags = geometry->getOffMeshConnectionFlags();
		params.offMeshConUserID = geometry->getOffMeshConnectionId();
		params.offMeshConCount = geometry->getOffMeshConnectionCount();
		params.walkableHeight = config.walkableHeight * config.ch;
		params.walkableRadius = config.walkableRadius;
		params.walkableClimb = config.walkableClimb;
		params.tileX = tx;
		params.tileY = ty;
		params.tileLayer = 0;
		rcVcopy(params.bmin, poly_mesh->bmin);
		rcVcopy(params.bmax, poly_mesh->bmax);
		params.cs = config.cs;
		params.ch = config.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &nav_data, &nav_data_size))
		{
			context->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return nullptr;
		}
	}

	data_size = nav_data_size;
	context->log(RC_LOG_PROGRESS, "data size: %d", data_size);

	return nullptr;
}