#include "navgeometry.h"

#include <iostream>
#include <math.h>

robo::navgeometry::navgeometry():
	min_bounds{0.0f, 0.0f, 0.0f},
	max_bounds{0.0f, 0.0f, 0.0f}
{}

// Taken from MeshLoaderObj.cpp
void robo::navgeometry::compute_normals()
{
	normals.resize(triangles.size());

	for (int i = 0; i < triangles.size() / 3; i += 3)
	{
		const float* v0 = &vertices[triangles[i] * 3];
		const float* v1 = &vertices[triangles[i + 1] * 3];
		const float* v2 = &vertices[triangles[i + 2] * 3];

		float e0[3], e1[3];
		for (int j = 0; j < 3; ++j)
		{
			e0[j] = v1[j] - v0[j];
			e1[j] = v2[j] - v0[j];
		}

		float* n = &normals[i];
		n[0] = e0[1] * e1[2] - e0[2] * e1[1];
		n[1] = e0[2] * e1[0] - e0[0] * e1[2];
		n[2] = e0[0] * e1[1] - e0[1] * e1[0];

		float d = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);

		if (d > 0)
		{
			d = 1.0f / d;
			n[0] *= d;
			n[1] *= d;
			n[2] *= d;
		}
	}
}

robo::navgeometry::~navgeometry()
{
	delete chunky_tri_mesh;
}

bool robo::navgeometry::load(std::vector<float>& new_vertices, std::vector<int>& new_triangles)
{
	if (chunky_tri_mesh)
	{
		delete chunky_tri_mesh;
		chunky_tri_mesh = nullptr;
	}

	// Move rather than copy for performance
	vertices = std::move(new_vertices);
	triangles = std::move(new_triangles);

	normals.clear();
	compute_normals();

	rcCalcBounds(vertices.data(), vertices.size() / 3, min_bounds, max_bounds);

	chunky_tri_mesh = new rcChunkyTriMesh;
	if (!chunky_tri_mesh)
	{
		std::cerr <<  "Out of memory for chunky tri mesh." << std::endl;
		return false;
	}

	if (!rcCreateChunkyTriMesh(vertices.data(), triangles.data(), triangles.size() / 3, 256, chunky_tri_mesh))
	{
		std::cerr << "Failed to build chunky tri mesh." << std::endl;
		return false;
	}
	
	return true;
}

bool robo::navgeometry::has_geometry() const
{
	return vertices.size() > 0 && triangles.size() > 0 && chunky_tri_mesh != nullptr;
}