/*
	navgeometry.h

	Stores the mesh components in the format required by Recast Navigation to compute navigation meshes.

	Load the vertices and triangle indices returned by mesh reconstruction into an instance of navgeometry
	to build the navmesh.
*/

#pragma once

#include "Recast.h"
#include "ChunkyTriMesh.h"

#include "vector.h"

#include <vector>
#include <optional>

namespace robo
{
	class navgeometry
	{
	private:
		// Set by set_navmesh_min_bounds and set_navmesh_max_bounds
		float custom_min_bounds[3];
		float custom_max_bounds[3];

		bool custom_min_bounds_set = false;
		bool custom_max_bounds_set = false;

		void compute_normals();

	public:
		std::vector<float> vertices;
		std::vector<float> normals;
		std::vector<int> triangles;

		// Bounds of the entire mesh
		float min_bounds[3];
		float max_bounds[3];

		rcChunkyTriMesh* chunky_tri_mesh = nullptr; // TODO: If we switch to solo meshes, we can remove this

		navgeometry();
		~navgeometry();

		bool load(std::vector<float>& new_vertices, std::vector<int>& new_triangles);
		bool has_geometry() const;
		const float* get_min_bounds();
		const float* get_max_bounds();

		// For specifying a bounding box within which to generate the navigation mesh
		void set_navmesh_min_bounds(const std::optional<vector3f>& bmin);
		void set_navmesh_max_bounds(const std::optional<vector3f>& bmax);
	};
}