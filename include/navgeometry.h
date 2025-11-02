#pragma once

#include "Recast.h"
#include "ChunkyTriMesh.h"

#include <vector>

namespace robo
{
	class navgeometry
	{
	private:
		void compute_normals();

	public:
		std::vector<float> vertices;
		std::vector<float> normals;
		std::vector<int> triangles;

		float min_bounds[3];
		float max_bounds[3];
		rcChunkyTriMesh* chunky_tri_mesh = nullptr;

		navgeometry();
		~navgeometry();

		bool load(std::vector<float>& new_vertices, std::vector<int>& new_triangles);
		bool has_geometry() const;
	};
}