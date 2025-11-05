#if defined(_WIN32)           
#define NOGDI         // Excludes GDI APIs
#define NOSERVICE     // Excludes Service Controller APIs
#define NOKANJI       // Excludes Kanji support stuff
#define NOHELP        // Excludes Help engine interface
#define NOPROFILER    // Excludes Profiler interface
#endif

#pragma once

#include "raylib.h"

#include "vector.h"

#include <vector>
#include <cstdint>

namespace ui
{
	class point_cloud_visualizer
	{
	private:
		Mesh point_cloud_mesh;
		Model point_cloud_model;

		Mesh reconstructed_mesh;
		std::vector<robo::vector3f> points;

		bool needs_update = false;
		
		void update_mesh();

	public:
		point_cloud_visualizer();
		~point_cloud_visualizer();

		void draw();
		void clear();
		void set_points(const std::vector<robo::vector3f>& new_points);
		void add_points(const std::vector<robo::vector3f>& delta_points);

		void set_triangles(const std::vector<uint32_t>& triangle_indices);
	};
}