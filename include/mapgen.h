//
// Created by Adithya Somashekhar on 10/5/25.
//

#pragma once

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

#include "vector.h"

namespace robo
{
    struct point_cloud_mesh_components
    {
        std::vector<float> vertices;
        std::vector<int> triangles;
    };

    struct point_cloud
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr points;
        uint32_t points_since_last_downsample = 0;

        point_cloud() : points(new pcl::PointCloud<pcl::PointXYZ>()) {}

        static point_cloud_mesh_components extract_mesh_components(const pcl::PolygonMesh& mesh);

        pcl::PolygonMesh reconstruct_mesh_from_points() const;

        void add_point(const vector3f& point);
        float get_adaptive_voxel_size() const;

        // 10 cm voxel grid
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_downsample(float voxel_size = 0.1f) const;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filter_navigation_box(
            const Eigen::Affine3f& robot_pose,
            float forward = 10.0f,
            float backward = 2.0f,
            float left = 5.0f,
            float right = 5.0f,
            float up = 3.0f,
            float down = 1.0f
        ) const;
    };
}