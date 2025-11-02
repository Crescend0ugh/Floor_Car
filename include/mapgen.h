//
// Created by Adithya Somashekhar on 10/5/25.
//

#pragma once

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/conversions.h>

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
        point_cloud() : points(new pcl::PointCloud<pcl::PointXYZ>()) {}

        static point_cloud_mesh_components extract_mesh_components(const pcl::PolygonMesh& mesh);

        pcl::PolygonMesh reconstruct_mesh_from_points() const;

        void add_point(const vector3f& point);

        pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    };
}