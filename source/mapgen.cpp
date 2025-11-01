//
// Created by Adithya Somashekhar on 11/1/25.
//

#include "mapgen.h"

pcl::PolygonMesh robo::point_cloud::reconstruct_mesh_from_points() const
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (points);
    n.setInputCloud (points);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*points, *normals, *cloud_with_normals);

    // Create search tree*

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    // Initialize objects

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    pcl::PolygonMesh triangles;


    // Set the maximum distance between connected points (maximum edge length)

    gp3.setSearchRadius (0.025);


    // Set typical values for the parameters

    gp3.setMu (2.5);

    gp3.setMaximumNearestNeighbors (100);

    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees

    gp3.setMinimumAngle(M_PI/18); // 10 degrees

    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees

    gp3.setNormalConsistency(false);


    // Get result

    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    return triangles;
}

void robo::point_cloud::add_point(const robo::vector3f &point)
{
    points->push_back({point.x, point.y, point.z});
}
