//
// Created by Adithya Somashekhar on 11/1/25.
//

#include "mapgen.h"

/*

*/
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

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    
#if 0
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    pcl::PolygonMesh triangles;


    // Set the maximum distance between connected points (maximum edge length)

    gp3.setSearchRadius(0.5);


    // Set typical values for the parameters

    gp3.setMu(2.5);

    gp3.setMaximumNearestNeighbors(100);

    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees

    gp3.setMinimumAngle(M_PI / 18); // 10 degrees

    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees

    gp3.setNormalConsistency(false);
    gp3.setConsistentVertexOrdering(false);


    // Get result

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
#endif

    pcl::PolygonMesh triangles;

    // Poisson reconstruction ensures there's no holes/gaps in the mesh
    // Recast requires that meshes have no holes
    pcl::Poisson<pcl::PointNormal> poisson;

    poisson.setSearchMethod(tree2);
    poisson.setDepth(8);
    poisson.setSolverDivide(8);
    poisson.setSamplesPerNode(1.0); // Typical values
    poisson.setPointWeight(0.5);
    poisson.setInputCloud(cloud_with_normals);
    poisson.reconstruct(triangles);

    return triangles;
}

robo::point_cloud_mesh_components robo::point_cloud::extract_mesh_components(const pcl::PolygonMesh& mesh)
{
    std::vector<float> vertices; // [x, y, z, x, y, z, ...]
    std::vector<int> triangles; // [vert_idx1, vert_idx2, vert_idx3, ...]

    // Convert to XYZ point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    for (const auto& p : cloud)
    {
        // Vertices [x, y, z]
        vertices.push_back(p.x);
        vertices.push_back(p.y);
        vertices.push_back(p.z);
    }

    for (const auto& polygon : mesh.polygons) 
    {
        if (polygon.vertices.size() == 3) 
        {
            triangles.push_back(polygon.vertices[0]);
            triangles.push_back(polygon.vertices[1]);
            triangles.push_back(polygon.vertices[2]);
        }
    }

    return point_cloud_mesh_components
    {
        .vertices = vertices,
        .triangles = triangles,
    };
}

void robo::point_cloud::add_point(const robo::vector3f &point)
{
    points->push_back({point.x, point.y, point.z});
}
