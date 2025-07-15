#ifndef ARCHITECTURE_H
#define ARCHITECTURE_H

#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


#include <Eigen/Dense>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>

#include <open3d/Open3D.h>
#include "common.h"

// CGAL typedefs
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point2 = K::Point_2;
using Segment2 = K::Segment_2;
using Polygon2 = CGAL::Polygon_2<K>;
using Vb = CGAL::Alpha_shape_vertex_base_2<K>;
using Fb = CGAL::Alpha_shape_face_base_2<K>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using Delaunay2 = CGAL::Delaunay_triangulation_2<K, Tds>;
using AlphaShape2 = CGAL::Alpha_shape_2<Delaunay2>;
using Voronoi_diagram_2 = CGAL::Voronoi_diagram_2<Delaunay2, CGAL::Delaunay_triangulation_adaptation_traits_2<Delaunay2>, CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<Delaunay2>>;


// HELPER STRUCTURES .......................................................................................................
// Structure to hold the result
struct CircleFitResult {
    std::vector<Eigen::Vector3d> centers;
    std::vector<double> radii;
    std::vector<Eigen::Vector3d> normals;
    std::vector<std::vector<Eigen::Vector3d>> boundaries;
};

struct RankedCandidate {
    Eigen::Vector3d center; // Circle center
    double radius;          // Circle radius
    Eigen::Vector3d normal; // Circle normal
    double roughness;       // Standard deviation of projected points on XZ plane
    double relief;          // Max Z - Min Z of projected points
    double point_density;   // Points per unit area in XY projection
    double slope_degrees;   // Slope angle of the circle plane in degrees
    double score;           // Weighted score for ranking
    int rank;               // Rank (1-based, lower is better)
};


struct sequentialOverlapResult {
    std::vector<CellSlope> cell_slopes;             // Slopes and metadata for valid cells
    pcl::PointIndices inlier_indices;               // Indices of inlier points in input cloud
    std::vector<pcl::PointIndices> cluster_indices; // Single entry with inlier indices for visualization
};

// Structure to hold slope and cell index
struct CellSlope {
    int index;
    double slope;
    Eigen::Vector3f centroid;
};


// VISUALIZATION RELATED FUNCTIONS ......................................................................................................
// Function to visualize clustered patches with different colors
void visualize_clustered_patches(const PointCloudPcl& inlier_cloud, 
    const std::vector<pcl::PointIndices>& cluster_indices) {
// Initialize PCL visualizer
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Clustered Patches"));
viewer->setBackgroundColor(1, 1, 1); // Black background

// Random number generator for colors
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dis(0, 255);

// Process each cluster
for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
// Create a point cloud for the current cluster
PointCloudPcl cluster_cloud(new pcl::PointCloud<PointPcl>);
for (const auto& idx : cluster_indices[cluster_idx].indices) {
if (idx >= 0 && idx < inlier_cloud->size()) {
cluster_cloud->points.push_back(inlier_cloud->points[idx]);
}
}
cluster_cloud->width = cluster_cloud->points.size();
cluster_cloud->height = 1;

// Generate a random color for the cluster
int r = dis(gen);
int g = dis(gen);
int b = dis(gen);

// Add point cloud to the viewer with a unique ID
std::string cloud_id = "cluster_" + std::to_string(cluster_idx);
pcl::visualization::PointCloudColorHandlerCustom<PointPcl> color_handler(cluster_cloud, r, g, b);
viewer->addPointCloud<PointPcl>(cluster_cloud, color_handler, cloud_id);
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_id);
}

// Add coordinate system for reference
viewer->addCoordinateSystem(1.0, "reference");

// Spin the viewer until closed
std::cout << "Displaying " << cluster_indices.size() << " clusters. Close the viewer to continue..." << std::endl;
viewer->spin();
}


void visualizeRankedCandidates(
    const PointCloudPcl& input_cloud,
    const CircleFitResult& ranked_result,
    const std::vector<pcl::PointIndices>& cluster_indices = {})
{
    // Step 1: Validate inputs
    if (!input_cloud || input_cloud->empty()) {
        std::cerr << "Error: Input point cloud is null or empty!" << std::endl;
        return;
    }
    if (ranked_result.centers.empty()) {
        std::cerr << "Error: Ranked CircleFitResult has no circles!" << std::endl;
        return;
    }

    // Step 2: Initialize PCL visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Ranked Candidates Visualization"));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->setSize(1280, 720);

    // Step 3: Compute bounding box and center
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;
    double min_z = min_x, max_z = max_x;
    size_t valid_points = 0;
    for (const auto& point : input_cloud->points) {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
            min_x = std::min(min_x, static_cast<double>(point.x));
            max_x = std::max(max_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            max_y = std::max(max_y, static_cast<double>(point.y));
            min_z = std::min(min_z, static_cast<double>(point.z));
            max_z = std::max(max_z, static_cast<double>(point.z));
            valid_points++;
        }
    }
    if (valid_points == 0) {
        std::cerr << "Error: No valid points in input cloud!" << std::endl;
        return;
    }
    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;
    double center_z = (min_z + max_z) / 2.0;

    // Step 4: Create colored point cloud (input: red, cluster indices: green)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->points.resize(input_cloud->size());
    colored_cloud->width = input_cloud->size();
    colored_cloud->height = 1;
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        if (!std::isnan(input_cloud->points[i].x) &&
            !std::isnan(input_cloud->points[i].y) &&
            !std::isnan(input_cloud->points[i].z)) {
            colored_cloud->points[i].x = input_cloud->points[i].x - center_x;
            colored_cloud->points[i].y = input_cloud->points[i].y - center_y;
            colored_cloud->points[i].z = input_cloud->points[i].z - center_z;
            colored_cloud->points[i].r = 255; // Red (default for input cloud)
            colored_cloud->points[i].g = 0;
            colored_cloud->points[i].b = 0;
        } else {
            colored_cloud->points[i].x = 0;
            colored_cloud->points[i].y = 0;
            colored_cloud->points[i].z = 0;
            colored_cloud->points[i].r = 0;
            colored_cloud->points[i].g = 0;
            colored_cloud->points[i].b = 0;
        }
    }

    // Modified: Mark points in cluster_indices as green
    if (!cluster_indices.empty()) {
        for (const auto& indices : cluster_indices) {
            for (const auto& idx : indices.indices) {
                if (static_cast<size_t>(idx) < colored_cloud->size()) {
                    colored_cloud->points[idx].r = 0;
                    colored_cloud->points[idx].g = 255; // Green
                    colored_cloud->points[idx].b = 0;
                }
            }
        }
    }

    // Step 5: Save colored cloud for debugging
    if (pcl::io::savePCDFileASCII("colored_ranked.pcd", *colored_cloud) == 0) {
        std::cout << "Saved colored point cloud to colored_ranked.pcd" << std::endl;
    }

    // Step 6: Add colored cloud to viewer
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "colored_cloud"); // Point size 3 for clarity

    // Step 7: Visualize circles and rank numbers
    const double z_offset = -1.0; // Offset circles 1m along normal
    Eigen::Vector3d viewer_dir(0, 0, 1);
    for (size_t i = 0; i < ranked_result.centers.size(); ++i) {
        const auto& center = ranked_result.centers[i];
        double radius = ranked_result.radii[i];
        auto normal = ranked_result.normals[i];
        std::string circle_id = "circle_" + std::to_string(i);

        // Ensure normal points toward viewer
        if (normal.dot(viewer_dir) < 0) {
            normal = -normal;
        }

        // Offset center along normal
        Eigen::Vector3d offset_center = center + z_offset * normal.normalized();

        // Compute basis vectors for circle plane
        Eigen::Vector3d u = normal.cross(Eigen::Vector3d(1, 0, 0)).normalized();
        if (u.norm() < 0.1) u = normal.cross(Eigen::Vector3d(0, 1, 0)).normalized();
        Eigen::Vector3d v = normal.cross(u).normalized();

        // Create circle points
        pcl::PointCloud<pcl::PointXYZ>::Ptr circle_points(new pcl::PointCloud<pcl::PointXYZ>);
        const int num_segments = 36;
        for (int j = 0; j < num_segments; ++j) {
            double theta = 2.0 * M_PI * j / num_segments;
            Eigen::Vector3d pt = offset_center + (u * std::cos(theta) + v * std::sin(theta)) * radius;
            circle_points->points.emplace_back(pt[0] - center_x, pt[1] - center_y, pt[2] - center_z);
        }

        // Draw wireframe
        for (int j = 0; j < num_segments; ++j) {
            viewer->addLine(
                circle_points->points[j],
                circle_points->points[(j + 1) % num_segments],
                0.0, 0.0, 1.0, // Blue
                circle_id + "_line_" + std::to_string(j)
            );
        }

        // Add filled polygon
        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : circle_points->points) {
            polygon_cloud->points.push_back(pt);
        }
        std::vector<pcl::Vertices> polygons;
        pcl::Vertices poly;
        for (int j = 0; j < num_segments; ++j) {
            poly.vertices.push_back(j);
        }
        polygons.push_back(poly);
        viewer->addPolygonMesh<pcl::PointXYZ>(polygon_cloud, polygons, circle_id + "_filled");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, circle_id + "_filled");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, circle_id + "_filled");

        // Add rank number (dark blue, proportional to radius)
        double text_scale = radius * 0.5;
        viewer->addText3D(
            std::to_string(i + 1),
            pcl::PointXYZ(offset_center[0] - center_x, offset_center[1] - center_y, offset_center[2] - center_z),
            text_scale,
            0.0, 0.0, 0.5, // Dark blue
            circle_id + "_rank"
        );
    }

    // Step 8: Set camera and coordinate system
    double extent = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
    double dist = extent * 2.0;
    if (dist < 1e-6) dist = 10.0;
    viewer->setCameraPosition(0, 0, dist, 0, 0, 0, 0, 1, 0);
    double coord_scale = extent / 10.0;
    if (coord_scale < 0.1) coord_scale = 0.1;
    if (coord_scale > 10.0) coord_scale = 10.0;
    viewer->addCoordinateSystem(coord_scale);
    viewer->initCameraParameters();

    // Step 9: Print debug info
    std::cout << "Starting visualization..."
              << "Original bounding box: "
              << "x=[" << min_x << ", " << max_x << "], "
              << "y=[" << min_y << ", " << max_y << "], "
              << "z=[" << min_z << ", " << max_z << "]"
              << ", Center: (" << center_x << ", " << center_y << ", " << center_z << ")"
              << ", Camera distance: " << dist
              << ", Visualizing " << ranked_result.centers.size() << " circles." << std::endl;

    // Step 10: Spin viewer (quit with 'q')
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    viewer->close();
    std::cout << "Visualization closed." << std::endl;
}

void visualizeCircles(const PointCloudPcl& merged_cloud, const CircleFitResult& circle_result) {
    if (merged_cloud->empty()) {
        std::cerr << "Error: Merged point cloud is empty!" << std::endl;
        return;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Circle Fitting Visualization"));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->setSize(1280, 720);

    double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;
    double min_z = min_x, max_z = max_x;

    for (const auto& pt : *merged_cloud) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            min_x = std::min(min_x, (double)pt.x);
            max_x = std::max(max_x, (double)pt.x);
            min_y = std::min(min_y, (double)pt.y);
            max_y = std::max(max_y, (double)pt.y);
            min_z = std::min(min_z, (double)pt.z);
            max_z = std::max(max_z, (double)pt.z);
        }
    }

    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;
    double center_z = (min_z + max_z) / 2.0;

    PointCloudPcl translated_cloud(new pcl::PointCloud<PointPcl>);
    translated_cloud->points.resize(merged_cloud->size());
    for (size_t i = 0; i < merged_cloud->size(); ++i) {
        auto& pt = merged_cloud->points[i];
        PointPcl translated;
        translated.x = pt.x - center_x;
        translated.y = pt.y - center_y;
        translated.z = pt.z - center_z;
        translated.intensity = pt.intensity;
        translated_cloud->points[i] = translated;
    }
    translated_cloud->width = translated_cloud->points.size();
    translated_cloud->height = 1;

    viewer->addPointCloud<PointPcl>(translated_cloud, "merged_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "merged_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "merged_cloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_vertices(new pcl::PointCloud<pcl::PointXYZ>);

    // Visualize boundaries for each cluster
    for (size_t cluster_idx = 0; cluster_idx < circle_result.boundaries.size(); ++cluster_idx) {
        const auto& cluster_boundaries = circle_result.boundaries[cluster_idx];
        if (cluster_boundaries.size() < 3) continue; // Skip invalid loops

        // Create a point cloud for the loop vertices
        pcl::PointCloud<pcl::PointXYZ>::Ptr loop_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& vertex : cluster_boundaries) {
            loop_cloud->push_back(pcl::PointXYZ(vertex[0] - center_x, vertex[1] - center_y, vertex[2] - center_z));
            boundary_vertices->push_back(pcl::PointXYZ(vertex[0] - center_x, vertex[1] - center_y, vertex[2] - center_z));
        }

        // Draw lines between consecutive vertices (closed loop)
        for (size_t i = 0; i < loop_cloud->size(); ++i) {
            size_t j = (i + 1) % loop_cloud->size(); // Connect to next vertex, wrap around
            std::stringstream line_id;
            line_id << "boundary_" << cluster_idx << "_line_" << i;
            viewer->addLine(
                loop_cloud->points[i],
                loop_cloud->points[j],
                1.0, 0.0, 0.0, // Red for boundaries
                line_id.str()
            );
        }
    }

    viewer->addPointCloud<pcl::PointXYZ>(boundary_vertices, "boundary_vertices");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "boundary_vertices");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "boundary_vertices");

    // Visualize circles
    for (size_t i = 0; i < circle_result.centers.size(); ++i) {
        const auto& center = circle_result.centers[i];
        double radius = circle_result.radii[i];
        const auto& normal = circle_result.normals[i];
        std::string circle_id = "circle_" + std::to_string(i);

        Eigen::Vector3d u = normal.cross(Eigen::Vector3d(1, 0, 0)).normalized();
        if (u.norm() < 0.1) u = normal.cross(Eigen::Vector3d(0, 1, 0)).normalized();
        Eigen::Vector3d v = normal.cross(u).normalized();

        pcl::PointCloud<pcl::PointXYZ>::Ptr circle_points(new pcl::PointCloud<pcl::PointXYZ>);
        int num_segments = 36;
        for (int j = 0; j < num_segments; ++j) {
            double theta = 2.0 * M_PI * j / num_segments;
            Eigen::Vector3d pt = center + (u * std::cos(theta) + v * std::sin(theta)) * radius;
            circle_points->points.emplace_back(pt[0] - center_x, pt[1] - center_y, pt[2] - center_z);
        }

        for (int j = 0; j < num_segments; ++j) {
            std::string line_id = circle_id + "_line_" + std::to_string(j);
            viewer->addLine(
                circle_points->points[j],
                circle_points->points[(j + 1) % num_segments],
                0.0, 0.0, 1.0, // Blue for circles
                line_id
            );
        }
    }

    // Set camera and coordinate system
    double extent = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
    double dist = extent * 2.0;
    if (dist < 1e-6) dist = 10.0;
    viewer->setCameraPosition(0, 0, dist, 0, 0, 0, 0, 1, 0);
    double coord_scale = extent / 10.0;
    if (coord_scale < 0.1) coord_scale = 0.1;
    if (coord_scale > 10.0) coord_scale = 10.0;
    viewer->addCoordinateSystem(coord_scale);
    viewer->initCameraParameters();

    // Spin viewer
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    viewer->close();
}

void visualizePointCloud(const PointCloudPcl& input_cloud, const forResultVizualization& result, const std::string& window_title = "Point Cloud Visualization") {
    // Create a PCLVisualizer object
    auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>(window_title);

    // Set background color to black
    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    // Add the input point cloud (in white)
    if (input_cloud && !input_cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> input_color(input_cloud, 255, 255, 255); // White
        viewer->addPointCloud<pcl::PointXYZI>(input_cloud, input_color, "input_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud");
    } else {
        std::cerr << "Warning: Input point cloud is empty or invalid." << std::endl;
    }

    // Add the inlier cloud (in green)
    if (result.inlier_cloud && !result.inlier_cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> inlier_color(result.inlier_cloud, 0, 255, 0); // Green
        viewer->addPointCloud<pcl::PointXYZI>(result.inlier_cloud, inlier_color, "inlier_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "inlier_cloud");
    } else {
        std::cerr << "Warning: Inlier cloud is empty or invalid." << std::endl;
    }

    // Add coordinate system for reference
    viewer->addCoordinateSystem(1.0);

    // Set camera position (optional, adjust as needed)
    viewer->setCameraPosition(0, 0, 50, 0, 0, 0, 0, 1, 0);

    // Start the visualization loop
    std::cout << "Press 'q' to exit the visualization window." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Close the viewer
    viewer->close();
}


// OTHER HELPER FUNCTIONS .......................................................................................................

CircleFitResult checkVerticalCollisionAndHazardMetrics(
    const PointCloudPcl& input_cloud,
    const CircleFitResult& input_result,
    double step_size = 0.0,
    const double min_point_density = 0.0,
    const double max_relief = 0.0,
    const double max_roughness = 0.0,
    bool enable_xz_proj_visualization = false)
{
    // Output struct to store collision-free candidates with metrics
    std::vector<RankedCandidate> candidates;
    CircleFitResult output_result;
    output_result.boundaries = {}; // Initialize empty boundaries

    // Step 1: Validate inputs
    if (!input_cloud || input_cloud->empty()) {
        std::cerr << "Error: Input point cloud is null or empty!" << std::endl;
        return output_result;
    }
    if (input_result.centers.empty()) {
        std::cerr << "Error: Input CircleFitResult has no circles!" << std::endl;
        return output_result;
    }
    if (input_result.centers.size() != input_result.radii.size() ||
        input_result.centers.size() != input_result.normals.size()) {
        std::cerr << "Error: Inconsistent sizes in CircleFitResult!" << std::endl;
        return output_result;
    }
    if (step_size <= 0) {
        std::cerr << "Error: Invalid step_size (" << step_size << ")!" << std::endl;
        return output_result;
    }

    // Step 2: Compute minZ and maxZ of the input cloud
    double minZ = std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();
    for (const auto& pt : input_cloud->points) {
        if (!std::isnan(pt.z)) {
            minZ = std::min(minZ, static_cast<double>(pt.z));
            maxZ = std::max(maxZ, static_cast<double>(pt.z));
        }
    }
    if (minZ == std::numeric_limits<double>::max()) {
        std::cerr << "Error: No valid Z values in input cloud!" << std::endl;
        return output_result;
    }

    // Step 3: Initialize KD-tree for nearest neighbor search
    pcl::KdTreeFLANN<PointPcl> kdtree;
    kdtree.setInputCloud(input_cloud);

    // Step 4: Process each circle
    for (size_t i = 0; i < input_result.centers.size(); ++i) {
        double roughness_display = 0;
        double relief_display = 0 ;
        double point_density = 0;
        const auto& center = input_result.centers[i];
        double radius = input_result.radii[i];
        auto normal = input_result.normals[i];

        // // Step 4.1: Normalize normal to face positive Z-axis
        // Eigen::Vector3d up(0, 0, 1);
        // if (normal.dot(up) < 0) {
        //     normal = -normal; // Flip normal to face positive Z
        // }
        normal = normal.normalized();

        // StepWAN 4.2: Collect points from minZ to maxZ using KD-tree
        std::vector<int> all_indices;
        std::vector<float> all_sqrDists;
        for (double z = minZ; z <= maxZ; z += step_size) {
            pcl::PointXYZI searchPt;
            searchPt.x = static_cast<float>(center[0]);
            searchPt.y = static_cast<float>(center[1]);
            searchPt.z = static_cast<float>(z);
            std::vector<int> indices;
            std::vector<float> sqrDists;
            kdtree.radiusSearch(searchPt, static_cast<float>(radius), indices, sqrDists);
            all_indices.insert(all_indices.end(), indices.begin(), indices.end());
        }

        // Remove duplicates
        std::sort(all_indices.begin(), all_indices.end());
        all_indices.erase(std::unique(all_indices.begin(), all_indices.end()), all_indices.end());

        if (all_indices.empty()) {
            std::cerr << "Error: all points removed while removing duplicates" << std::endl;
            continue;
        }

        // // Step 4.3: Compute patchMaxZ
        // double patchMaxZ = -std::numeric_limits<double>::max();
        // for (size_t idx : all_indices) {
        //     if (idx < input_cloud->points.size()) {
        //         patchMaxZ = std::max(patchMaxZ, static_cast<double>(input_cloud->points[idx].z));
        //     }
        // }
        // if (patchMaxZ == -std::numeric_limits<double>::max()) {
        //     continue;
        // }

        // Step 4.5: Project points onto XZ plane (Z along normal, X perpendicular)
        Eigen::Vector3d z_axis = normal;
        Eigen::Vector3d x_axis = z_axis.cross(Eigen::Vector3d(1, 0, 0)).normalized();
        if (x_axis.norm() < 0.1) x_axis = z_axis.cross(Eigen::Vector3d(0, 1, 0)).normalized();
        std::vector<Eigen::Vector2d> xz_points;
        for (size_t idx : all_indices) {
            if (idx < input_cloud->points.size()) {
                Eigen::Vector3d pt(input_cloud->points[idx].x - center[0],
                                  input_cloud->points[idx].y - center[1],
                                  input_cloud->points[idx].z - center[2]);
                double x = pt.dot(x_axis);
                double z = pt.dot(z_axis);
                xz_points.emplace_back(x, z);
            }
        }

        // Step 4.5: Project points onto XZ plane (Z along normal, X perpendicular)
        const double threshold = 0.5; // Threshold in meters
        size_t points_above_threshold = 0;
        for (size_t idx : all_indices) {
            if (idx < input_cloud->points.size()) {
                Eigen::Vector3d pt(input_cloud->points[idx].x,
                                   input_cloud->points[idx].y,
                                   input_cloud->points[idx].z);
                Eigen::Vector3d vec_to_pt = pt - center;
                double distance = vec_to_pt.dot(normal); // Signed distance along normal
                if (distance > threshold) {
                    points_above_threshold++;
                }
            }
        }
        if (points_above_threshold >= 20) {

            std::cout << "Error: Number of points above 20: " << points_above_threshold << std::endl;
            continue;
        } 
            // Fewer than 20 points above threshold, filter them out for metrics
            std::vector<Eigen::Vector2d> xz_points_for_metrics = xz_points;
            xz_points_for_metrics.clear();
            for (const auto& pt : xz_points) {
                if (pt[1] <= threshold) {
                    xz_points_for_metrics.push_back(pt);
                }
            }

        // Step 4.7: Compute metrics for visualization (using all points for display)
        double mean_z_display = 0;
        for (const auto& pt : xz_points) mean_z_display += pt[1];
        mean_z_display /= xz_points.size();
        double variance_z_display = 0;
        for (const auto& pt : xz_points) variance_z_display += (pt[1] - mean_z_display) * (pt[1] - mean_z_display);
        roughness_display = std::sqrt(variance_z_display / xz_points.size());
        double min_z_display = std::numeric_limits<double>::max();
        double max_z_display = -std::numeric_limits<double>::max();
        for (const auto& pt : xz_points) {
            min_z_display = std::min(min_z_display, pt[1]);
            max_z_display = std::max(max_z_display, pt[1]);
        }
        relief_display = max_z_display - min_z_display;

        if (enable_xz_proj_visualization) {
            std::cout << "enable_xz_proj_visualization is" << enable_xz_proj_visualization << std::endl;
            // Step 4.8: Visualize XZ projection with legend and metrics
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("XZ Projection - Circle " + std::to_string(i)));
            viewer->setBackgroundColor(0, 0, 0);
            pcl::PointCloud<pcl::PointXYZ>::Ptr xz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& pt : xz_points) {
                xz_cloud->points.emplace_back(pt[0], 0, pt[1]);
            }
            xz_cloud->width = xz_cloud->points.size();
            xz_cloud->height = 1;
            viewer->addPointCloud<pcl::PointXYZ>(xz_cloud, "xz_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "xz_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "xz_cloud");
            viewer->addLine(pcl::PointXYZ(-radius, 0, 0), pcl::PointXYZ(radius, 0, 0), 1, 0, 0, "x_axis");
            viewer->addLine(pcl::PointXYZ(0, 0, -radius), pcl::PointXYZ(0, 0, radius), 0, 1, 0, "z_axis");
            viewer->addText("X-axis (Red)", 10, 50, 1.0, 0.0, 0.0, "x_legend");
            viewer->addText("Z-axis (Green)", 10, 30, 0.0, 1.0, 0.0, "z_legend");
            viewer->addText("Relief (All Points): " + std::to_string(relief_display) + " m", 10, 90, 1.0, 1.0, 1.0, "relief_text");
            viewer->addText("Roughness (All Points): " + std::to_string(roughness_display) + " m", 10, 70, 1.0, 1.0, 1.0, "roughness_text");
            viewer->setCameraPosition(0, 10, 0, 0, 0, 0, 0, 0, 1);
            viewer->addCoordinateSystem(0.5);
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            viewer->close();
        }

        // Step 4.9: Compute roughness and relief using filtered points (if applicable)
        if (xz_points_for_metrics.empty()) {
            std::cout << "Error: No points for metrics calculation!" << std::endl;
            continue;
        }
        // Check roughness threshold
        if (roughness_display >= max_roughness) {
            std::cout << "Error: Roughness exceeds threshold!" << roughness_display << std::endl;
            continue;
        }

        if (relief_display >= max_relief) {
            std::cout << "Error: Relief exceeds threshold!" << relief_display << std::endl;
            continue;
        }

        // Step 4.10: Project points onto XY plane and compute point density
        std::vector<Eigen::Vector2d> xy_points;
        for (size_t idx : all_indices) {
            if (idx < input_cloud->points.size()) {
                Eigen::Vector3d pt(input_cloud->points[idx].x - center[0],
                                  input_cloud->points[idx].y - center[1],
                                  input_cloud->points[idx].z - center[2]);
                xy_points.emplace_back(pt[0], pt[1]);
            }
        }
        double area = M_PI * radius * radius;
        point_density = xy_points.size() / area;
        // Check point density threshold
        if (point_density <= min_point_density) {
            std::cout << "Error: Point density below threshold!" << point_density <<std::endl;
            continue;
        }

        // Step 4.11: Compute slope in degrees
        double slope_rad = std::acos(normal.dot(Eigen::Vector3d(0, 0, 1)));
        double slope_degrees = slope_rad * 180.0 / M_PI;

        // Step 4.12: Store candidate (all thresholds met)
        RankedCandidate candidate;
        candidate.center = center;
        candidate.radius = radius;
        candidate.normal = normal;
        candidate.roughness = roughness_display;
        candidate.relief = relief_display;
        candidate.point_density = point_density;
        candidate.slope_degrees = slope_degrees;
        candidates.push_back(candidate);
    }

    // Step 5: Rank candidates based on weighted sum
    double max_density_rank = 0, max_radius_rank = 0, max_relief_rank = 0, max_roughness_rank = 0;
    for (const auto& c : candidates) {
        max_density_rank = std::max(max_density_rank, c.point_density);
        max_radius_rank = std::max(max_radius_rank, c.radius);
        max_relief_rank = std::max(max_relief_rank, c.relief);
        max_roughness_rank = std::max(max_roughness_rank, c.roughness);
    }
    for (auto& c : candidates) {
        double norm_density = max_density_rank > 0 ? c.point_density / max_density_rank : 0;
        double norm_radius = max_radius_rank > 0 ? c.radius / max_radius_rank : 0;
        double norm_relief = max_relief_rank > 0 ? c.relief / max_relief_rank : 0;
        double norm_roughness = max_roughness_rank > 0 ? c.roughness / max_roughness_rank : 0;
        c.score = 0.4 * norm_density + 0.3 * norm_radius - 0.2 * norm_relief - 0.1 * norm_roughness;
    }

    // Sort by score (descending)
    std::sort(candidates.begin(), candidates.end(),
              [](const RankedCandidate& a, const RankedCandidate& b) {
                  return a.score > b.score;
              });

    // Assign ranks and populate output_result
    for (size_t i = 0; i < candidates.size(); ++i) {
        candidates[i].rank = i + 1;
        output_result.centers.push_back(candidates[i].center);
        output_result.radii.push_back(candidates[i].radius);
        output_result.normals.push_back(candidates[i].normal);
    }

    // Step 6: Print results
    for (const auto& c : candidates) {
        std::cout << "Rank: " << c.rank
                  << ", Center: (" << c.center[0] << ", " << c.center[1] << ", " << c.center[2] << ")"
                  << ", Radius: " << c.radius
                  << ", Slope: " << c.slope_degrees << " deg"
                  << ", Point Density: " << c.point_density
                  << ", Relief: " << c.relief
                  << ", Roughness: " << c.roughness
                  << ", Score: " << c.score << std::endl;
    }

    return output_result;
}

std::vector<Eigen::Vector3d> computeAlphaShapeBoundary3D(
    const std::vector<Eigen::Vector2d>& points_2d, 
    const Eigen::Vector3d& centroid, const Eigen::Vector3d& u, const Eigen::Vector3d& v, 
    double base_alpha, const double min_radius) {

    std::vector<Point2> cgal_pts;
    for (const auto& p : points_2d) {
        cgal_pts.emplace_back(p[0], p[1]);
    }

    std::vector<double> test_alphas = {
        base_alpha * 0.25, base_alpha * 0.5, base_alpha,
        base_alpha * 1.5, base_alpha * 2.0
    };

    // Structure to store alpha values and their segments
    struct AlphaSegments {
        double alpha;
        std::vector<Segment2> segments;
    };
    std::vector<AlphaSegments> alpha_results;

    for (double alpha : test_alphas) {
        AlphaShape2 alpha_shape(cgal_pts.begin(), cgal_pts.end(), alpha, AlphaShape2::GENERAL);

        std::vector<Segment2> segments;
        for (auto eit = alpha_shape.alpha_shape_edges_begin(); eit != alpha_shape.alpha_shape_edges_end(); ++eit) {
            if (alpha_shape.classify(*eit) == AlphaShape2::REGULAR) {
                segments.push_back(alpha_shape.segment(*eit));
            }
        }

        if (!segments.empty()) {
            alpha_results.push_back({alpha, segments});
        }
    }

    if (alpha_results.empty()) {
        std::cerr << "[ERROR] No alpha values produced valid segments." << std::endl;
        return {};
    }

    // Sort alpha_results by alpha value
    std::sort(alpha_results.begin(), alpha_results.end(), 
              [](const AlphaSegments& a, const AlphaSegments& b) { return a.alpha < b.alpha; });

    // Find median alpha index
    size_t median_idx = alpha_results.size() / 2;

    // Create ordered list of indices: median, median-1, median+1, median-2, median+2, ...
    std::vector<size_t> ordered_indices;
    ordered_indices.push_back(median_idx);
    for (size_t offset = 1; offset <= alpha_results.size(); ++offset) {
        if (median_idx >= offset) {
            ordered_indices.push_back(median_idx - offset); // Lower side
        }
        if (median_idx + offset < alpha_results.size()) {
            ordered_indices.push_back(median_idx + offset); // Higher side
        }
    }

    // Process alphas in the specified order
    std::vector<std::vector<Point2>> valid_loops;
    const double min_area = M_PI * min_radius * min_radius;

    for (size_t idx : ordered_indices) {
        const auto& alpha_seg = alpha_results[idx];
        double alpha = alpha_seg.alpha;
        const auto& segments = alpha_seg.segments;

        // Build connectivity graph
        std::multimap<Point2, Point2> connectivity;
        for (const auto& seg : segments) {
            connectivity.insert({seg.source(), seg.target()});
            connectivity.insert({seg.target(), seg.source()});
        }

        // Trace loops
        std::set<Point2> visited;
        for (const auto& [start, _] : connectivity) {
            if (visited.count(start)) continue;
            std::vector<Point2> loop;
            Point2 current = start;
            Point2 prev(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

            while (true) {
                loop.push_back(current);
                visited.insert(current);

                auto range = connectivity.equal_range(current);
                Point2 next = prev;
                for (auto it = range.first; it != range.second; ++it) {
                    if (it->second != prev && !visited.count(it->second)) {
                        next = it->second;
                        break;
                    }
                }
                if (next == prev || next == start) break;
                prev = current;
                current = next;
            }

            if (loop.size() >= 3) {
                Polygon2 poly(loop.begin(), loop.end());
                if (poly.is_simple() && std::abs(poly.area()) >= min_area) {
                    valid_loops.push_back(loop);
                }
            }
        }

        // Stop if valid loops are found
        if (!valid_loops.empty()) {
            break;
        }
    }

    // Convert loops to 3D vertices
    std::vector<Eigen::Vector3d> boundary_3d;
    for (size_t i = 0; i < valid_loops.size(); ++i) {
        const auto& loop = valid_loops[i];
        for (const auto& p : loop) {
            Eigen::Vector3d p_3d = centroid + p.x() * u + p.y() * v;
            boundary_3d.push_back(p_3d);
        }
        // Add separator vertex to distinguish loops
        if (i < valid_loops.size() - 1) {
            boundary_3d.push_back(Eigen::Vector3d(std::numeric_limits<double>::max(),
                                                  std::numeric_limits<double>::max(),
                                                  std::numeric_limits<double>::max()));
        }
    }

    if (valid_loops.empty()) {
        std::cerr << "[ERROR] No valid boundary loops found with area >= " << min_area << std::endl;
    }

    return boundary_3d;
}


CircleFitResult circleFitting(const PointCloudPcl& inlier_cloud, const double min_radius, 
    double alpha = 0.1, int max_num_of_lzs = 10, double max_slope_threshold = 0, double cluster_tolerance = 0.25, 
    bool enable_visualization = true) {
    CircleFitResult result;
    int max_num_of_lz_per_cluster = 5;
    const double min_area = M_PI * min_radius * min_radius;
    int total_lzs_found = 0;

    if (!inlier_cloud || inlier_cloud->empty()) {
        std::cerr << "Error: Merged point cloud is null or empty!" << std::endl;
        return result;
    }

    // Step 1: Cluster the input point cloud using Euclidean distance
    std::cout << "Clustering merged point cloud..." << std::endl;
    pcl::search::KdTree<PointPcl>::Ptr tree(new pcl::search::KdTree<PointPcl>);
    tree->setInputCloud(inlier_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointPcl> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(inlier_cloud);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        std::cerr << "Error: No clusters found!" << std::endl;
        return result;
    }

    std::cout << "Found " << cluster_indices.size() << " clusters." << std::endl;

    if (enable_visualization) {
        visualize_clustered_patches(inlier_cloud, cluster_indices);
    }

    // Step 3: Process each cluster individually and save point clouds
    for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
        const auto& indices = cluster_indices[cluster_idx];

        // Build cluster-specific point cloud
        PointCloudPcl cluster_cloud(new pcl::PointCloud<PointPcl>);
        for (const auto& idx : indices.indices) {
            if (idx >= 0 && idx < inlier_cloud->size()) {
                cluster_cloud->points.push_back(inlier_cloud->points[idx]);
            }
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;

        std::cout << "Processing cluster " << cluster_idx << " with " << cluster_cloud->size() << " points..." << std::endl;

        // Step 4: Compute PCA to fit a plane and get normal + tangent vectors
        pcl::PCA<PointPcl> pca;
        pca.setInputCloud(cluster_cloud);
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
        Eigen::Vector3f eigen_values = pca.getEigenValues();

        // Find index of smallest eigenvalue (normal direction)
        int min_index;
        eigen_values.minCoeff(&min_index);

        // Get normal from eigenvector corresponding to smallest eigenvalue
        Eigen::Vector3d normal = eigen_vectors.col(min_index).cast<double>();

        // Compute centroid
        Eigen::Vector3d centroid(0, 0, 0);
        for (const auto& p : cluster_cloud->points) {
            centroid += Eigen::Vector3d(p.x, p.y, p.z);
        }
        centroid /= cluster_cloud->size();

        // Compute tangent basis vectors in the plane
        Eigen::Vector3d u = normal.cross(Eigen::Vector3d(1, 0, 0)).normalized();
        if (u.norm() < 0.1) u = normal.cross(Eigen::Vector3d(0, 1, 0)).normalized();
        Eigen::Vector3d v = normal.cross(u).normalized();

        // Step 5: Project 3D points onto 2D plane coordinates (u-v frame)
        std::vector<Eigen::Vector2d> points_2d(cluster_cloud->size());
        for (size_t i = 0; i < cluster_cloud->size(); ++i) {
            Eigen::Vector3d p(cluster_cloud->points[i].x, cluster_cloud->points[i].y, cluster_cloud->points[i].z);
            Eigen::Vector3d d = p - centroid;
            points_2d[i] = Eigen::Vector2d(d.dot(u), d.dot(v));
        }

        // Step 6: Compute boundary of cluster using CGAL Alpha Shape
        auto boundary_3d = computeAlphaShapeBoundary3D(points_2d, centroid, u, v, alpha, min_radius);
        std::vector<std::vector<Eigen::Vector3d>> separated_loops;
        std::vector<Eigen::Vector3d> current_loop;
        for (const auto& p : boundary_3d) {
            if (p.x() == std::numeric_limits<double>::max()) {
                if (!current_loop.empty()) {
                    separated_loops.push_back(current_loop);
                    current_loop.clear();
                }
            } else {
                current_loop.push_back(p);
            }
        }
        if (!current_loop.empty()) {
            separated_loops.push_back(current_loop);
        }
        // Append each loop to result.boundaries
        for (const auto& loop : separated_loops) {
            result.boundaries.push_back(loop);
        }
        std::cout << "Extracted " << separated_loops.size() << " boundary loops with "
                  << boundary_3d.size() << " vertices for cluster " << cluster_idx << "." << std::endl;

        // Step 7: Inscribe circles within the boundary loops
        if (separated_loops.empty()) {
            std::cout << "No valid boundary loops for cluster " << cluster_idx << ", skipping circle fitting." << std::endl;
            continue;
        }

        // Convert 3D loops to 2D for circle inscription
        std::vector<std::vector<Point2>> loops_2d;
        for (const auto& loop : separated_loops) {
            std::vector<Point2> loop_2d;
            for (const auto& p_3d : loop) {
                Eigen::Vector3d delta = p_3d - centroid;
                double x = delta.dot(u);
                double y = delta.dot(v);
                loop_2d.emplace_back(x, y);
            }
            loops_2d.push_back(loop_2d);
        }

        // Identify outer and inner loops
        size_t outer_idx = 0;
        double max_area = 0.0;
        for (size_t i = 0; i < loops_2d.size(); ++i) {
            Polygon2 poly(loops_2d[i].begin(), loops_2d[i].end());
            if (poly.is_simple()) {
                double area = std::abs(poly.area());
                if (area > max_area) {
                    max_area = area;
                    outer_idx = i;
                }
            }
        }
        std::vector<Point2> outer_loop = loops_2d[outer_idx];
        std::vector<std::vector<Point2>> inner_loops;
        for (size_t i = 0; i < loops_2d.size(); ++i) {
            if (i != outer_idx) {
                inner_loops.push_back(loops_2d[i]);
            }
        }

        // Create CGAL polygons
        Polygon2 outer_poly(outer_loop.begin(), outer_loop.end());
        std::vector<Polygon2> inner_polys;
        for (const auto& inner : inner_loops) {
            inner_polys.emplace_back(inner.begin(), inner.end());
        }

        // Compute Voronoi diagram for circle centers
        std::vector<Point2> cgal_pts;
        for (const auto& p : points_2d) {
            cgal_pts.emplace_back(p[0], p[1]);
        }
        Delaunay2 dt(cgal_pts.begin(), cgal_pts.end());
        Voronoi_diagram_2 vd(dt);

        int lzs_in_cluster = 0;
        for (auto vit = vd.vertices_begin(); vit != vd.vertices_end() && lzs_in_cluster < max_num_of_lz_per_cluster && total_lzs_found < max_num_of_lzs; ++vit) {
            if (!vit->is_valid()) continue;
            auto dual_face = vit->dual();
            if (!dt.is_infinite(dual_face)) {
                Point2 center = dt.circumcenter(dual_face);
                // Check if center is in the annular region (or inside single loop)
                if (!outer_poly.has_on_bounded_side(center)) continue;
                bool inside_inner = false;
                for (const auto& inner_poly : inner_polys) {
                    if (inner_poly.has_on_bounded_side(center)) {
                        inside_inner = true;
                        break;
                    }
                }
                if (inside_inner) continue;

                // Compute radius as min distance to boundary vertices
                double radius = std::numeric_limits<double>::max();
                for (const auto& p : outer_loop) {
                    radius = std::min(radius, std::sqrt(CGAL::squared_distance(center, p)));
                }
                for (const auto& inner : inner_loops) {
                    for (const auto& p : inner) {
                        radius = std::min(radius, std::sqrt(CGAL::squared_distance(center, p)));
                    }
                }
                if (radius < min_radius) continue;

                // Validate circle containment
                bool is_valid_circle = true;
                const int num_segments = 32;
                for (int j = 0; j < num_segments; ++j) {
                    double theta = 2.0 * M_PI * j / num_segments;
                    double x = center.x() + radius * std::cos(theta);
                    double y = center.y() + radius * std::sin(theta);
                    Point2 perimeter_point(x, y);
                    if (!outer_poly.has_on_bounded_side(perimeter_point) &&
                        !outer_poly.has_on_boundary(perimeter_point)) {
                        is_valid_circle = false;
                        break;
                    }
                    for (const auto& inner_poly : inner_polys) {
                        if (inner_poly.has_on_bounded_side(perimeter_point) ||
                            inner_poly.has_on_boundary(perimeter_point)) {
                            is_valid_circle = false;
                            break;
                        }
                    }
                }
                if (is_valid_circle) {
                    // Convert 2D center to 3D
                    Eigen::Vector3d center_3d = centroid + center.x() * u + center.y() * v;
                    result.centers.push_back(center_3d);
                    result.radii.push_back(radius);
                    float dot_product = std::abs(normal.cast<float>().dot(Eigen::Vector3f::UnitZ()));
                    dot_product = std::min(1.0f, std::max(-1.0f, dot_product)); // Clamp to valid acos input
                    double slope = std::acos(dot_product); // radians
                    double slope_deg = slope * 180.0 / M_PI;
                    if (slope_deg > max_slope_threshold) {
                        std::cerr << "Circle center " << center_3d.transpose() << " has slope " << slope_deg << " degrees, skipping." << std::endl;
                        continue;
                    }
                    result.normals.push_back(normal);
                    lzs_in_cluster++;
                    total_lzs_found++;
                    std::cout << "Circle in cluster " << cluster_idx << ": Center = (" << center_3d.x() << ", "
                              << center_3d.y() << ", " << center_3d.z() << "), Radius = " << radius << " meters" << std::endl;
                }
            }
        }
    }

    return result;
}



// ALGORITHMS.......................................................................................................................

inline CircleFitResult kdtreeNeighbourhoodPCAFilterOMP(
    const PointCloudPcl& input_cloud,
    double initRadius,
    int k,
    float angleThreshold,
    int maxlandingZones,
    int maxAttempts,
    double radiusIncrement)
{
    CircleFitResult result;
    result.boundaries = {};

    #pragma omp critical
    std::cout << "[kdtreeNeighbourhoodPCAFilterOMP] Starting process...\n";

    if (!input_cloud || input_cloud->empty()) {
        #pragma omp critical
        std::cerr << "[kdtreeNeighbourhoodPCAFilterOMP] Error: Empty input cloud!\n";
        return result;
    }

    double minX = DBL_MAX, maxX = -DBL_MAX;
    double minY = DBL_MAX, maxY = -DBL_MAX;

    for (const auto& pt : input_cloud->points) {
        minX = std::min(minX, static_cast<double>(pt.x));
        maxX = std::max(maxX, static_cast<double>(pt.x));
        minY = std::min(minY, static_cast<double>(pt.y));
        maxY = std::max(maxY, static_cast<double>(pt.y));
    }

    pcl::KdTreeFLANN<PointPcl> kdtree;
    kdtree.setInputCloud(input_cloud);

    bool cancel_flag = false;

    #pragma omp parallel for shared(result, cancel_flag)
    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        if (cancel_flag) continue;

        double currentRadius = initRadius;
        bool foundFlat = false;
        PointCloudPcl best_patch;
        Eigen::Vector3f best_normal;
        Eigen::Vector4f best_mean;

        thread_local std::mt19937 rng(std::random_device{}());
        int rand_idx = std::uniform_int_distribution<>(0, input_cloud->size() - 1)(rng);
        PointPcl searchPoint = input_cloud->points[rand_idx];

        if (searchPoint.x < (minX + initRadius) || searchPoint.x > (maxX - initRadius) ||
            searchPoint.y < (minY + initRadius) || searchPoint.y > (maxY - initRadius)) {
            continue;
        }

        while (true) {
            std::vector<int> idx;
            std::vector<float> dist;
            int found = kdtree.radiusSearch(searchPoint, currentRadius, idx, dist);
            if (found < k) break;

            auto patch = std::make_shared<pcl::PointCloud<PointPcl>>();
            for (int i : idx)
                patch->points.push_back(input_cloud->points[i]);

            // --- PCA + Normal Estimation ---
            pcl::NormalEstimationOMP<PointPcl, pcl::Normal> ne;
            ne.setInputCloud(patch);
            ne.setKSearch(k);
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            ne.compute(*normals);

            if (normals->size() != patch->size()) break;

            pcl::PCA<PointPcl> pca;
            pca.setInputCloud(patch);
            Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
            Eigen::Vector4f mean = pca.getMean();
            Eigen::Vector3f normal = eigenvectors.col(2);

            float dot_z = std::fabs(normal.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
            float slope = std::acos(dot_z) * 180.0f / static_cast<float>(M_PI);

            if (slope > angleThreshold) break;

            size_t inliers = 0;
            for (size_t i = 0; i < patch->size(); ++i) {
                Eigen::Vector3f n(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
                float angle = std::acos(std::fabs(n.dot(normal))) * 180.0f / static_cast<float>(M_PI);
                if (angle <= angleThreshold)
                    inliers++;
            }

            if (inliers == patch->size()) {
                foundFlat = true;
                best_patch = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*patch);
                best_normal = normal;
                best_mean = mean;
                currentRadius += radiusIncrement;
                continue;
            }
            else if (foundFlat) {
                double finalRadius = currentRadius - radiusIncrement;
                #pragma omp critical
                {
                    result.centers.emplace_back(searchPoint.x, searchPoint.y, searchPoint.z);
                    result.normals.emplace_back(best_normal.cast<double>());
                    result.radii.push_back(finalRadius);
                    std::cout << "[Thread " << omp_get_thread_num() << "] Found patch at radius " << finalRadius << "\n";
                }

                if (result.centers.size() >= static_cast<size_t>(maxlandingZones)) {
                    #pragma omp critical
                    {
                        std::cout << "[Thread " << omp_get_thread_num() << "] Max landing zones reached.\n";
                    }
                    cancel_flag = true;
                    #pragma omp cancel for
                }
                break;
            }
            else {
                break;
            }
        }
    }

    #pragma omp critical
    {
        std::cout << "[kdtreeNeighbourhoodPCAFilterOMP] Done. Found " << result.centers.size() << " zones.\n";
    }

    return result;
}

forResultVizualization sequentialOverlappingApproach(const PointCloudPcl& cloud, float window_size, float slope_threshold_deg){
    forResultVizualization result;
    result.inlier_cloud = PointCloudPcl(new pcl::PointCloud<PointPcl>);
    std::set<int> unique_inlier_indices; // Use set for deduplication

    // Validate inputs
    if (window_size <= 0) {
        throw std::invalid_argument("Window size must be positive.");
    }
    if (cloud->empty()) {
        throw std::runtime_error("Input point cloud is empty.");
    }
    if (slope_threshold_deg < 0 || slope_threshold_deg > 90) {
        throw std::invalid_argument("Slope threshold must be between 0 and 90 degrees.");
    }

    // Find bounds
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    // Calculate grid parameters
    float step_size = window_size / 2;
    int grid_width = static_cast<int>(std::ceil((max_x - min_x) / step_size)) + 1;
    int grid_height = static_cast<int>(std::ceil((max_y - min_y) / step_size)) + 1;
    constexpr int inter_cell_step_count = 2; // Hardcoded overlap steps

    // Iterate over grid with overlapping windows
    for (int cell_y = 0; cell_y < grid_height; ++cell_y) {
        for (int j = 0; j < inter_cell_step_count; ++j) {
            for (int cell_x = 0; cell_x < grid_width; ++cell_x) {
                for (int i = 0; i < inter_cell_step_count; ++i) {
                    // Define kernel bounds
                    float kernel_x_min = min_x + cell_x * step_size + i * step_size;
                    float kernel_x_max = kernel_x_min + window_size;
                    float kernel_y_min = min_y + cell_y * step_size + j * step_size;
                    float kernel_y_max = kernel_y_min + window_size;

                    // Collect points within the square window
                    std::vector<int> cell_indices;
                    PointCloudPcl cell_cloud(new pcl::PointCloud<PointPcl>);
                    for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
                        const auto& point = cloud->points[idx];
                        if (point.x >= kernel_x_min && point.x < kernel_x_max &&
                            point.y >= kernel_y_min && point.y < kernel_y_max) {
                            cell_indices.push_back(idx);
                            cell_cloud->points.push_back(point);
                        }
                    }

                    // Skip cells with fewer than 3 points
                    if (cell_indices.size() < 3) {
                        continue;
                    }

                    // Perform PCA
                    try {
                        pcl::PCA<PointPcl> pca;
                        pca.setInputCloud(cell_cloud);
                        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
                        Eigen::Vector3f eigen_values = pca.getEigenValues();

                        // Find normal (smallest eigenvalue)
                        int min_index;
                        eigen_values.minCoeff(&min_index);
                        Eigen::Vector3f normal = eigen_vectors.col(min_index);

                        // Compute slope
                        float dot_product = std::abs(normal.dot(Eigen::Vector3f::UnitZ()));
                        dot_product = std::min(1.0f, std::max(-1.0f, dot_product));
                        double slope_deg = std::acos(dot_product) * 180.0 / M_PI;

                        // Add indices if slope is within threshold
                        if (slope_deg <= slope_threshold_deg) {
                            unique_inlier_indices.insert(cell_indices.begin(), cell_indices.end());
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Error computing PCA for cell: " << e.what() << std::endl;
                        continue;
                    }
                }
            }
        }
    }

    // Build inlier_cloud and cluster_indices
    if (!unique_inlier_indices.empty()) {
        result.inlier_cloud->points.reserve(unique_inlier_indices.size());
        pcl::PointIndices inlier_indices;
        for (const auto& idx : unique_inlier_indices) {
            if (idx >= 0 && idx < cloud->points.size()) {
                result.inlier_cloud->points.push_back(cloud->points[idx]);
                inlier_indices.indices.push_back(idx);
            }
        }
        result.inlier_cloud->width = result.inlier_cloud->points.size();
        result.inlier_cloud->height = 1;
        if (!inlier_indices.indices.empty()) {
            result.cluster_indices.push_back(inlier_indices);
        }
    } else {
        std::cerr << "Warning: No inlier points found." << std::endl;
    }

    return result;
}

forResultVizualization sequentialApproach(const PointCloudPcl& cloud, float window_size, float slope_threshold_deg) {
    forResultVizualization result;
    result.inlier_cloud = PointCloudPcl(new pcl::PointCloud<PointPcl>);
    std::set<int> unique_inlier_indices; // Use set for deduplication

    // Validate inputs
    if (window_size <= 0) {
        throw std::invalid_argument("Window size must be positive.");
    }
    if (cloud->empty()) {
        throw std::runtime_error("Input point cloud is empty.");
    }
    if (slope_threshold_deg < 0 || slope_threshold_deg > 90) {
        throw std::invalid_argument("Slope threshold must be between 0 and 90 degrees.");
    }

    // Find bounds
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    // Calculate grid parameters (non-overlapping)
    float step_size = window_size; // Step size equals window size for no overlap
    int grid_width = static_cast<int>(std::ceil((max_x - min_x) / step_size));
    int grid_height = static_cast<int>(std::ceil((max_y - min_y) / step_size));

    // Iterate over grid with non-overlapping square windows
    for (int cell_y = 0; cell_y < grid_height; ++cell_y) {
        for (int cell_x = 0; cell_x < grid_width; ++cell_x) {
            // Define cell bounds
            float kernel_x_min = min_x + cell_x * step_size;
            float kernel_x_max = kernel_x_min + window_size;
            float kernel_y_min = min_y + cell_y * step_size;
            float kernel_y_max = kernel_y_min + window_size;

            // Collect points within the square cell iteratively
            std::vector<int> filtered_indices;
            for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
                const auto& point = cloud->points[idx];
                if (point.x >= kernel_x_min && point.x < kernel_x_max &&
                    point.y >= kernel_y_min && point.y < kernel_y_max) {
                    filtered_indices.push_back(idx);
                }
            }

            // Skip cells with fewer than 25 points
            if (filtered_indices.size() < 25) {
                continue;
            }

            // Create cell_cloud for PCA
            PointCloudPcl cell_cloud(new pcl::PointCloud<PointPcl>);
            cell_cloud->points.reserve(filtered_indices.size());
            for (const auto& idx : filtered_indices) {
                cell_cloud->points.push_back(cloud->points[idx]);
            }

            // Perform PCA
            try {
                pcl::PCA<PointPcl> pca;
                pca.setInputCloud(cell_cloud);
                Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
                Eigen::Vector3f eigen_values = pca.getEigenValues();

                // Find normal (smallest eigenvalue)
                int min_index;
                eigen_values.minCoeff(&min_index);
                Eigen::Vector3f normal = eigen_vectors.col(min_index);

                // Compute slope
                float dot_product = std::abs(normal.dot(Eigen::Vector3f::UnitZ()));
                dot_product = std::min(1.0f, std::max(-1.0f, dot_product));
                double slope_deg = std::acos(dot_product) * 180.0 / M_PI;

                // Add indices if slope is within threshold
                if (slope_deg <= slope_threshold_deg) {
                    unique_inlier_indices.insert(filtered_indices.begin(), filtered_indices.end());
                }
            } catch (const std::exception& e) {
                std::cerr << "Error computing PCA for cell: " << e.what() << std::endl;
                continue;
            }
        }
    }

    // Build inlier_cloud and cluster_indices
    if (!unique_inlier_indices.empty()) {
        result.inlier_cloud->points.reserve(unique_inlier_indices.size());
        pcl::PointIndices inlier_indices;
        for (const auto& idx : unique_inlier_indices) {
            if (idx >= 0 && idx < cloud->points.size()) {
                result.inlier_cloud->points.push_back(cloud->points[idx]);
                inlier_indices.indices.push_back(idx);
            }
        }
        result.inlier_cloud->width = result.inlier_cloud->points.size();
        result.inlier_cloud->height = 1;
        if (!inlier_indices.indices.empty()) {
            result.cluster_indices.push_back(inlier_indices);
        }
    } else {
        std::cerr << "Warning: No inlier points found." << std::endl;
    }

    return result;
}

forResultVizualization sequentialApproachKdtree(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    float window_size,
    float slope_threshold_deg) {
    
    forResultVizualization result;
    result.inlier_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::set<int> unique_inlier_indices; // For deduplication

    // Validate inputs
    if (window_size <= 0) {
        throw std::invalid_argument("Window size must be positive.");
    }
    if (cloud->empty()) {
        throw std::runtime_error("Input point cloud is empty.");
    }
    if (slope_threshold_deg < 0 || slope_threshold_deg > 90) {
        throw std::invalid_argument("Slope threshold must be between 0 and 90 degrees.");
    }

    // Build KD-tree for efficient spatial queries
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    // Step 1: Compute min and max x, y of the input cloud
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        if (!std::isnan(point.z)) {
            min_z = std::min(min_z, static_cast<double>(point.z));
            max_z = std::max(max_z, static_cast<double>(point.z));
        }
    }

    // Calculate grid parameters (non-overlapping)
    float step_size = window_size; // No overlap
    int grid_width = static_cast<int>(std::ceil((max_x - min_x) / step_size));
    int grid_height = static_cast<int>(std::ceil((max_y - min_y) / step_size));
    const float radius = window_size; // Use half the cell size as radius

    // Iterate over grid with non-overlapping square windows
    for (int cell_y = 0; cell_y < grid_height; ++cell_y) {
        for (int cell_x = 0; cell_x < grid_width; ++cell_x) {
            // Define kernel bounds
            float kernel_x_min = min_x + cell_x * step_size;
            float kernel_x_max = kernel_x_min + window_size;
            float kernel_y_min = min_y + cell_y * step_size;
            float kernel_y_max = kernel_y_min + window_size;

            // Compute kernel center
            float center_x = (kernel_x_min + kernel_x_max) / 2.0f;
            float center_y = (kernel_y_min + kernel_y_max) / 2.0f;

            // Collect points from min_z to max_z using KD-tree
            std::vector<int> all_indices;
            std::vector<float> all_sqrDists;
            for (double z = min_z; z <= max_z; z += 0.5) { // Step by 0.5m in z
                pcl::PointXYZI searchPt;
                searchPt.x = center_x;
                searchPt.y = center_y;
                searchPt.z = static_cast<float>(z);
                std::vector<int> indices;
                std::vector<float> sqrDists;
                kdtree.radiusSearch(searchPt, radius, indices, sqrDists);
                all_indices.insert(all_indices.end(), indices.begin(), indices.end());
            }

            // Filter points to keep only those within kernel bounds
            std::vector<int> filtered_indices;
            filtered_indices.reserve(all_indices.size());
            for (const auto& idx : all_indices) {
                if (idx >= 0 && idx < cloud->points.size()) {
                    const auto& point = cloud->points[idx];
                    if (point.x >= kernel_x_min && point.x < kernel_x_max &&
                        point.y >= kernel_y_min && point.y < kernel_y_max) {
                        filtered_indices.push_back(idx);
                    }
                }
            }

            // Skip cells with fewer than 25 points
            if (filtered_indices.size() < 25) {
                continue;
            }

            // Create cell_cloud for PCA
            pcl::PointCloud<pcl::PointXYZI>::Ptr cell_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            cell_cloud->points.reserve(filtered_indices.size());
            for (const auto& idx : filtered_indices) {
                cell_cloud->points.push_back(cloud->points[idx]);
            }

            // Perform PCA
            try {
                pcl::PCA<pcl::PointXYZI> pca;
                pca.setInputCloud(cell_cloud);
                Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
                Eigen::Vector3f eigen_values = pca.getEigenValues();

                // Find normal (smallest eigenvalue)
                int min_index;
                eigen_values.minCoeff(&min_index);
                Eigen::Vector3f normal = eigen_vectors.col(min_index);

                // Compute slope
                float dot_product = std::abs(normal.dot(Eigen::Vector3f::UnitZ()));
                dot_product = std::min(1.0f, std::max(-1.0f, dot_product));
                double slope_deg = std::acos(dot_product) * 180.0 / M_PI;

                // Add indices if slope is within threshold
                if (slope_deg <= slope_threshold_deg) {
                    unique_inlier_indices.insert(filtered_indices.begin(), filtered_indices.end());
                }
            } catch (const std::exception& e) {
                std::cerr << "Error computing PCA for cell: " << e.what() << std::endl;
                continue;
            }
        }
    }

    // Build inlier_cloud and cluster_indices
    if (!unique_inlier_indices.empty()) {
        result.inlier_cloud->points.reserve(unique_inlier_indices.size());
        pcl::PointIndices inlier_indices;
        for (const auto& idx : unique_inlier_indices) {
            if (idx >= 0 && idx < cloud->points.size()) {
                result.inlier_cloud->points.push_back(cloud->points[idx]);
                inlier_indices.indices.push_back(idx);
            }
        }
        result.inlier_cloud->width = result.inlier_cloud->points.size();
        result.inlier_cloud->height = 1;
        if (!inlier_indices.indices.empty()) {
            result.cluster_indices.push_back(inlier_indices);
        }
    } else {
        std::cerr << "Warning: No inlier points found." << std::endl;
    }

    return result;
}

forResultVizualization segmentPointCloud(const PointCloud& input_cloud,
    float curvature_threshold,
    float angle_threshold_deg,
    int min_cluster_size = 50) {
    forResultVizualization result;
    // Initialize inlier_cloud
    result.inlier_cloud = PointCloudPcl(new pcl::PointCloud<PointPcl>);

    PointCloudPcl pcl_cloud;
    if (std::holds_alternative<PointCloudPcl>(input_cloud)) {
        pcl_cloud = std::get<PointCloudPcl>(input_cloud);
    } else if (std::holds_alternative<PointCloudOpen3D>(input_cloud)) {
        pcl_cloud = convertOpen3DToPCL(std::get<PointCloudOpen3D>(input_cloud));
    } else {
        std::cerr << "Error: Invalid point cloud type!" << std::endl;
        return result;
    }

    if (pcl_cloud->empty()) {
        std::cerr << "Error: Input point cloud is empty!" << std::endl;
        return result;
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointPcl, pcl::Normal> normal_estimator;
    pcl::search::KdTree<PointPcl>::Ptr tree(new pcl::search::KdTree<PointPcl>);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(pcl_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    if (normals->empty()) {
        std::cerr << "Error: Normal estimation failed!" << std::endl;
        return result;
    }

    pcl::RegionGrowing<PointPcl, pcl::Normal> reg;
    reg.setMinClusterSize(min_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(pcl_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(angle_threshold_deg * M_PI / 180.0);
    reg.setCurvatureThreshold(curvature_threshold);

    reg.extract(result.cluster_indices);

    for (const auto& cluster : result.cluster_indices) {
        PointCloudPcl patch(new pcl::PointCloud<PointPcl>);
        for (const auto& idx : cluster.indices) {
            patch->points.push_back(pcl_cloud->points[idx]);
            // Add point to merged cloud
            result.inlier_cloud->points.push_back(pcl_cloud->points[idx]);
        }
        patch->width = patch->points.size();
        patch->height = 1;
        if (!patch->empty()) {
            // result.patches.push_back(patch);
        }
    }

    // Set merged cloud properties
    result.inlier_cloud->width = result.inlier_cloud->points.size();
    result.inlier_cloud->height = 1;
    result.inlier_cloud->is_dense = pcl_cloud->is_dense;

    return result;
}

#endif // ARCHITECTURE_H