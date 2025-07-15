#ifndef HAZARD_METRICES_H
#define HAZARD_METRICES_H

#include <iostream>
#include <cmath>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include <eigen3/Eigen/Dense>

#include <common.h>
#include <variant>

inline double calculateRoughness(processingResult& result)
{auto result_inlier_cloud = std::get<PointCloudPcl>(result.inlier_cloud);
auto result_plane_coefficients = std::get<pcl::ModelCoefficients::Ptr>(result.plane_coefficients);
  // Check if the plane coefficients are valid
  if (result_plane_coefficients->values.size() < 4 || result_inlier_cloud->points.empty())
  {
    std::cerr << "Invalid plane coefficients or empty inlier cloud. Cannot compute roughness." << std::endl;
    return -1.0;
  }
  
  // Extract plane parameters (ax + by + cz + d = 0).
  double a = result_plane_coefficients->values[0];
  double b = result_plane_coefficients->values[1];
  double c = result_plane_coefficients->values[2];
  double d = result_plane_coefficients->values[3];
  
  // Calculate the plane normal's magnitude for normalization
  double norm = std::sqrt(a * a + b * b + c * c);
  
  // Variable to accumulate the squared distance of each point from the plane
  double sum_squared = 0.0;
  size_t N = result_inlier_cloud->points.size();
  
  // Loop over each point in the result cloud to calculate the roughness
  for (const auto &pt : result_inlier_cloud->points)
  {
    // Calculate the distance from the point to the plane
    double distance = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
    sum_squared += distance * distance;
  }
  
  // Return the square root of the average squared distance (roughness)
  return std::sqrt(sum_squared / static_cast<double>(N));
}
//=============================Calculate Relief==========================================================================
// Calculate relief from the inlier cloud (safe landing zone).
inline double calculateRelief(processingResult& result)
{auto result_inlier_cloud = std::get<PointCloudPcl>(result.inlier_cloud);
    if (!result_inlier_cloud || result_inlier_cloud->points.empty()) {
        std::cerr << "Error: Inlier cloud is empty." << std::endl;
        return -1.0;
    }

    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();

    // Iterate through inlier points and compute min and max z values.
    for (const auto &pt : result_inlier_cloud->points) {
        double z = pt.z;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
    }
    
    return z_max - z_min;
}
//=============================Calculate Data Confidence=======================================================================
// It computes the 2D convex hull (projecting the inlier cloud) and returns N divided by the hull area.
inline double calculateDataConfidence(processingResult& result)
{auto result_inlier_cloud = std::get<PointCloudPcl>(result.inlier_cloud);
    if (!result_inlier_cloud || result_inlier_cloud->points.empty()) {
        std::cerr << "Error: Inlier cloud is empty." << std::endl;
        return -1.0;
    }

    size_t N = result_inlier_cloud->points.size();

    // Compute the convex hull of the inlier cloud projected onto a plane.
    pcl::ConvexHull<pcl::PointXYZI> chull;
    chull.setInputCloud(result_inlier_cloud);
    chull.setDimension(2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::Vertices> polygons;
    chull.reconstruct(*hull_points, polygons);

    if (polygons.empty() || hull_points->points.empty()) {
        std::cerr << "Error: Convex hull could not be computed." << std::endl;
        return -1.0;
    }

    // Compute the area of the first polygon using the shoelace formula.
    double area = 0.0;
    const std::vector<int>& indices = polygons[0].vertices;
    size_t n = indices.size();
    if (n < 3) {
        std::cerr << "Error: Convex hull does not have enough points to form an area." << std::endl;
        return -1.0;
    }

    for (size_t i = 0; i < n; i++) {
        const auto& p1 = hull_points->points[indices[i]];
        const auto& p2 = hull_points->points[indices[(i + 1) % n]];
        area += (p1.x * p2.y - p2.x * p1.y);
    }
    area = std::abs(area) / 2.0;

    if (area <= 0.0) {
        std::cerr << "Error: Computed hull area is non-positive." << std::endl;
        return -1.0;
    }

    double data_confidence = static_cast<double>(N) / area;
    return data_confidence;
}
#endif 


