#ifndef HAZARD_METRICES_H
#define HAZARD_METRICES_H


#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <omp.h>
#include "common.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/common.h> 
#include <pcl/common/pca.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <eigen3/Eigen/Dense>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

using CloudInput = std::variant<std::string, typename pcl::PointCloud<PointT>::Ptr>;
 




//======================================PCA (Principle Component Analysis)(PCL)==========================================================================================================
inline PCLResult PrincipleComponentAnalysis(const CloudInput<PointT>& input,
  float angleThreshold = 20.0f,
  int k = 10)
{
PCLResult result;
result.pcl_method = "Principal Component Analysis (using NormalEstimationOMP)";
result.inlier_cloud = pcl::make_shared<PointCloudT>();
result.outlier_cloud = pcl::make_shared<PointCloudT>();
result.downsampled_cloud = pcl::make_shared<PointCloudT>();

// Load the cloud and determine if downsampling is needed.
auto cloud = loadPCLCloud<PointT>(input);

result.downsampled_cloud = cloud;


// Compute normals in parallel using NormalEstimationOMP.
pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
ne.setInputCloud(result.downsampled_cloud);
ne.setKSearch(k);

pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
ne.compute(*normals);


// Compute PCA on the entire cloud.
pcl::PCA<PointT> pca;
pca.setInputCloud(result.downsampled_cloud);
Eigen::Matrix3f eigenvectors = pca.getEigenVectors(); // Eigenvectors sorted by descending eigenvalues.
Eigen::Vector4f mean = pca.getMean();

// Use the eigenvector with the smallest eigenvalue (typically the 3rd column) as the global plane normal.
Eigen::Vector3f global_normal = eigenvectors.col(2);

// Compute and output the global slope (angle between global_normal and the vertical (0,0,1)).
float global_dot = std::fabs(global_normal.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
float global_slope = std::acos(global_dot) * 180.0f / static_cast<float>(M_PI);
std::cout << "Global PCA plane slope: " << global_slope << " degrees" << std::endl;

// Compute plane coefficients: Ax + By + Cz + D = 0.
float A = global_normal(0);
float B = global_normal(1);
float C = global_normal(2);
float D = -(A * mean(0) + B * mean(1) + C * mean(2));
result.plane_coefficients = std::make_shared<pcl::ModelCoefficients>();
result.plane_coefficients->values.push_back(A);
result.plane_coefficients->values.push_back(B);
result.plane_coefficients->values.push_back(C);
result.plane_coefficients->values.push_back(D);
std::cout << "Plane coefficients (A, B, C, D): " << A << ", " << B << ", " << C << ", " << D << std::endl;

// Classify points based on the computed normals and the angle threshold.
for (size_t i = 0; i < normals->points.size(); i++) {
Eigen::Vector3f normal(normals->points[i].normal_x,
normals->points[i].normal_y,
normals->points[i].normal_z);
// Check for invalid normal values.
if (std::isnan(normal.norm()) || normal.norm() == 0) {
result.outlier_cloud->push_back(result.downsampled_cloud->points[i]);
continue;
}
float dot_product = std::fabs(normal.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
float slope = std::acos(dot_product) * 180.0f / static_cast<float>(M_PI);
if (slope <= angleThreshold)
result.inlier_cloud->push_back(result.downsampled_cloud->points[i]);
else
result.outlier_cloud->push_back(result.downsampled_cloud->points[i]);
}
std::cout << "Inliers (slope ≤ " << angleThreshold << "°): " << result.inlier_cloud->size() << std::endl;
std::cout << "Outliers (slope > " << angleThreshold << "°): " << result.outlier_cloud->size() << std::endl;

return result;
}

inline double calculateRoughnessPCL(PCLResult& result)
{
  // Check if the plane coefficients are valid
  if (result.plane_coefficients->values.size() < 4 || result.inlier_cloud->points.empty())
  {
    std::cerr << "Invalid plane coefficients or empty inlier cloud. Cannot compute roughness." << std::endl;
    return -1.0;
  }
  
  // Extract plane parameters (ax + by + cz + d = 0).
  double a = result.plane_coefficients->values[0];
  double b = result.plane_coefficients->values[1];
  double c = result.plane_coefficients->values[2];
  double d = result.plane_coefficients->values[3];
  
  // Calculate the plane normal's magnitude for normalization
  double norm = std::sqrt(a * a + b * b + c * c);
  
  // Variable to accumulate the squared distance of each point from the plane
  double sum_squared = 0.0;
  size_t N = result.inlier_cloud->points.size();
  
  // Loop over each point in the result.er cloud to calculate the roughness
  for (const auto &pt : result.inlier_cloud->points)
  {
    // Calculate the distance from the point to the plane
    double distance = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
    sum_squared += distance * distance;
  }
  
  // Return the square root of the average squared distance (roughness)
  return std::sqrt(sum_squared / static_cast<double>(N));
}

//=============================Calculate Relief (PCL)==========================================================================
// Calculate relief from the inlier cloud (safe landing zone).
inline double calculateReliefPCL(PCLResult& result)
{
    if (!result.inlier_cloud || result.inlier_cloud->points.empty()) {
        std::cerr << "Error: Inlier cloud is empty." << std::endl;
        return -1.0;
    }

    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();

    // Iterate through inlier points and compute min and max z values.
    for (const auto &pt : result.inlier_cloud->points) {
        double z = pt.z;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
    }
    
    return z_max - z_min;
}

//=============================Calculate Data Confidence (OPEN3D)==========================================================================

// Calculate data confidence for Open3DResult.
// It computes the convex hull of the inlier cloud and returns N (number of points)
// divided by the convex hull area.
inline double calculateDataConfidenceOpen3D(const OPEN3DResult &result)
{
    if (!result.inlier_cloud || result.inlier_cloud->points_.empty()) {
        std::cerr << "Error: Inlier cloud is empty." << std::endl;
        return -1.0;
    }
    
    size_t N = result.inlier_cloud->points_.size();

    // Compute convex hull of the inlier cloud.
    std::shared_ptr<open3d::geometry::TriangleMesh> hull_mesh;
    std::vector<size_t> hull_indices;
    std::tie(hull_mesh, hull_indices) = result.inlier_cloud->ComputeConvexHull();

    if (!hull_mesh) {
        std::cerr << "Error: Convex hull computation failed." << std::endl;
        return -1.0;
    }
    
    double area = hull_mesh->GetSurfaceArea();
    if (area <= 0.0) {
        std::cerr << "Error: Computed hull area is non-positive." << std::endl;
        return -1.0;
    }
    
    double data_confidence = static_cast<double>(N) / area;
    return data_confidence;
}

inline SLZDCandidatePoints rankCandidatePatchFromPCLResult(PCLResult& result, const std::string& metrics = "ALL") 
{
    // Create an SLZDCandidatePoints object for the result
    SLZDCandidatePoints candidate;  
    // By default, the new SLZDCandidatePoints has dataConfidence, relief,
    // roughness, and score set to 0.0 in its constructor.

    // Check if the result contains a valid inlier cloud
    if (result.inlier_cloud && !result.inlier_cloud->points.empty()) 
    {
        PCLResult surfResult;
        surfResult.inlier_cloud = result.inlier_cloud;
        surfResult.plane_coefficients = result.plane_coefficients;

        // Calculate metrics if "ALL" is selected
        if (metrics == "ALL") 
        {
            candidate.dataConfidence = calculateDataConfidencePCL(surfResult);
            candidate.relief         = calculateReliefPCL(surfResult);
            candidate.roughness      = calculateRoughnessPCL(surfResult);
        }
        else if (metrics == "DATA_CONFIDENCE") 
        {
            candidate.dataConfidence = calculateDataConfidencePCL(surfResult);
        }
        else if (metrics == "RELIEF") 
        {
            candidate.relief = calculateReliefPCL(surfResult);
        }
        else if (metrics == "ROUGHNESS") 
        {
            candidate.roughness = calculateRoughnessPCL(surfResult);
        }
        else 
        {
            std::cerr << "[rankCandidatePatchFromPCLResult] Error: Invalid metric specified." << std::endl;
        }

        // Compute the candidate patch's score.
        // If "ALL" or "DATA_CONFIDENCE" is selected => add dataConfidence
        // If "ALL" or "RELIEF" => subtract relief
        // If "ALL" or "ROUGHNESS" => subtract roughness
        double tempScore = 0.0;
        if (metrics == "ALL" || metrics == "DATA_CONFIDENCE") {
            tempScore += candidate.dataConfidence;
        }
        if (metrics == "ALL" || metrics == "RELIEF") {
            tempScore -= candidate.relief;
        }
        if (metrics == "ALL" || metrics == "ROUGHNESS") {
            tempScore -= candidate.roughness;
        }

        candidate.score = tempScore;

        // Print details of the candidate patch
        std::cout << "Candidate Patch Details:" << std::endl;
        if (metrics == "ALL" || metrics == "DATA_CONFIDENCE") {
            std::cout << "  Data Confidence: " << candidate.dataConfidence << std::endl;
        }
        if (metrics == "ALL" || metrics == "RELIEF") {
            std::cout << "  Relief: " << candidate.relief << std::endl;
        }
        if (metrics == "ALL" || metrics == "ROUGHNESS") {
            std::cout << "  Roughness: " << candidate.roughness << std::endl;
        }

        std::cout << "  Final Score: " << candidate.score << std::endl;
    }
    else 
    {
        std::cerr << "[rankCandidatePatchFromPCLResult] Error: Inlier cloud is empty." << std::endl;
    }

    return candidate;
}



#endif 


