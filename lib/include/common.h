#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <open3d/Open3D.h>
#include <variant>

using PointPcl = pcl::PointXYZI;
using PointCloudPcl = pcl::PointCloud<PointPcl>::Ptr;
using PointCloudOpen3D = std::shared_ptr<open3d::geometry::PointCloud>;
using PointCloud = std::variant<PointCloudPcl, PointCloudOpen3D>; //covers both pcl and open3d pointcloud types

PointCloud cloud;
PointCloud inlier_cloud ;
PointCloud outlier_cloud ;
std::variant<pcl::ModelCoefficients::Ptr, std::shared_ptr<Eigen::Vector4d>> plane_coefficients; //covers both pcl and open3d libs
std::string processing_method;
std::string hazardMetric_type;

// HELPER STRUCTURES .......................................................................................................

struct processingResult {
    PointCloud inlier_cloud;
    PointCloud outlier_cloud;
    std::string processing_method;
    std::string hazardMetric_type;
    std::variant<pcl::ModelCoefficients::Ptr, std::shared_ptr<Eigen::Vector4d>> plane_coefficients;
    // Constructor to initialize all members
    processingResult() 
        : inlier_cloud(PointCloud(PointCloudPcl(new pcl::PointCloud<PointPcl>()))),
          outlier_cloud(PointCloud(PointCloudPcl(new pcl::PointCloud<PointPcl>()))),
          processing_method("none"),
          hazardMetric_type("none"),
          plane_coefficients(pcl::ModelCoefficients::Ptr())  // Explicit null PCL coefficients 
    {}
    
};
struct LandingZoneCandidatePoint {
    PointPcl center;  // A center of circular region selected to be a potential candidate for landing
    PointCloud circular_patch;  // Single detected surface, represented as a point cloud
    double dataConfidence;  // Data confidence for the candidate zone
    double roughness;  // Roughness value for the candidate landing zone
    double relief;  // Relief value for the candidate landing zone
    double score;  // Score of the candidate
    double patchRadius; // final radius of the grown patch
    std::variant<pcl::ModelCoefficients::Ptr, std::shared_ptr<Eigen::Vector4d>> plane_coefficients;  // Plane coefficients for the surface

    // Constructor to initialize the struct
    LandingZoneCandidatePoint()
        : center(0.0, 0.0, 0.0, 0.0),  // x, y, z, intensity
          circular_patch(PointCloud(PointCloudPcl(new pcl::PointCloud<PointPcl>()))),  // Empty PCL cloud
          dataConfidence(0.0),
          roughness(0.0),
          relief(0.0),
          score(0.0),
          patchRadius(0.0),
          plane_coefficients(pcl::ModelCoefficients::Ptr(nullptr))  // Explicit null PCL coefficients
    {}
};
struct forResultVizualization {
    std::vector<pcl::PointIndices> cluster_indices;
    PointCloudPcl inlier_cloud;  // Added merged point cloud
};

// OTHER HELPER FUNCTIONS .......................................................................................................
//===================================Function to convert OPEN3D to PCL ==============================================
inline PointCloudPcl convertOpen3DToPCL(const PointCloudOpen3D &open3d_cloud) {
    PointCloudPcl pcl_cloud = std::make_shared<pcl::PointCloud<PointPcl>>();

    if (open3d_cloud && !open3d_cloud->points_.empty()) {
        for (const auto &pt : open3d_cloud->points_) {
            PointPcl pcl_pt;
            pcl_pt.x = pt(0);
            pcl_pt.y = pt(1);
            pcl_pt.z = pt(2);
            pcl_cloud->points.push_back(pcl_pt);
        }
        pcl_cloud->width = pcl_cloud->points.size();
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = true;
    } else {
        std::cerr << "[convertOpen3DToPCL] Warning: input open3d type cloud is empty." << std::endl;
    }
    return pcl_cloud;
}

// ========================Function to convert PCL into OPEN3D =============================================

inline PointCloudOpen3D convertPCLToOpen3D(const PointCloudPcl &pcl_cloud) {
    PointCloudOpen3D open3d_cloud;
    
    // Create new Open3D point cloud.
    open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();

    // Convert downsampled cloud.
    if (pcl_cloud && !pcl_cloud->points.empty()) {
        for (const auto &pt : pcl_cloud->points) {
            open3d_cloud->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        }
    } else {
        std::cerr << "[convertPCLToOpen3D] Warning: input pcl type cloud is empty." << std::endl;
    }
    return open3d_cloud;
}


//====================== Helper function to downsample open3d and pcl types point cloud==================================
inline PointCloud downSamplePointCloud(const PointCloud& cloud, double voxelSize) {
    PointCloud downsampled_cloud = PointCloudPcl(new pcl::PointCloud<PointPcl>()); // Default initialization

    if (std::holds_alternative<PointCloudPcl>(cloud)) {
        auto pcl_cloud = std::get<PointCloudPcl>(cloud);
        if (pcl_cloud && !pcl_cloud->points.empty()) {
            pcl::VoxelGrid<PointPcl> vg;
            auto pcl_downsampled = std::make_shared<pcl::PointCloud<PointPcl>>();
            vg.setInputCloud(pcl_cloud);
            vg.setLeafSize(voxelSize, voxelSize, voxelSize);
            vg.filter(*pcl_downsampled);
            downsampled_cloud = pcl_downsampled;
            std::cout << "Downsampled PCL point cloud has " 
                      << pcl_downsampled->points.size() << " points." << std::endl;
        }
    } else if (std::holds_alternative<PointCloudOpen3D>(cloud)) {
        auto open3d_cloud = std::get<PointCloudOpen3D>(cloud);
        if (open3d_cloud && !open3d_cloud->points_.empty()) {
            auto open3d_downsampled = open3d_cloud->VoxelDownSample(voxelSize);
            downsampled_cloud = open3d_downsampled;
            std::cout << "Downsampled Open3D point cloud has " 
                      << open3d_downsampled->points_.size() << " points." << std::endl;
        }
    } else {
        std::cerr << "[downSamplePointCloud] Unsupported point cloud type." << std::endl;
    }

    return downsampled_cloud;
}

// Function to load a point cloud from a file path and return it as open3d pointer
inline PointCloud loadPointCloudFromFile(const std::string& file_path) {
    
    PointCloudOpen3D cloud = std::make_shared<open3d::geometry::PointCloud>();
        if (!open3d::io::ReadPointCloud(file_path, *cloud)) {
            std::cerr << "Failed to load file: " << file_path << std::endl;
            exit(EXIT_FAILURE);
        }
        std::cout << "Loaded Open3D cloud with " << cloud->points_.size() << " points." << std::endl;
        return cloud; // Returns as Open3D variant
}

inline void visualizePCL(const processingResult &result, const std::string& viz_inlier_or_outlier_or_both = "both")
{
    std::cout << "[INFO] visualizePCL: Starting visualization\n";

    // Create a visualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Processing method: " + result.processing_method +
                                             " HazardMetric type: " + result.hazardMetric_type +
                                             " PCL based visualization"));
    viewer->setBackgroundColor(1.0, 1.0, 1.0);
    std::cout << "[INFO] visualizePCL: Viewer initialized\n";

    // Initialize PCL clouds
    std::shared_ptr<pcl::PointCloud<PointPcl>> pcl_outlier_cloud = nullptr;
    std::shared_ptr<pcl::PointCloud<PointPcl>> pcl_inlier_cloud = nullptr;

    // Handle outlier cloud
    if (std::holds_alternative<PointCloudPcl>(result.outlier_cloud)) {
        pcl_outlier_cloud = std::get<PointCloudPcl>(result.outlier_cloud);
        std::cout << "[INFO] visualizePCL: Outlier cloud is PCL, size: " 
                  << (pcl_outlier_cloud ? pcl_outlier_cloud->size() : 0) << "\n";
    } else if (std::holds_alternative<PointCloudOpen3D>(result.outlier_cloud)) {
        std::cout << "[INFO] visualizePCL: Converting Open3D outlier cloud to PCL\n";
        auto open3d_cloud = std::get<PointCloudOpen3D>(result.outlier_cloud);
        pcl_outlier_cloud = std::make_shared<pcl::PointCloud<PointPcl>>();
        if (open3d_cloud && !open3d_cloud->points_.empty()) {
            pcl_outlier_cloud->points.reserve(open3d_cloud->points_.size());
            for (const auto& point : open3d_cloud->points_) {
                PointPcl p;
                p.x = point(0);
                p.y = point(1);
                p.z = point(2);
                p.intensity = 1.0; // Default intensity
                pcl_outlier_cloud->push_back(p);
            }
            pcl_outlier_cloud->width = pcl_outlier_cloud->size();
            pcl_outlier_cloud->height = 1;
            pcl_outlier_cloud->is_dense = true;
        }
        std::cout << "[INFO] visualizePCL: Outlier cloud (converted), size: " 
                  << (pcl_outlier_cloud ? pcl_outlier_cloud->size() : 0) << "\n";
    } else {
        std::cerr << "[ERROR] visualizePCL: Unknown variant type for outlier_cloud\n";
    }

    // Handle inlier cloud
    if (std::holds_alternative<PointCloudPcl>(result.inlier_cloud)) {
        pcl_inlier_cloud = std::get<PointCloudPcl>(result.inlier_cloud);
        std::cout << "[INFO] visualizePCL: Inlier cloud is PCL, size: " 
                  << (pcl_inlier_cloud ? pcl_inlier_cloud->size() : 0) << "\n";
    } else if (std::holds_alternative<PointCloudOpen3D>(result.inlier_cloud)) {
        std::cout << "[INFO] visualizePCL: Converting Open3D inlier cloud to PCL\n";
        auto open3d_cloud = std::get<PointCloudOpen3D>(result.inlier_cloud);
        pcl_inlier_cloud = std::make_shared<pcl::PointCloud<PointPcl>>();
        if (open3d_cloud && !open3d_cloud->points_.empty()) {
            pcl_inlier_cloud->points.reserve(open3d_cloud->points_.size());
            for (const auto& point : open3d_cloud->points_) {
                PointPcl p;
                p.x = point(0);
                p.y = point(1);
                p.z = point(2);
                p.intensity = 1.0;
                pcl_inlier_cloud->push_back(p);
            }
            pcl_inlier_cloud->width = pcl_inlier_cloud->size();
            pcl_inlier_cloud->height = 1;
            pcl_inlier_cloud->is_dense = true;
        }
        std::cout << "[INFO] visualizePCL: Inlier cloud (converted), size: " 
                  << (pcl_inlier_cloud ? pcl_inlier_cloud->size() : 0) << "\n";
    } else {
        std::cerr << "[ERROR] visualizePCL: Unknown variant type for inlier_cloud\n";
    }

    // Add the outlier cloud (red) if available
    if (pcl_outlier_cloud && !pcl_outlier_cloud->empty() &&
        (viz_inlier_or_outlier_or_both == "outlier_cloud" || viz_inlier_or_outlier_or_both == "both"))
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointPcl> outlierColorHandler(pcl_outlier_cloud, 255, 0, 0); // Red color
        viewer->addPointCloud<PointPcl>(pcl_outlier_cloud, outlierColorHandler, "outlier_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "outlier_cloud");
        std::cout << "[INFO] visualizePCL: Added outlier cloud to viewer\n";
    }

    // Add the inlier cloud (green) if available
    if (pcl_inlier_cloud && !pcl_inlier_cloud->empty() &&
        (viz_inlier_or_outlier_or_both == "inlier_cloud" || viz_inlier_or_outlier_or_both == "both"))
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointPcl> inlierColorHandler(pcl_inlier_cloud, 0, 255, 0); // Green color
        viewer->addPointCloud<PointPcl>(pcl_inlier_cloud, inlierColorHandler, "inlier_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "inlier_cloud");
        std::cout << "[INFO] visualizePCL: Added inlier cloud to viewer\n";
    }

    // If no points added, log warning
    if (!(pcl_outlier_cloud && !pcl_outlier_cloud->empty()) && !(pcl_inlier_cloud && !pcl_inlier_cloud->empty())) {
        std::cout << "[WARNING] visualizePCL: No points to display (both clouds empty)\n";
    }

    // Set up camera and axes for better visibility
    viewer->resetCamera();
    viewer->addCoordinateSystem(1.0); // Add XYZ axes (1 meter scale)
    std::cout << "[INFO] visualizePCL: Camera and axes set\n";

    // Main loop to keep the visualizer window open
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[INFO] visualizePCL: Visualizer closed\n";
}

#endif 