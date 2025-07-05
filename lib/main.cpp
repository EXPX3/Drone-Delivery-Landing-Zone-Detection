
#include <vector>
#include "yaml-cpp/yaml.h"
#include "common.h"
#include "hazard_metrices.h"
#include "pointcloud_preprocessing.h"
#include "architecture.h"

using PointT = pcl::PointXYZI;

int main(int argc, char **argv)
{

    std::string config_file = "/home/airsim_user/Landing-Assist-Module-LAM/lib/config/pipeline2Octree.yaml";
    // Load YAML configuration.
    YAML::Node config = YAML::LoadFile(config_file);
    YAML::Node params = config["ros__parameters"];

    // Set file paths.
    std::string pcd_file_path = params["pcd_file_path"].as<std::string>();

    bool g_visualize = params["visualize"].as<bool>();
    bool final_visualize = params["final_visualize"].as<bool>();
    bool voxel_downsample_pointcloud = params["voxel_downsample_pointcloud"].as<bool>();
    std::string final_result_visualization = params["final_result_visualization"].as<std::string>();
    float voxelSize = params["voxel_size"].as<float>();
    PCLResult pclResult;
    pclResult.downsampled_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    pclResult.inlier_cloud = pcl::make_shared<typename pcl::PointCloud<PointT>>();
    auto loaded_cloud_pcl = loadPCLCloud<PointT>(pcd_file_path);


    PCLResult final_result;
    final_result.outlier_cloud = loaded_cloud_pcl;

    if (voxel_downsample_pointcloud)
    {
        downsamplePointCloudPCL<PointT>(loaded_cloud_pcl, pclResult.inlier_cloud, voxelSize);
        std::cout << "Downsampled cloud has " << pclResult.inlier_cloud->points.size() << " points." << std::endl;
     
    }
    else
    {
        pclResult.inlier_cloud = loaded_cloud_pcl;
    }
    // Get the pipeline configuration.
    YAML::Node pipeline = params["pipeline"];

    // Process each step in the pipeline sequentially.
    for (std::size_t i = 0; i < pipeline.size(); ++i)
    {
        std::string step = pipeline[i]["step"].as<std::string>();
        bool enabled = pipeline[i]["enabled"].as<bool>();
        if (!enabled)
        {
            std::cout << "\n--- Skipping disabled step: " << step << " ---\n";
            continue;
        }
        std::cout << "\n--- Running pipeline step: " << step << " ---\n";

        if (step == "SOR")
        {
            int nb_neighbors = pipeline[i]["parameters"]["nb_neighbors"].as<int>();
            double std_ratio = pipeline[i]["parameters"]["std_ratio"].as<double>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();

            // Convert pointcloud into open3d version
            OPEN3DResult pointcloud;
            auto pointCloud = convertPCLToOpen3D(pclResult);
            OPEN3DResult result = apply_sor_filter(pointCloud.inlier_cloud, nb_neighbors, std_ratio);
            auto pclResult = convertOpen3DToPCL(result);

            if (g_visualize)
            {
                visualizePCL(pclResult, visualization);
            }

        }
        else if (step == "SphericalNeighbourhood")
        {
            auto start = std::chrono::high_resolution_clock::now();

            
            int k = pipeline[i]["parameters"]["k"].as<int>();
            float angleThreshold = pipeline[i]["parameters"]["angleThreshold"].as<float>();
            double radius = pipeline[i]["parameters"]["radius"].as<double>();
            std::string visualization = pipeline[i]["parameters"]["visualization"].as<std::string>();
            int landingZoneNumber = pipeline[i]["parameters"]["landingZoneNumber"].as<int>();
            int maxAttempts = pipeline[i]["parameters"]["maxAttempts"].as<int>();
            float textSize = pipeline[i]["parameters"]["visualization_textSize"].as<float>();
            
            PCLResult result;
    
            std::vector<SLZDCandidatePoints> candidatePoints;
            std::tie(result, candidatePoints) =kdtreeNeighbourhoodPCAFilterOMP(pclResult.inlier_cloud,
                                            radius, k, angleThreshold,
                                            landingZoneNumber, maxAttempts);
            
            
            result.plane_coefficients = pclResult.plane_coefficients;
            auto rankedCandidates = rankCandidatePatches(candidatePoints, result);
           
             // End the timer
            auto end = std::chrono::high_resolution_clock::now();
            // Calculate the elapsed time in seconds (or choose another unit)
            std::chrono::duration<double> duration = end - start;
            // Print the elapsed time
            std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
           
            pclResult.inlier_cloud = result.inlier_cloud;
            if (g_visualize)
            {
                visualizeRankedCandidatePatches(rankedCandidates, result,textSize);
                
            }
        }
        else if(step == "HazarMetrices"){
            std::string hazardMetricsName = pipeline[i]["parameters"]["hazard"].as<std::string>();
            auto hazard = rankCandidatePatchFromPCLResult(pclResult, hazardMetricsName);
        }
        else
        {
            std::cerr << "Unknown pipeline step: " << step << std::endl;
            return -1;
        }
    }

    // Final visualization of the chained output.
    if (g_visualize || final_visualize)
    {
        std::cout << "\n--- Final Processed Cloud ---\n";

        final_result.inlier_cloud = pclResult.inlier_cloud;
        visualizePCL(final_result, final_result_visualization);
    }

    return 0;
}
