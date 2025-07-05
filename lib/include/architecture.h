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
#include <tuple>
#include <algorithm>
#include <cstdlib>
#include <ctime>

// PCL includes

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/surface/convex_hull.h>

#include <open3d/Open3D.h>
#include "common.h"  // For PCLResult, loadPCLCloud, etc.
#include <vtkCamera.h>


//-----------------------------------------------------------------------------
// computePatchData: compute XY centroid, patchRadius, and maxZ for a given patch
//-----------------------------------------------------------------------------
inline void computePatchData(const pcl::PointCloud<pcl::PointXYZI>::Ptr &patchCloud,
                             double &cx, double &cy,
                             double &patchRadius,
                             double &maxZ)
{
    if (!patchCloud || patchCloud->empty()) {
        cx = cy = patchRadius = maxZ = 0.0;
        return;
    }
    // Compute XY centroid
    cx = 0.0; cy = 0.0;
    for (const auto &pt : patchCloud->points) {
        cx += pt.x;
        cy += pt.y;
    }
    cx /= patchCloud->size();
    cy /= patchCloud->size();

    // Compute XY radius (max distance to centroid in XY) and maxZ
    patchRadius = 0.0;
    maxZ = -std::numeric_limits<double>::infinity();
    for (const auto &pt : patchCloud->points) {
        double dx = pt.x - cx;
        double dy = pt.y - cy;
        double distXY = std::hypot(dx, dy);
        if (distXY > patchRadius) {
            patchRadius = distXY;
        }
        if (pt.z > maxZ) {
            maxZ = pt.z;
        }
    }
}
//-----------------------------------------------------------------------------
// isPatchCollisionFreekdtree: checks if there's an obstacle above the patch
//-----------------------------------------------------------------------------
inline bool isPatchCollisionFreeKdtree(
    const pcl::PointCloud<PointT>::Ptr &cloud,
    pcl::KdTreeFLANN<PointT> &kdtree,
    double cx,            // Patch centroid X
    double cy,            // Patch centroid Y
    double patchRadius,   // Patch's XY radius
    double patchMaxZ,     // Patch's highest Z
    double margin,        // Additional margin above patchMaxZ considered an obstacle
    double clearance,     // Total height above patchMaxZ to check
    double step_size)     // Step size in Z
{
    // We'll sample from patchMaxZ in steps of step_size,
    // up to patchMaxZ + clearance
    int steps = static_cast<int>(std::ceil(clearance / step_size));
    
    // The threshold above which we consider it "blocked" if we find a point
    double obstacleZThreshold = patchMaxZ + margin;
    
    for (int i = 0; i <= steps; i++) {
        // Current Z level
        double zCurrent = patchMaxZ + i * step_size;
        
        // Build a search point at this Z level, same XY center
        PointT searchPt;
        searchPt.x = static_cast<float>(cx);
        searchPt.y = static_cast<float>(cy);
        searchPt.z = static_cast<float>(zCurrent);
        
        // Perform a radius search with the same XY radius
        std::vector<int> indices;
        std::vector<float> sqrDists;
        int found = kdtree.radiusSearch(searchPt, static_cast<float>(patchRadius),
                                        indices, sqrDists);
        if (found <= 0) {
            // No points at this level => no immediate obstacle here
            continue;
        }

        // Among these points, check if any have z > patchMaxZ + margin
        // (i.e., actual obstacle above the patch)
        for (int idx : indices) {
            if (cloud->points[idx].z > obstacleZThreshold) {
                // Found an obstacle -> patch is not collision-free
                return false;
            }
        }
    }

    // If we finish the entire sampling without finding obstacles,
    // then the patch is clear in that cylindrical region
    return true;
}
//-----------------------------------------------------------------------------

// kdtreeNeighbourhoodPCAFilter: returns multiple collision-free patches(OMP PARELLIZATION)
//----------------------------------------------------------------------------------------
inline std::tuple<PCLResult, std::vector<SLZDCandidatePoints>> kdtreeNeighbourhoodPCAFilterOMP(
    const CloudInput<pcl::PointXYZI>& input,
    double initRadius,
    int k,
    float angleThreshold,
    int landingZoneNumber,
    int maxAttempts)
{
    srand(static_cast<unsigned int>(time(0)));

    std::vector<SLZDCandidatePoints> finalCandidates;
    std::vector<PCLResult> candidatePatches;

    auto cloud = loadPCLCloud<pcl::PointXYZI>(input);
    if (!cloud || cloud->empty()) {
        return std::make_tuple(PCLResult(), finalCandidates);
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();

    for (const auto &pt : cloud->points) {
        if (pt.x < minX) minX = pt.x;
        if (pt.x > maxX) maxX = pt.x;
        if (pt.y < minY) minY = pt.y;
        if (pt.y > maxY) maxY = pt.y;
        if (pt.z < minZ) minZ = pt.z;
        if (pt.z > maxZ) maxZ = pt.z;
    }

    std::cout << "[kdtreeNeighbourhoodPCAFilter] Cloud bounding box (XYZ): "
              << "x=[" << minX << ", " << maxX << "], "
              << "y=[" << minY << ", " << maxY << "], "
              << "z=[" << minZ << ", " << maxZ << "]\n";


    // Build an kdtree
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);


    std::cout << "[kdtreeNeighbourhoodPCAFilter] initRadius=" << initRadius
              << ", k=" << k
              << ", angleThreshold=" << angleThreshold
              << ", landingZoneNumber=" << landingZoneNumber
              << ", maxAttempts=" << maxAttempts << "\n\n";

    bool stopParallel = false;
    omp_lock_t stop_lock;
    omp_init_lock(&stop_lock); // Initialize the lock

    #pragma omp parallel for shared(finalCandidates, candidatePatches, cloud, \
                                    kdtree, minX, maxX, minY, maxY, stopParallel, stop_lock)
    for (int attempt = 0; attempt < maxAttempts; ++attempt)
    {
        if (stopParallel) {
            continue; // Skip the current iteration if stopParallel is set
        }

        double currentRadius = initRadius;
        double radiusIncrement = 0.5;
        double circularityThreshold = 0.1;
        double clearance = 10.0;
        double margin = 0.01;
        double step_size = 0.2;

        bool foundFlat = false;
        PCLResult bestFlatPatch;
        double bestCircularity = 0.0;
        double finalSuccessfulRadius = 0.0;

        int randomIndex = rand() % cloud->size();
        pcl::PointXYZI searchPoint = cloud->points[randomIndex];

        #pragma omp critical
        {
            std::cout << "[Thread " << omp_get_thread_num()
                      << "] --> Attempt #" << attempt
                      << "  Seed=(" << searchPoint.x << ", "
                      << searchPoint.y << ", "
                      << searchPoint.z << ")\n";
        }

        if (searchPoint.x < (minX + initRadius) || searchPoint.x > (maxX - initRadius) ||
            searchPoint.y < (minY + initRadius) || searchPoint.y > (maxY - initRadius))
        {
            #pragma omp critical
            {
                std::cout << "[Thread " << omp_get_thread_num()
                          << "] Attempt " << attempt
                          << ": seed near XY edge => skip.\n";
            }
            continue;
        }

        while (true) {
            std::vector<int> idx;
            std::vector<float> dist;
            int found = kdtree.radiusSearch(searchPoint, currentRadius, idx, dist);

            #pragma omp critical
            {
                std::cout << "[Thread " << omp_get_thread_num()
                          << "] Attempt " << attempt
                          << "  radius=" << currentRadius
                          << " => " << found << " neighbors.\n";
            }

            if (found < 3) {
                break;
            }

            auto patch = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            patch->reserve(found);
            for (int i = 0; i < found; i++) {
                patch->points.push_back(cloud->points[idx[i]]);
            }

            PCLResult pcaResult = PrincipleComponentAnalysis(patch, angleThreshold, k);

            if (pcaResult.outlier_cloud->empty()) {
                pcl::ConvexHull<pcl::PointXYZI> chull;
                chull.setInputCloud(patch);
                chull.setDimension(2);

                auto hull = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                chull.reconstruct(*hull);

                double area = 0.0, perimeter = 0.0;
                if (hull->size() >= 3) {
                    for (size_t i = 0; i < hull->size(); i++) {
                        size_t j = (i + 1) % hull->size();
                        double xi = hull->points[i].x, yi = hull->points[i].y;
                        double xj = hull->points[j].x, yj = hull->points[j].y;
                        area += (xi * yj - xj * yi);
                        perimeter += std::hypot(xj - xi, yj - yi);
                    }
                    area = std::fabs(area) * 0.5;
                }
                double circ = (perimeter > 0)
                              ? (4.0 * M_PI * area) / (perimeter * perimeter)
                              : 0.0;

                #pragma omp critical
                {
                    std::cout << "[Thread " << omp_get_thread_num()
                              << "] Attempt " << attempt
                              << " => FLAT, circ=" << circ << "\n";
                }

                if (circ >= circularityThreshold) {
                    bestFlatPatch = pcaResult;
                    bestCircularity = circ;
                    foundFlat = true;
                    finalSuccessfulRadius = currentRadius;
                }
                currentRadius += radiusIncrement;
                continue;
            }
            else {
                #pragma omp critical
                {
                    std::cout << "[Thread " << omp_get_thread_num()
                              << "] Attempt " << attempt
                              << " => NOT flat => stop.\n";
                }
            }
            break;
        }

        if (foundFlat) {
            double cx = 0, cy = 0, patchR = 0, patchMaxZ = 0;
            computePatchData(bestFlatPatch.inlier_cloud, cx, cy, patchR, patchMaxZ);

            bool collisionFree = isPatchCollisionFreeKdtree(
                cloud, kdtree, cx, cy, patchR, patchMaxZ,
                margin, clearance, step_size);

            if (collisionFree) {
                SLZDCandidatePoints candidate;
                candidate.seedPoint.x = searchPoint.x;
                candidate.seedPoint.y = searchPoint.y;
                candidate.seedPoint.z = searchPoint.z;
                candidate.plane_coefficients = bestFlatPatch.plane_coefficients;

                candidate.dataConfidence = calculateDataConfidencePCL(bestFlatPatch);
                candidate.relief = calculateReliefPCL(bestFlatPatch);
                candidate.roughness = calculateRoughnessPCL(bestFlatPatch);
                candidate.patchRadius = finalSuccessfulRadius;
                candidate.score = candidate.dataConfidence
                                 + candidate.patchRadius
                                 - candidate.relief
                                 - candidate.roughness;
                candidate.detectedSurface = bestFlatPatch.inlier_cloud;

                #pragma omp critical
                {
                    finalCandidates.push_back(candidate);
                    candidatePatches.push_back(bestFlatPatch);

                    if (finalCandidates.size() >= landingZoneNumber) {
                        stopParallel = true;  // Stop further attempts if we have enough landing zones
                    }

                    std::cout << "[Thread " << omp_get_thread_num()
                              << "] Attempt " << attempt
                              << " => Patch is collision-free, appended!\n";
                }
            }
            else {
                #pragma omp critical
                {
                    std::cout << "[Thread " << omp_get_thread_num()
                              << "] Attempt " << attempt
                              << " => Patch has obstacle => skip.\n";
                }
            }
        }
        else {
            #pragma omp critical
            {
                std::cout << "[Thread " << omp_get_thread_num()
                          << "] Attempt " << attempt
                          << " => No suitable patch from this seed.\n";
            }
        }

        currentRadius = initRadius;
    }

    std::cout << "\n[kdtreeNeighbourhoodPCAFilter] Finished parallel loop.\n"
              << "  Found " << finalCandidates.size() << " patches.\n";

    PCLResult finalResult;
    finalResult.inlier_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    finalResult.outlier_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    finalResult.downsampled_cloud = cloud;

    for (auto &patch : candidatePatches) {
        finalResult.inlier_cloud->insert(finalResult.inlier_cloud->end(),
                                         patch.inlier_cloud->begin(),
                                         patch.inlier_cloud->end());
    }

    for (size_t i = 0; i < cloud->size(); i++) {
        finalResult.outlier_cloud->push_back(cloud->points[i]);
    }

    std::cout << "[kdtreeNeighbourhoodPCAFilter] FINAL:\n"
              << "   Inlier cloud = " << finalResult.inlier_cloud->size() << " pts\n"
              << "   Outlier cloud= " << finalResult.outlier_cloud->size() << " pts\n\n";

    // Cleanup the lock
    omp_destroy_lock(&stop_lock);

    return std::make_tuple(finalResult, finalCandidates);
}
//-----------------------------------------------------------------------------
// rankCandidatePatches: sorts all your collected patches by descending score
//-----------------------------------------------------------------------------
inline std::vector<SLZDCandidatePoints> rankCandidatePatches(
    std::vector<SLZDCandidatePoints>& candidatePoints,
    PCLResult& result)
{
    // Sort candidate patches by descending score
    std::sort(candidatePoints.begin(), candidatePoints.end(),
              [](const SLZDCandidatePoints &a, const SLZDCandidatePoints &b) {
                  return a.score > b.score;
              });

    std::cout << "Ranked Candidates (sorted by score):\n";
    for (size_t i = 0; i < candidatePoints.size(); ++i) {
        const auto &cand = candidatePoints[i];
        std::cout << "Rank " << i + 1 << " - Score: " << cand.score << "\n";
        std::cout << "  Data Confidence: " << cand.dataConfidence << "\n";
        std::cout << "  Relief: " << cand.relief << "\n";
        std::cout << "  Roughness: " << cand.roughness << "\n";
        std::cout << "  Patch Radius: " << cand.patchRadius << "\n";
        std::cout << "--------------------------\n";
    }
    return candidatePoints;
}


//-----------------------------------------------------------------------------
// visualizeRankedCandidatePatches: displays a rank label for each patch
//-----------------------------------------------------------------------------
inline void visualizeRankedCandidatePatches(const std::vector<SLZDCandidatePoints>& candidatePoints, 
    const PCLResult &result,float textSize)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer(result.pcl_method + " PCL RESULT"));
    viewer->setBackgroundColor(1.0, 1.0, 1.0);

    // Fixed color for visualization
    pcl::RGB fixedColor = {255, 0, 0};  // Red color

    // Visualize outliers if available
    if (result.outlier_cloud && !result.outlier_cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> outlierColorHandler(
            result.outlier_cloud, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZI>(result.outlier_cloud, outlierColorHandler, "outlier_cloud");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "outlier_cloud");
    }

    // Visualize inliers if available
    if (result.inlier_cloud && !result.inlier_cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> inlierColorHandler(
            result.inlier_cloud, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZI>(result.inlier_cloud, inlierColorHandler, "inlier_cloud");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "inlier_cloud");
    }

    std::cout << "Candidate patches size: " << candidatePoints.size() << std::endl;

    // Prepare data in a thread-safe way
    std::vector<std::tuple<int, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZI>::Ptr, float>> patchData;

    // Parallelize the data processing part (calculating data for each patch)
    #pragma omp parallel for default(none) shared(candidatePoints, patchData)
    for (size_t idx = 0; idx < candidatePoints.size(); ++idx) {
        const auto &candidate = candidatePoints[idx];

        // Calculate the center point of the patch (seedPoint)
        pcl::PointXYZ centerXYZ;
        centerXYZ.x = candidate.seedPoint.x;
        centerXYZ.y = candidate.seedPoint.y;
        centerXYZ.z = candidate.seedPoint.z;

        // Store the required data for visualization
        #pragma omp critical
        patchData.push_back(std::make_tuple(idx, centerXYZ, candidate.detectedSurface, candidate.patchRadius));
    }

    // Now visualize the patches
    for (const auto &data : patchData) {
        int idx;
        pcl::PointXYZ centerXYZ;
        pcl::PointCloud<pcl::PointXYZI>::Ptr detectedSurface;
        float radius;

        std::tie(idx, centerXYZ, detectedSurface, radius) = data;

        // Visualize the patch surface (points of the patch)
        std::stringstream ss;
        ss << "candidate_" << idx << "_surface";
        viewer->addPointCloud<pcl::PointXYZI>(detectedSurface, ss.str());

        // Add center point as a sphere with the fixed color
        std::stringstream centerName;
        centerName << "center_" << idx;
        viewer->addSphere(centerXYZ, 500.1, fixedColor.r, fixedColor.g, fixedColor.b, centerName.str());

        // Label as "Rank i+1" with the same color as the center point
        std::stringstream label;
        label << "Rank " << idx + 1;
        viewer->addText3D(label.str(), centerXYZ, textSize, fixedColor.r, fixedColor.g, fixedColor.b, "label_" + std::to_string(idx));

        // Generate and visualize the boundary (circumference of the patch)
        int num_points = 10000; // Number of points to form the circle
        pcl::PointCloud<pcl::PointXYZ>::Ptr circle(new pcl::PointCloud<pcl::PointXYZ>);

        // Generate points for the circumference of the circle
        for (int i = 0; i < num_points; ++i) {
            // Calculate points on the circumference of the circle
            float angle = 2 * M_PI * i / num_points;
            pcl::PointXYZ pt;
            pt.x = centerXYZ.x + radius * cos(angle);
            pt.y = centerXYZ.y + radius * sin(angle);
            pt.z = centerXYZ.z;

            circle->points.push_back(pt);
        }

        // Visualize the boundary (circle) with the fixed color
        std::stringstream circleName;
        circleName << "circle_" << idx;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> circleColorHandler(circle, fixedColor.r * 255, fixedColor.g * 255, fixedColor.b * 255);
        viewer->addPointCloud<pcl::PointXYZ>(circle, circleColorHandler, circleName.str());
    }

    viewer->resetCamera();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



#endif
