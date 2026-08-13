#include "ddlzd_core/detector.hpp"

#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ddlzd
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRequiredRadiusM = 2.5;

double quantile(std::vector<double> values, const double q)
{
  if (values.empty()) {
    return 0.0;
  }
  const double position = std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(lower), values.end());
  const double lower_value = values[lower];
  if (upper == lower) {
    return lower_value;
  }
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(upper), values.end());
  return lower_value + (position - static_cast<double>(lower)) * (values[upper] - lower_value);
}

double horizontalDistance(const Candidate & lhs, const Candidate & rhs)
{
  return (lhs.center.head<2>() - rhs.center.head<2>()).norm();
}

struct LatticeCellHash
{
  std::size_t operator()(const std::pair<std::int64_t, std::int64_t> & cell) const noexcept
  {
    const auto x_hash = std::hash<std::int64_t>{}(cell.first);
    const auto y_hash = std::hash<std::int64_t>{}(cell.second);
    return x_hash ^ (y_hash + 0x9e3779b9U + (x_hash << 6U) + (x_hash >> 2U));
  }
};

}  // namespace

Detector::Detector(DetectorConfig config)
: config_(std::move(config))
{
  if (std::abs(config_.landing_radius_m - kRequiredRadiusM) > 1e-9) {
    throw std::invalid_argument("live_demo_cargopack requires an exact 2.5 m landing radius");
  }
  if (!(config_.candidate_spacing_m > 0.0 && config_.voxel_leaf_m > 0.0 &&
    config_.nms_distance_m > 0.0 && config_.plane_inlier_threshold_m > 0.0 &&
    config_.safety_margin_m >= 0.0 &&
    config_.obstacle_search_radius_m >= config_.landing_radius_m + config_.safety_margin_m &&
    config_.obstacle_height_m > 0.0 &&
    config_.region_smoothness_deg > 0.0 && config_.region_smoothness_deg < 90.0 &&
    config_.curvature_threshold > 0.0 && config_.max_slope_deg > 0.0 &&
    config_.max_slope_deg < 90.0 && config_.min_density_m2 > 0.0 &&
    config_.max_relief_m > 0.0 && config_.max_roughness_m > 0.0 &&
    config_.min_observed_fraction > 0.0 && config_.min_observed_fraction <= 1.0))
  {
    throw std::invalid_argument("invalid detector metric or distance threshold");
  }
  if (config_.angular_bins == 0U || config_.radial_bins == 0U ||
    config_.normal_neighbors < 3U || config_.min_plane_points < 3U ||
    config_.obstacle_min_points == 0U || config_.max_candidates == 0U)
  {
    throw std::invalid_argument("invalid detector count parameter");
  }
}

std::vector<Candidate> Detector::detect(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & input) const
{
  std::vector<Candidate> evaluated;
  if (!input || input->empty()) {
    return evaluated;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr finite(new pcl::PointCloud<pcl::PointXYZI>());
  finite->reserve(input->size());
  for (const auto & point : input->points) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  if (finite->size() < config_.min_plane_points) {
    return evaluated;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(
    static_cast<float>(config_.voxel_leaf_m),
    static_cast<float>(config_.voxel_leaf_m),
    static_cast<float>(config_.voxel_leaf_m));
  voxel.setInputCloud(finite);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  voxel.filter(*cloud);
  if (cloud->size() < config_.min_plane_points) {
    return evaluated;
  }

  auto search_tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setSearchMethod(search_tree);
  normal_estimator.setKSearch(static_cast<int>(config_.normal_neighbors));
  normal_estimator.compute(*normals);

  pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> region_growing;
  region_growing.setInputCloud(cloud);
  region_growing.setInputNormals(normals);
  region_growing.setSearchMethod(search_tree);
  region_growing.setNumberOfNeighbours(static_cast<unsigned int>(config_.normal_neighbors));
  region_growing.setMinClusterSize(static_cast<int>(config_.min_plane_points));
  region_growing.setSmoothnessThreshold(config_.region_smoothness_deg * kPi / 180.0);
  region_growing.setCurvatureThreshold(config_.curvature_threshold);
  std::vector<pcl::PointIndices> clusters;
  region_growing.extract(clusters);
  std::stable_sort(
    clusters.begin(), clusters.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.indices.size() > rhs.indices.size();
    });

  pcl::PointCloud<pcl::PointXYZI>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  xy_cloud->reserve(cloud->size());
  for (const auto & point : cloud->points) {
    pcl::PointXYZI xy = point;
    xy.z = 0.0F;
    xy_cloud->push_back(xy);
  }
  pcl::KdTreeFLANN<pcl::PointXYZI> xy_tree;
  xy_tree.setInputCloud(xy_cloud);

  const double radius = config_.landing_radius_m;
  const double safety_radius = radius + config_.safety_margin_m;
  const double disc_area = kPi * radius * radius;
  const double max_slope_rad = config_.max_slope_deg * kPi / 180.0;
  std::unordered_set<std::pair<std::int64_t, std::int64_t>, LatticeCellHash>
    evaluated_lattice_cells;

  for (const auto & cluster : clusters) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cluster_cloud->reserve(cluster.indices.size());
    for (const int index : cluster.indices) {
      cluster_cloud->push_back(cloud->points[static_cast<std::size_t>(index)]);
    }

    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(max_slope_rad);
    segmentation.setDistanceThreshold(config_.plane_inlier_threshold_m);
    segmentation.setMaxIterations(200);
    segmentation.setInputCloud(cluster_cloud);
    pcl::PointIndices plane_inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(plane_inliers, coefficients);
    if (plane_inliers.indices.size() < config_.min_plane_points || coefficients.values.size() < 4U) {
      continue;
    }

    Eigen::Vector3d normal(
      coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    const double normal_norm = normal.norm();
    if (normal_norm < 1e-9) {
      continue;
    }
    normal /= normal_norm;
    double plane_d = static_cast<double>(coefficients.values[3]) / normal_norm;
    if (normal.z() < 0.0) {
      normal = -normal;
      plane_d = -plane_d;
    }
    if (std::abs(normal.z()) < 1e-6) {
      continue;
    }
    const double slope_deg = std::acos(std::clamp(normal.z(), 0.0, 1.0)) * 180.0 / kPi;

    pcl::PointXYZI cluster_min;
    pcl::PointXYZI cluster_max;
    pcl::getMinMax3D(*cluster_cloud, cluster_min, cluster_max);
    const auto min_lattice_x = static_cast<std::int64_t>(std::ceil(
        static_cast<double>(cluster_min.x) / config_.candidate_spacing_m));
    const auto max_lattice_x = static_cast<std::int64_t>(std::floor(
        static_cast<double>(cluster_max.x) / config_.candidate_spacing_m));
    const auto min_lattice_y = static_cast<std::int64_t>(std::ceil(
        static_cast<double>(cluster_min.y) / config_.candidate_spacing_m));
    const auto max_lattice_y = static_cast<std::int64_t>(std::floor(
        static_cast<double>(cluster_max.y) / config_.candidate_spacing_m));

    for (std::int64_t lattice_x = min_lattice_x; lattice_x <= max_lattice_x; ++lattice_x) {
      for (std::int64_t lattice_y = min_lattice_y; lattice_y <= max_lattice_y; ++lattice_y) {
        if (!evaluated_lattice_cells.emplace(lattice_x, lattice_y).second) {
          continue;
        }
        Candidate candidate;
        candidate.radius = kRequiredRadiusM;
        candidate.center.x() = static_cast<double>(lattice_x) * config_.candidate_spacing_m;
        candidate.center.y() = static_cast<double>(lattice_y) * config_.candidate_spacing_m;
        candidate.center.z() = -(
          normal.x() * candidate.center.x() + normal.y() * candidate.center.y() + plane_d) /
          normal.z();
        candidate.normal = normal;
        candidate.geometry.slope_deg = slope_deg;

        pcl::PointXYZI query;
        query.x = static_cast<float>(candidate.center.x());
        query.y = static_cast<float>(candidate.center.y());
        query.z = 0.0F;
        std::vector<int> obstacle_search_indices;
        std::vector<float> obstacle_search_squared_distances;
        xy_tree.radiusSearch(
          query, static_cast<float>(config_.obstacle_search_radius_m),
          obstacle_search_indices, obstacle_search_squared_distances);

        std::vector<double> plane_residuals;
        plane_residuals.reserve(obstacle_search_indices.size());
        std::vector<double> terrain_residuals;
        terrain_residuals.reserve(obstacle_search_indices.size());
        std::vector<bool> coverage(config_.angular_bins * config_.radial_bins, false);
        std::vector<bool> clearance_coverage(
          config_.angular_bins * config_.radial_bins, false);
        std::uint32_t obstacle_count = 0U;
        double nearest_obstacle = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < obstacle_search_indices.size(); ++i) {
          const auto & point = cloud->points[
            static_cast<std::size_t>(obstacle_search_indices[i])];
          const double horizontal_radius = std::sqrt(
            static_cast<double>(obstacle_search_squared_distances[i]));
          const Eigen::Vector3d point_vector(point.x, point.y, point.z);
          const double residual = normal.dot(point_vector - candidate.center);
          const double dx = static_cast<double>(point.x) - candidate.center.x();
          const double dy = static_cast<double>(point.y) - candidate.center.y();
          double angle = std::atan2(dy, dx);
          if (angle < 0.0) {
            angle += 2.0 * kPi;
          }
          const auto angular_index = std::min(
            config_.angular_bins - 1U,
            static_cast<std::size_t>(angle / (2.0 * kPi) * config_.angular_bins));
          const auto clearance_radial_index = std::min(
            config_.radial_bins - 1U,
            static_cast<std::size_t>(
              horizontal_radius / config_.obstacle_search_radius_m * config_.radial_bins));
          clearance_coverage[
            clearance_radial_index * config_.angular_bins + angular_index] = true;
          if (horizontal_radius <= radius) {
            if (residual <= config_.obstacle_height_m) {
              terrain_residuals.push_back(residual);
            }
            if (std::abs(residual) <= config_.plane_inlier_threshold_m) {
              plane_residuals.push_back(residual);
              const auto radial_index = std::min(
                config_.radial_bins - 1U,
                static_cast<std::size_t>(horizontal_radius / radius * config_.radial_bins));
              coverage[radial_index * config_.angular_bins + angular_index] = true;
            }
          }
          if (residual > config_.obstacle_height_m) {
            nearest_obstacle = std::min(nearest_obstacle, horizontal_radius);
            if (horizontal_radius <= safety_radius) {
              ++obstacle_count;
            }
          }
        }

        candidate.geometry.obstacle_count = obstacle_count;
        candidate.geometry.nearest_obstacle_clearance_m = nearest_obstacle;
        candidate.geometry.observed_fraction = static_cast<double>(
          std::count(coverage.begin(), coverage.end(), true)) /
          static_cast<double>(coverage.size());
        candidate.geometry.clearance_observed_fraction = static_cast<double>(
          std::count(clearance_coverage.begin(), clearance_coverage.end(), true)) /
          static_cast<double>(clearance_coverage.size());
        candidate.geometry.density_m2 = static_cast<double>(plane_residuals.size()) / disc_area;
        if (!terrain_residuals.empty()) {
          double sum_squared = 0.0;
          for (const double residual : terrain_residuals) {
            sum_squared += residual * residual;
          }
          candidate.geometry.roughness_m = std::sqrt(
            sum_squared / static_cast<double>(terrain_residuals.size()));
          candidate.geometry.relief_m =
            quantile(terrain_residuals, 0.95) - quantile(terrain_residuals, 0.05);
        }

        if (plane_residuals.size() < config_.min_plane_points) {
          candidate.rejection_reason = "insufficient_footprint_points";
        } else if (candidate.geometry.slope_deg > config_.max_slope_deg) {
          candidate.rejection_reason = "slope";
        } else if (candidate.geometry.density_m2 < config_.min_density_m2) {
          candidate.rejection_reason = "density";
        } else if (candidate.geometry.observed_fraction < config_.min_observed_fraction) {
          candidate.rejection_reason = "coverage";
        } else if (candidate.geometry.relief_m > config_.max_relief_m) {
          candidate.rejection_reason = "relief";
        } else if (candidate.geometry.roughness_m > config_.max_roughness_m) {
          candidate.rejection_reason = "roughness";
        } else if (obstacle_count >= config_.obstacle_min_points) {
          candidate.rejection_reason = "obstacle";
        } else {
          candidate.geometry_valid = true;
        }
        evaluated.push_back(std::move(candidate));
      }
    }
  }

  std::stable_sort(
    evaluated.begin(), evaluated.end(), [](const Candidate & lhs, const Candidate & rhs) {
      if (lhs.geometry_valid != rhs.geometry_valid) {
        return lhs.geometry_valid > rhs.geometry_valid;
      }
      const double lhs_quality = lhs.geometry.observed_fraction -
        0.02 * lhs.geometry.slope_deg - lhs.geometry.roughness_m - lhs.geometry.relief_m;
      const double rhs_quality = rhs.geometry.observed_fraction -
        0.02 * rhs.geometry.slope_deg - rhs.geometry.roughness_m - rhs.geometry.relief_m;
      return lhs_quality > rhs_quality;
    });

  std::vector<Candidate> selected;
  selected.reserve(std::min(config_.max_candidates, evaluated.size()));
  for (auto & candidate : evaluated) {
    const bool overlaps = std::any_of(
      selected.begin(), selected.end(), [&](const Candidate & kept) {
        return horizontalDistance(candidate, kept) < config_.nms_distance_m;
      });
    if (!overlaps) {
      selected.push_back(std::move(candidate));
      if (selected.size() >= config_.max_candidates) {
        break;
      }
    }
  }
  return selected;
}

}  // namespace ddlzd
