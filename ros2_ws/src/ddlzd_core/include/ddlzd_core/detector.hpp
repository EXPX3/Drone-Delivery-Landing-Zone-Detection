#pragma once

#include "ddlzd_core/types.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <vector>

namespace ddlzd
{

struct DetectorConfig
{
  // This live variant is intentionally fixed to the requested 2.5 m radius.
  double landing_radius_m{2.5};
  double safety_margin_m{0.0};
  double obstacle_search_radius_m{12.0};
  double candidate_spacing_m{0.75};
  double nms_distance_m{1.0};
  double voxel_leaf_m{0.10};
  double plane_inlier_threshold_m{0.08};
  double region_smoothness_deg{6.0};
  double curvature_threshold{0.08};
  double obstacle_height_m{0.50};
  double max_slope_deg{12.0};
  double min_density_m2{8.0};
  double max_relief_m{5.0};
  double max_roughness_m{2.0};
  double min_observed_fraction{0.70};
  std::size_t min_plane_points{120};
  std::size_t normal_neighbors{30};
  std::size_t obstacle_min_points{20};
  std::size_t angular_bins{36};
  std::size_t radial_bins{5};
  std::size_t max_candidates{200};
};

class Detector
{
public:
  explicit Detector(DetectorConfig config);

  [[nodiscard]] std::vector<Candidate> detect(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & input) const;

  [[nodiscard]] const DetectorConfig & config() const noexcept {return config_;}

private:
  DetectorConfig config_;
};

}  // namespace ddlzd
