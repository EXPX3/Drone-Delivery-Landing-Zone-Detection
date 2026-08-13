#pragma once

#include "ddlzd_core/types.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ddlzd_ros
{

struct SemanticConfig
{
  double exg_low{0.02};
  double exg_high{0.35};
  double vari_low{0.02};
  double vari_high{0.45};
  double saturation_low{0.10};
  double saturation_high{0.75};
  double texture_low{0.015};
  double texture_high{0.18};
  double tree_threshold{0.70};
  double grass_threshold{0.55};
  double occlusion_tolerance_m{0.15};
  std::size_t evidence_angular_bins{36};
  std::size_t evidence_radial_bins{5};
};

struct SemanticProjection
{
  std::vector<float> tree_score;
  std::vector<float> grass_score;
  std::vector<float> texture_score;
  std::vector<std::uint8_t> visible;
  cv::Mat semantic_bgr;
};

class SemanticProjector
{
public:
  explicit SemanticProjector(SemanticConfig config);

  [[nodiscard]] SemanticProjection project(
    const pcl::PointCloud<pcl::PointXYZI> & cloud_target,
    const cv::Mat & rectified_bgr,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const Eigen::Isometry3d & camera_from_target) const;

  [[nodiscard]] ddlzd::CameraEvidence evidenceForCandidate(
    const ddlzd::Candidate & candidate,
    const pcl::PointCloud<pcl::PointXYZI> & cloud_target,
    const SemanticProjection & projection) const;

  [[nodiscard]] bool projectPoint(
    const Eigen::Vector3d & point_target,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const Eigen::Isometry3d & camera_from_target,
    cv::Point & pixel) const;

private:
  SemanticConfig config_;
};

}  // namespace ddlzd_ros
