#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>

namespace ddlzd
{

enum class Category : std::uint8_t
{
  kUnknown = 0,
  kSafest = 1,
  kSafe = 2,
  kRisky = 3,
};

struct GeometryMetrics
{
  double slope_deg{0.0};
  double density_m2{0.0};
  double relief_m{0.0};
  double roughness_m{0.0};
  std::uint32_t obstacle_count{0};
  double nearest_obstacle_clearance_m{std::numeric_limits<double>::infinity()};
  double observed_fraction{0.0};
  double clearance_observed_fraction{0.0};
};

struct CameraEvidence
{
  bool valid{false};
  double coverage_fraction{0.0};
  double tree_fraction{0.0};
  double tree_score{0.0};
  double grass_fraction{0.0};
  double texture_risk{0.0};
};

struct Candidate
{
  std::uint64_t id{0};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double radius{2.5};
  bool geometry_valid{false};
  std::string rejection_reason;
  GeometryMetrics geometry;
  CameraEvidence camera;
  double risk_score{1.0};
  Category category{Category::kUnknown};
  bool temporally_stable{false};
  std::uint32_t consecutive_observations{0};
};

}  // namespace ddlzd
