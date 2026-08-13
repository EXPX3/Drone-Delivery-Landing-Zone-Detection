#include "ddlzd_core/risk.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ddlzd
{
namespace
{

double increasingRisk(const double value, const double good, const double bad)
{
  if (!(bad > good)) {
    throw std::invalid_argument("risk upper bound must exceed lower bound");
  }
  return std::clamp((value - good) / (bad - good), 0.0, 1.0);
}

}  // namespace

RiskClassifier::RiskClassifier(RiskConfig config)
: config_(std::move(config))
{
  if (!(config_.safest_threshold >= 0.0 &&
    config_.safest_threshold < config_.safe_threshold && config_.safe_threshold <= 1.0))
  {
    throw std::invalid_argument("risk thresholds must be ordered within [0, 1]");
  }
  if (!(config_.minimum_camera_coverage >= 0.0 && config_.minimum_camera_coverage <= 1.0 &&
    config_.minimum_clearance_coverage >= 0.0 &&
    config_.minimum_clearance_coverage <= 1.0 &&
    config_.slope_good_deg >= 0.0 && config_.slope_bad_deg > config_.slope_good_deg &&
    config_.relief_good_m >= 0.0 && config_.relief_bad_m > config_.relief_good_m &&
    config_.roughness_good_m >= 0.0 &&
    config_.roughness_bad_m > config_.roughness_good_m &&
    config_.clearance_bad_m >= 0.0 &&
    config_.clearance_good_m > config_.clearance_bad_m &&
    config_.observed_bad_fraction >= 0.0 &&
    config_.observed_good_fraction > config_.observed_bad_fraction &&
    config_.observed_good_fraction <= 1.0 && config_.obstacle_bad_count > 0.0))
  {
    throw std::invalid_argument("invalid risk normalizer or coverage parameter");
  }
}

void RiskClassifier::classify(Candidate & candidate) const
{
  if (!candidate.geometry_valid) {
    candidate.risk_score = 1.0;
    candidate.category = Category::kRisky;
    return;
  }

  const auto & g = candidate.geometry;
  const bool camera_usable = candidate.camera.valid &&
    candidate.camera.coverage_fraction >= config_.minimum_camera_coverage;
  const bool clearance_usable =
    g.clearance_observed_fraction >= config_.minimum_clearance_coverage;
  if (!camera_usable || !clearance_usable) {
    candidate.risk_score = std::numeric_limits<double>::quiet_NaN();
    candidate.category = Category::kUnknown;
    return;
  }

  const double obstacle_risk = std::clamp(
    static_cast<double>(g.obstacle_count) / config_.obstacle_bad_count, 0.0, 1.0);
  const double clearance_risk = std::isfinite(g.nearest_obstacle_clearance_m) ?
    1.0 - increasingRisk(
    g.nearest_obstacle_clearance_m, config_.clearance_bad_m, config_.clearance_good_m) : 0.0;
  const double relief_risk = increasingRisk(
    g.relief_m, config_.relief_good_m, config_.relief_bad_m);
  const double roughness_risk = increasingRisk(
    g.roughness_m, config_.roughness_good_m, config_.roughness_bad_m);
  const double slope_risk = increasingRisk(
    g.slope_deg, config_.slope_good_deg, config_.slope_bad_deg);
  const double coverage_risk = 1.0 - increasingRisk(
    g.observed_fraction, config_.observed_bad_fraction, config_.observed_good_fraction);

  const double tree_risk = std::clamp(
    0.55 * candidate.camera.tree_fraction + 0.45 * candidate.camera.tree_score, 0.0, 1.0);
  const double texture_risk = candidate.camera.texture_risk;
  const double grass_bonus = candidate.camera.grass_fraction;

  candidate.risk_score = std::clamp(
    0.23 * obstacle_risk +
    0.18 * clearance_risk +
    0.17 * tree_risk +
    0.14 * relief_risk +
    0.10 * roughness_risk +
    0.07 * texture_risk +
    0.06 * slope_risk +
    0.05 * coverage_risk -
    0.10 * grass_bonus,
    0.0, 1.0);

  if (candidate.risk_score <= config_.safest_threshold) {
    candidate.category = Category::kSafest;
  } else if (candidate.risk_score <= config_.safe_threshold) {
    candidate.category = Category::kSafe;
  } else {
    candidate.category = Category::kRisky;
  }
}

}  // namespace ddlzd
