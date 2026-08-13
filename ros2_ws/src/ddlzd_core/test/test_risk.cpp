#include "ddlzd_core/risk.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

TEST(RiskClassifier, FailsClosedWithoutCameraEvidence)
{
  ddlzd::Candidate candidate;
  candidate.geometry_valid = true;
  candidate.geometry.observed_fraction = 1.0;
  candidate.geometry.clearance_observed_fraction = 1.0;
  candidate.geometry.nearest_obstacle_clearance_m = 10.0;

  ddlzd::RiskClassifier classifier(ddlzd::RiskConfig{});
  classifier.classify(candidate);

  EXPECT_EQ(candidate.category, ddlzd::Category::kUnknown);
  EXPECT_TRUE(std::isnan(candidate.risk_score));
}

TEST(RiskClassifier, ClearObservedCandidateIsSafest)
{
  ddlzd::Candidate candidate;
  candidate.geometry_valid = true;
  candidate.geometry.observed_fraction = 1.0;
  candidate.geometry.clearance_observed_fraction = 1.0;
  candidate.geometry.nearest_obstacle_clearance_m = 10.0;
  candidate.geometry.slope_deg = 0.0;
  candidate.geometry.relief_m = 0.0;
  candidate.geometry.roughness_m = 0.0;
  candidate.camera.valid = true;
  candidate.camera.coverage_fraction = 1.0;
  candidate.camera.grass_fraction = 0.8;

  ddlzd::RiskClassifier classifier(ddlzd::RiskConfig{});
  classifier.classify(candidate);

  EXPECT_EQ(candidate.category, ddlzd::Category::kSafest);
  EXPECT_LE(candidate.risk_score, 0.33);
}

TEST(RiskClassifier, InsufficientCameraCoverageHasNoNumericScore)
{
  ddlzd::Candidate candidate;
  candidate.geometry_valid = true;
  candidate.geometry.observed_fraction = 1.0;
  candidate.geometry.clearance_observed_fraction = 1.0;
  candidate.geometry.nearest_obstacle_clearance_m = 10.0;
  candidate.camera.valid = true;
  candidate.camera.coverage_fraction = 0.54;

  ddlzd::RiskClassifier classifier(ddlzd::RiskConfig{});
  classifier.classify(candidate);

  EXPECT_EQ(candidate.category, ddlzd::Category::kUnknown);
  EXPECT_TRUE(std::isnan(candidate.risk_score));
}

TEST(RiskClassifier, InsufficientClearanceCoverageHasNoNumericScore)
{
  ddlzd::Candidate candidate;
  candidate.geometry_valid = true;
  candidate.geometry.observed_fraction = 1.0;
  candidate.geometry.clearance_observed_fraction = 0.54;
  candidate.geometry.nearest_obstacle_clearance_m = 10.0;
  candidate.camera.valid = true;
  candidate.camera.coverage_fraction = 1.0;

  ddlzd::RiskClassifier classifier(ddlzd::RiskConfig{});
  classifier.classify(candidate);

  EXPECT_EQ(candidate.category, ddlzd::Category::kUnknown);
  EXPECT_TRUE(std::isnan(candidate.risk_score));
}

TEST(RiskClassifier, GeometryRejectionIsRisky)
{
  ddlzd::Candidate candidate;
  candidate.geometry_valid = false;
  candidate.rejection_reason = "obstacle";

  ddlzd::RiskClassifier classifier(ddlzd::RiskConfig{});
  classifier.classify(candidate);

  EXPECT_EQ(candidate.category, ddlzd::Category::kRisky);
  EXPECT_DOUBLE_EQ(candidate.risk_score, 1.0);
}

TEST(RiskClassifier, RejectsInvalidNormalizationRanges)
{
  ddlzd::RiskConfig config;
  config.relief_bad_m = config.relief_good_m;
  EXPECT_THROW(ddlzd::RiskClassifier classifier(config), std::invalid_argument);
}
