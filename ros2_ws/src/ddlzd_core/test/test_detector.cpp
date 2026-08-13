#include "ddlzd_core/detector.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

TEST(Detector, ProducesOnlyExactRadiusCandidatesOnDensePlane)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (double x = -5.0; x <= 5.0; x += 0.1) {
    for (double y = -5.0; y <= 5.0; y += 0.1) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(0.002 * std::sin(x) * std::cos(y));
      cloud->push_back(point);
    }
  }

  ddlzd::DetectorConfig config;
  config.min_observed_fraction = 0.60;
  config.max_candidates = 10;
  ddlzd::Detector detector(config);
  const auto candidates = detector.detect(cloud);

  ASSERT_FALSE(candidates.empty());
  for (const auto & candidate : candidates) {
    EXPECT_DOUBLE_EQ(candidate.radius, 2.5);
  }
  EXPECT_TRUE(std::any_of(
    candidates.begin(), candidates.end(),
    [](const ddlzd::Candidate & candidate) {return candidate.geometry_valid;}));
}

TEST(Detector, RejectsAnyRadiusOtherThanTwoPointFiveMetres)
{
  ddlzd::DetectorConfig config;
  config.landing_radius_m = 3.0;
  EXPECT_THROW(ddlzd::Detector detector(config), std::invalid_argument);
}

TEST(Detector, HardRejectsObstacleInsideFootprint)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (double x = -5.0; x <= 5.0; x += 0.1) {
    for (double y = -5.0; y <= 5.0; y += 0.1) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = 0.0F;
      cloud->push_back(point);
    }
  }
  for (int x_index = -2; x_index <= 2; ++x_index) {
    for (int y_index = -2; y_index <= 2; ++y_index) {
      pcl::PointXYZI obstacle;
      obstacle.x = static_cast<float>(0.2 * x_index);
      obstacle.y = static_cast<float>(0.2 * y_index);
      obstacle.z = 1.0F;
      cloud->push_back(obstacle);
    }
  }

  ddlzd::DetectorConfig config;
  config.candidate_spacing_m = 1.0;
  config.nms_distance_m = 0.5;
  config.max_candidates = 500;
  ddlzd::Detector detector(config);
  const auto candidates = detector.detect(cloud);
  const auto center = std::find_if(
    candidates.begin(), candidates.end(), [](const ddlzd::Candidate & candidate) {
      return candidate.center.head<2>().norm() < 1e-6;
    });

  ASSERT_NE(center, candidates.end());
  EXPECT_FALSE(center->geometry_valid);
  EXPECT_EQ(center->rejection_reason, "obstacle");
  EXPECT_GE(center->geometry.obstacle_count, 20U);
}
