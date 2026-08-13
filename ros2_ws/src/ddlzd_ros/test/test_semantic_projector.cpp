#include "ddlzd_ros/semantic_projector.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

namespace
{

sensor_msgs::msg::CameraInfo cameraInfo()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 100U;
  info.height = 100U;
  info.k = {50.0, 0.0, 50.0, 0.0, 50.0, 50.0, 0.0, 0.0, 1.0};
  info.p = {50.0, 0.0, 50.0, 0.0, 0.0, 50.0, 50.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

TEST(SemanticProjector, UsesRectifiedProjectionMatrix)
{
  ddlzd_ros::SemanticProjector projector(ddlzd_ros::SemanticConfig{});
  auto info = cameraInfo();
  info.k[2] = 5.0;
  cv::Point pixel;

  ASSERT_TRUE(projector.projectPoint(
      Eigen::Vector3d(0.0, 0.0, 5.0), info, Eigen::Isometry3d::Identity(), pixel));
  EXPECT_EQ(pixel.x, 50);
  EXPECT_EQ(pixel.y, 50);
}

}  // namespace

TEST(SemanticProjector, SmoothGreenSurfaceIsGrassRatherThanTree)
{
  ddlzd_ros::SemanticProjector projector(ddlzd_ros::SemanticConfig{});
  cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 220, 0));
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (double x = -1.0; x <= 1.0; x += 0.1) {
    for (double y = -1.0; y <= 1.0; y += 0.1) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = 5.0F;
      cloud.push_back(point);
    }
  }
  const auto projection = projector.project(
    cloud, image, cameraInfo(), Eigen::Isometry3d::Identity());
  ddlzd::Candidate candidate;
  candidate.center = Eigen::Vector3d(0.0, 0.0, 5.0);
  candidate.radius = 2.5;
  const auto evidence = projector.evidenceForCandidate(candidate, cloud, projection);

  EXPECT_TRUE(evidence.valid);
  EXPECT_GT(evidence.coverage_fraction, 0.95);
  EXPECT_LT(evidence.tree_fraction, 0.05);
  EXPECT_GT(evidence.grass_fraction, 0.95);
}

TEST(SemanticProjector, DepthBufferRejectsOccludedPoint)
{
  ddlzd_ros::SemanticProjector projector(ddlzd_ros::SemanticConfig{});
  cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 220, 0));
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI near_point;
  near_point.z = 5.0F;
  pcl::PointXYZI far_point;
  far_point.z = 8.0F;
  cloud.push_back(near_point);
  cloud.push_back(far_point);

  const auto projection = projector.project(
    cloud, image, cameraInfo(), Eigen::Isometry3d::Identity());

  ASSERT_EQ(projection.visible.size(), 2U);
  EXPECT_EQ(projection.visible[0], 1U);
  EXPECT_EQ(projection.visible[1], 0U);
}
