#include "ddlzd_ros/semantic_projector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ddlzd_ros
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

cv::Mat normalizedRange(const cv::Mat & value, const double low, const double high)
{
  if (!(high > low)) {
    throw std::invalid_argument("semantic upper threshold must exceed lower threshold");
  }
  cv::Mat output;
  value.convertTo(output, CV_32F, 1.0 / (high - low), -low / (high - low));
  cv::max(output, 0.0, output);
  cv::min(output, 1.0, output);
  return output;
}

}  // namespace

SemanticProjector::SemanticProjector(SemanticConfig config)
: config_(std::move(config))
{
  if (!(config_.exg_high > config_.exg_low && config_.vari_high > config_.vari_low &&
    config_.saturation_high > config_.saturation_low &&
    config_.texture_high > config_.texture_low && config_.tree_threshold >= 0.0 &&
    config_.tree_threshold <= 1.0 && config_.grass_threshold >= 0.0 &&
    config_.grass_threshold <= 1.0 && config_.occlusion_tolerance_m >= 0.0) ||
    config_.evidence_angular_bins == 0U || config_.evidence_radial_bins == 0U)
  {
    throw std::invalid_argument("invalid semantic projection configuration");
  }
}

bool SemanticProjector::projectPoint(
  const Eigen::Vector3d & point_target,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & camera_from_target,
  cv::Point & pixel) const
{
  const Eigen::Vector3d camera_point = camera_from_target * point_target;
  if (!(camera_point.z() > 1e-6)) {
    return false;
  }
  const double image_x = camera_info.p[0] * camera_point.x() +
    camera_info.p[1] * camera_point.y() + camera_info.p[2] * camera_point.z() + camera_info.p[3];
  const double image_y = camera_info.p[4] * camera_point.x() +
    camera_info.p[5] * camera_point.y() + camera_info.p[6] * camera_point.z() + camera_info.p[7];
  const double image_w = camera_info.p[8] * camera_point.x() +
    camera_info.p[9] * camera_point.y() + camera_info.p[10] * camera_point.z() + camera_info.p[11];
  if (!(image_w > 1e-9)) {
    return false;
  }
  const double u = image_x / image_w;
  const double v = image_y / image_w;
  if (u < 0.0 || v < 0.0 || u >= static_cast<double>(camera_info.width) ||
    v >= static_cast<double>(camera_info.height))
  {
    return false;
  }
  pixel.x = static_cast<int>(std::lround(u));
  pixel.y = static_cast<int>(std::lround(v));
  pixel.x = std::clamp(pixel.x, 0, static_cast<int>(camera_info.width) - 1);
  pixel.y = std::clamp(pixel.y, 0, static_cast<int>(camera_info.height) - 1);
  return true;
}

SemanticProjection SemanticProjector::project(
  const pcl::PointCloud<pcl::PointXYZI> & cloud_target,
  const cv::Mat & rectified_bgr,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & camera_from_target) const
{
  if (rectified_bgr.empty() || rectified_bgr.type() != CV_8UC3) {
    throw std::invalid_argument("semantic input must be a non-empty rectified BGR8 image");
  }
  if (camera_info.width != static_cast<std::uint32_t>(rectified_bgr.cols) ||
    camera_info.height != static_cast<std::uint32_t>(rectified_bgr.rows) ||
    camera_info.p[0] <= 0.0 || camera_info.p[5] <= 0.0 || camera_info.p[10] <= 0.0)
  {
    throw std::invalid_argument("CameraInfo does not match the rectified image");
  }

  cv::Mat bgr_float;
  rectified_bgr.convertTo(bgr_float, CV_32FC3, 1.0 / 255.0);
  std::vector<cv::Mat> bgr;
  cv::split(bgr_float, bgr);
  const cv::Mat & blue = bgr[0];
  const cv::Mat & green = bgr[1];
  const cv::Mat & red = bgr[2];

  cv::Mat hsv;
  cv::cvtColor(bgr_float, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> hsv_channels;
  cv::split(hsv, hsv_channels);
  cv::Mat saturation = hsv_channels[1];

  cv::Mat gray;
  cv::cvtColor(bgr_float, gray, cv::COLOR_BGR2GRAY);
  cv::Mat mean;
  cv::Mat mean_square;
  cv::boxFilter(gray, mean, CV_32F, cv::Size(11, 11), cv::Point(-1, -1), true);
  cv::boxFilter(gray.mul(gray), mean_square, CV_32F, cv::Size(11, 11), cv::Point(-1, -1), true);
  cv::Mat variance = mean_square - mean.mul(mean);
  cv::max(variance, 0.0, variance);
  cv::Mat texture;
  cv::sqrt(variance, texture);

  const cv::Mat exg = 2.0F * green - red - blue;
  cv::Mat denominator = green + red - blue;
  cv::Mat vari(denominator.size(), CV_32F);
  for (int row = 0; row < denominator.rows; ++row) {
    const auto * denominator_row = denominator.ptr<float>(row);
    const auto * green_row = green.ptr<float>(row);
    const auto * red_row = red.ptr<float>(row);
    auto * vari_row = vari.ptr<float>(row);
    for (int column = 0; column < denominator.cols; ++column) {
      const float divisor = std::abs(denominator_row[column]) >= 1e-6F ?
        denominator_row[column] : std::copysign(1e-6F, denominator_row[column]);
      vari_row[column] = (green_row[column] - red_row[column]) / divisor;
    }
  }
  cv::Mat green_score =
    0.45 * normalizedRange(exg, config_.exg_low, config_.exg_high) +
    0.35 * normalizedRange(vari, config_.vari_low, config_.vari_high) +
    0.20 * normalizedRange(saturation, config_.saturation_low, config_.saturation_high);
  cv::Mat texture_score = normalizedRange(texture, config_.texture_low, config_.texture_high);
  cv::Mat dark_score = 1.0F - gray;
  cv::Mat tree_score = 0.58F * green_score + 0.28F * texture_score + 0.14F * dark_score;
  cv::Mat grass_score = 0.72F * green_score + 0.18F * (1.0F - texture_score) + 0.10F * gray;
  cv::Mat smoothed_tree;
  cv::Mat smoothed_grass;
  cv::bilateralFilter(tree_score, smoothed_tree, 7, 0.08, 3.0);
  cv::bilateralFilter(grass_score, smoothed_grass, 7, 0.08, 3.0);
  tree_score = smoothed_tree;
  grass_score = smoothed_grass;

  SemanticProjection result;
  result.tree_score.assign(cloud_target.size(), 0.0F);
  result.grass_score.assign(cloud_target.size(), 0.0F);
  result.texture_score.assign(cloud_target.size(), 0.0F);
  result.visible.assign(cloud_target.size(), 0U);
  result.semantic_bgr = rectified_bgr.clone();

  const std::size_t pixel_count = static_cast<std::size_t>(rectified_bgr.rows * rectified_bgr.cols);
  std::vector<double> z_buffer(pixel_count, std::numeric_limits<double>::infinity());
  std::vector<cv::Point> projected(cloud_target.size(), cv::Point(-1, -1));
  std::vector<double> depths(cloud_target.size(), 0.0);
  for (std::size_t i = 0; i < cloud_target.size(); ++i) {
    const auto & point = cloud_target.points[i];
    const Eigen::Vector3d target_point(point.x, point.y, point.z);
    cv::Point pixel;
    if (!projectPoint(target_point, camera_info, camera_from_target, pixel)) {
      continue;
    }
    const double depth = (camera_from_target * target_point).z();
    projected[i] = pixel;
    depths[i] = depth;
    const std::size_t offset = static_cast<std::size_t>(pixel.y * rectified_bgr.cols + pixel.x);
    z_buffer[offset] = std::min(z_buffer[offset], depth);
  }

  for (std::size_t i = 0; i < cloud_target.size(); ++i) {
    const cv::Point pixel = projected[i];
    if (pixel.x < 0) {
      continue;
    }
    const std::size_t offset = static_cast<std::size_t>(pixel.y * rectified_bgr.cols + pixel.x);
    if (depths[i] > z_buffer[offset] + config_.occlusion_tolerance_m) {
      continue;
    }
    result.visible[i] = 1U;
    result.tree_score[i] = tree_score.at<float>(pixel.y, pixel.x);
    result.grass_score[i] = grass_score.at<float>(pixel.y, pixel.x);
    result.texture_score[i] = texture_score.at<float>(pixel.y, pixel.x);
    const cv::Scalar color = result.tree_score[i] >= config_.tree_threshold ?
      cv::Scalar(0, 0, 255) :
      (result.grass_score[i] >= config_.grass_threshold ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255));
    cv::circle(result.semantic_bgr, pixel, 1, color, -1, cv::LINE_AA);
  }
  return result;
}

ddlzd::CameraEvidence SemanticProjector::evidenceForCandidate(
  const ddlzd::Candidate & candidate,
  const pcl::PointCloud<pcl::PointXYZI> & cloud_target,
  const SemanticProjection & projection) const
{
  ddlzd::CameraEvidence evidence;
  if (projection.visible.size() != cloud_target.size()) {
    return evidence;
  }

  const std::size_t bin_count = config_.evidence_angular_bins * config_.evidence_radial_bins;
  std::vector<std::uint8_t> observed_bins(bin_count, 0U);
  std::vector<std::uint8_t> visible_bins(bin_count, 0U);
  std::size_t visible_points = 0U;
  std::size_t tree_points = 0U;
  std::size_t grass_points = 0U;
  double texture_sum = 0.0;
  double tree_score_sum = 0.0;
  for (std::size_t i = 0; i < cloud_target.size(); ++i) {
    const auto & point = cloud_target.points[i];
    const double dx = static_cast<double>(point.x) - candidate.center.x();
    const double dy = static_cast<double>(point.y) - candidate.center.y();
    const double radius = std::hypot(dx, dy);
    if (radius > candidate.radius) {
      continue;
    }
    double angle = std::atan2(dy, dx);
    if (angle < 0.0) {
      angle += 2.0 * kPi;
    }
    const auto angular_index = std::min(
      config_.evidence_angular_bins - 1U,
      static_cast<std::size_t>(angle / (2.0 * kPi) * config_.evidence_angular_bins));
    const auto radial_index = std::min(
      config_.evidence_radial_bins - 1U,
      static_cast<std::size_t>(radius / candidate.radius * config_.evidence_radial_bins));
    const std::size_t bin = radial_index * config_.evidence_angular_bins + angular_index;
    observed_bins[bin] = 1U;
    if (projection.visible[i] == 0U) {
      continue;
    }
    visible_bins[bin] = 1U;
    ++visible_points;
    tree_points += projection.tree_score[i] >= config_.tree_threshold ? 1U : 0U;
    grass_points += projection.grass_score[i] >= config_.grass_threshold &&
      projection.tree_score[i] < config_.tree_threshold ? 1U : 0U;
    texture_sum += projection.texture_score[i];
    tree_score_sum += projection.tree_score[i];
  }

  const std::size_t observed_count = static_cast<std::size_t>(
    std::count(observed_bins.begin(), observed_bins.end(), 1U));
  const std::size_t visible_count = static_cast<std::size_t>(
    std::count(visible_bins.begin(), visible_bins.end(), 1U));
  evidence.coverage_fraction = observed_count > 0U ?
    static_cast<double>(visible_count) / static_cast<double>(observed_count) : 0.0;
  evidence.valid = visible_points > 0U;
  if (visible_points > 0U) {
    evidence.tree_fraction = static_cast<double>(tree_points) / static_cast<double>(visible_points);
    evidence.tree_score = tree_score_sum / static_cast<double>(visible_points);
    evidence.grass_fraction = static_cast<double>(grass_points) / static_cast<double>(visible_points);
    evidence.texture_risk = texture_sum / static_cast<double>(visible_points);
  }
  return evidence;
}

}  // namespace ddlzd_ros
