#include "ddlzd_ros/live_landing_zone_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace ddlzd_ros
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  return std::any_of(
    cloud.fields.begin(), cloud.fields.end(), [&](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
}

const sensor_msgs::msg::PointField & requireField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name,
  const std::uint8_t datatype)
{
  const auto iterator = std::find_if(
    cloud.fields.begin(), cloud.fields.end(), [&](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
  if (iterator == cloud.fields.end() || iterator->datatype != datatype || iterator->count != 1U) {
    throw std::invalid_argument("PointCloud2 field '" + name + "' has an invalid or missing type");
  }
  return *iterator;
}

bool hostIsBigEndian()
{
  const std::uint16_t marker = 0x0102U;
  return *reinterpret_cast<const std::uint8_t *>(&marker) == 0x01U;
}

template<typename T>
T readScalar(const std::uint8_t * data, const bool swap)
{
  T result;
  std::memcpy(&result, data, sizeof(T));
  if (swap) {
    auto * bytes = reinterpret_cast<std::uint8_t *>(&result);
    std::reverse(bytes, bytes + sizeof(T));
  }
  return result;
}

std_msgs::msg::ColorRGBA categoryColor(const ddlzd::Category category)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.90F;
  switch (category) {
    case ddlzd::Category::kSafest:
      color.g = 1.0F;
      break;
    case ddlzd::Category::kSafe:
      color.r = 1.0F;
      color.g = 0.55F;
      break;
    case ddlzd::Category::kRisky:
      color.r = 1.0F;
      break;
    case ddlzd::Category::kUnknown:
    default:
      color.r = 0.55F;
      color.g = 0.55F;
      color.b = 0.55F;
      break;
  }
  return color;
}

std::string categoryName(const ddlzd::Category category)
{
  switch (category) {
    case ddlzd::Category::kSafest: return "safest";
    case ddlzd::Category::kSafe: return "safe";
    case ddlzd::Category::kRisky: return "risky";
    case ddlzd::Category::kUnknown:
    default: return "unknown";
  }
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

Eigen::Vector3d pointOnHorizontalFootprint(
  const ddlzd::Candidate & candidate, const double angle, const double normal_offset = 0.0)
{
  const double dx = candidate.radius * std::cos(angle);
  const double dy = candidate.radius * std::sin(angle);
  const double dz = -(candidate.normal.x() * dx + candidate.normal.y() * dy) /
    candidate.normal.z();
  return candidate.center + Eigen::Vector3d(dx, dy, dz) + normal_offset * candidate.normal;
}

std::size_t positiveSize(const int value, const std::string & parameter)
{
  if (value <= 0) {
    throw std::invalid_argument(parameter + " must be a positive integer");
  }
  return static_cast<std::size_t>(value);
}

}  // namespace

LiveLandingZoneNode::LiveLandingZoneNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("live_landing_zone", options)
{
}

LiveLandingZoneNode::~LiveLandingZoneNode()
{
  stopWorker();
}

ddlzd::DetectorConfig LiveLandingZoneNode::loadDetectorConfig()
{
  ddlzd::DetectorConfig config;
  config.safety_margin_m = declare_parameter<double>("detector.safety_margin_m", config.safety_margin_m);
  config.obstacle_search_radius_m = declare_parameter<double>(
    "detector.obstacle_search_radius_m", config.obstacle_search_radius_m);
  config.candidate_spacing_m = declare_parameter<double>("detector.candidate_spacing_m", config.candidate_spacing_m);
  config.nms_distance_m = declare_parameter<double>("detector.nms_distance_m", config.nms_distance_m);
  config.voxel_leaf_m = declare_parameter<double>("detector.voxel_leaf_m", config.voxel_leaf_m);
  config.plane_inlier_threshold_m = declare_parameter<double>(
    "detector.plane_inlier_threshold_m", config.plane_inlier_threshold_m);
  config.region_smoothness_deg = declare_parameter<double>(
    "detector.region_smoothness_deg", config.region_smoothness_deg);
  config.curvature_threshold = declare_parameter<double>(
    "detector.curvature_threshold", config.curvature_threshold);
  config.obstacle_height_m = declare_parameter<double>("detector.obstacle_height_m", config.obstacle_height_m);
  config.max_slope_deg = declare_parameter<double>("detector.max_slope_deg", config.max_slope_deg);
  config.min_density_m2 = declare_parameter<double>("detector.min_density_m2", config.min_density_m2);
  config.max_relief_m = declare_parameter<double>("detector.max_relief_m", config.max_relief_m);
  config.max_roughness_m = declare_parameter<double>("detector.max_roughness_m", config.max_roughness_m);
  config.min_observed_fraction = declare_parameter<double>(
    "detector.min_observed_fraction", config.min_observed_fraction);
  config.min_plane_points = positiveSize(
    declare_parameter<int>("detector.min_plane_points", static_cast<int>(config.min_plane_points)),
    "detector.min_plane_points");
  config.normal_neighbors = positiveSize(
    declare_parameter<int>("detector.normal_neighbors", static_cast<int>(config.normal_neighbors)),
    "detector.normal_neighbors");
  config.obstacle_min_points = positiveSize(
    declare_parameter<int>("detector.obstacle_min_points", static_cast<int>(config.obstacle_min_points)),
    "detector.obstacle_min_points");
  config.angular_bins = positiveSize(
    declare_parameter<int>("detector.angular_bins", static_cast<int>(config.angular_bins)),
    "detector.angular_bins");
  config.radial_bins = positiveSize(
    declare_parameter<int>("detector.radial_bins", static_cast<int>(config.radial_bins)),
    "detector.radial_bins");
  config.max_candidates = positiveSize(
    declare_parameter<int>("detector.max_candidates", static_cast<int>(config.max_candidates)),
    "detector.max_candidates");
  return config;
}

ddlzd::RiskConfig LiveLandingZoneNode::loadRiskConfig()
{
  ddlzd::RiskConfig config;
  config.safest_threshold = declare_parameter<double>("risk.safest_threshold", config.safest_threshold);
  config.safe_threshold = declare_parameter<double>("risk.safe_threshold", config.safe_threshold);
  config.minimum_camera_coverage = declare_parameter<double>(
    "risk.minimum_camera_coverage", config.minimum_camera_coverage);
  config.minimum_clearance_coverage = declare_parameter<double>(
    "risk.minimum_clearance_coverage", config.minimum_clearance_coverage);
  config.slope_good_deg = declare_parameter<double>("risk.slope_good_deg", config.slope_good_deg);
  config.slope_bad_deg = declare_parameter<double>("risk.slope_bad_deg", config.slope_bad_deg);
  config.relief_good_m = declare_parameter<double>("risk.relief_good_m", config.relief_good_m);
  config.relief_bad_m = declare_parameter<double>("risk.relief_bad_m", config.relief_bad_m);
  config.roughness_good_m = declare_parameter<double>("risk.roughness_good_m", config.roughness_good_m);
  config.roughness_bad_m = declare_parameter<double>("risk.roughness_bad_m", config.roughness_bad_m);
  config.clearance_bad_m = declare_parameter<double>("risk.clearance_bad_m", config.clearance_bad_m);
  config.clearance_good_m = declare_parameter<double>("risk.clearance_good_m", config.clearance_good_m);
  config.obstacle_bad_count = declare_parameter<double>("risk.obstacle_bad_count", config.obstacle_bad_count);
  config.observed_bad_fraction = declare_parameter<double>(
    "risk.observed_bad_fraction", config.observed_bad_fraction);
  config.observed_good_fraction = declare_parameter<double>(
    "risk.observed_good_fraction", config.observed_good_fraction);
  safest_threshold_ = config.safest_threshold;
  safe_threshold_ = config.safe_threshold;
  minimum_camera_coverage_ = config.minimum_camera_coverage;
  return config;
}

SemanticConfig LiveLandingZoneNode::loadSemanticConfig()
{
  SemanticConfig config;
  config.exg_low = declare_parameter<double>("semantic.exg_low", config.exg_low);
  config.exg_high = declare_parameter<double>("semantic.exg_high", config.exg_high);
  config.vari_low = declare_parameter<double>("semantic.vari_low", config.vari_low);
  config.vari_high = declare_parameter<double>("semantic.vari_high", config.vari_high);
  config.saturation_low = declare_parameter<double>("semantic.saturation_low", config.saturation_low);
  config.saturation_high = declare_parameter<double>("semantic.saturation_high", config.saturation_high);
  config.texture_low = declare_parameter<double>("semantic.texture_low", config.texture_low);
  config.texture_high = declare_parameter<double>("semantic.texture_high", config.texture_high);
  config.tree_threshold = declare_parameter<double>("semantic.tree_threshold", config.tree_threshold);
  config.grass_threshold = declare_parameter<double>("semantic.grass_threshold", config.grass_threshold);
  config.occlusion_tolerance_m = declare_parameter<double>(
    "semantic.occlusion_tolerance_m", config.occlusion_tolerance_m);
  config.evidence_angular_bins = positiveSize(
    declare_parameter<int>(
      "semantic.evidence_angular_bins", static_cast<int>(config.evidence_angular_bins)),
    "semantic.evidence_angular_bins");
  config.evidence_radial_bins = positiveSize(
    declare_parameter<int>(
      "semantic.evidence_radial_bins", static_cast<int>(config.evidence_radial_bins)),
    "semantic.evidence_radial_bins");
  return config;
}

LiveLandingZoneNode::CallbackReturn LiveLandingZoneNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    point_cloud_topic_ = declare_parameter<std::string>("point_cloud_topic", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "");
    target_frame_ = declare_parameter<std::string>("target_frame", "");
    input_is_motion_compensated_ = declare_parameter<bool>("input_is_motion_compensated", false);
    if (point_cloud_topic_.empty() || image_topic_.empty() || camera_info_topic_.empty() ||
      target_frame_.empty())
    {
      throw std::invalid_argument("point_cloud_topic, image_topic, camera_info_topic and target_frame are required");
    }
    rolling_window_sec_ = declare_parameter<double>("rolling_window_sec", rolling_window_sec_);
    detection_period_sec_ = declare_parameter<double>("detection_period_sec", detection_period_sec_);
    local_map_radius_m_ = declare_parameter<double>("local_map_radius_m", local_map_radius_m_);
    sync_tolerance_sec_ = declare_parameter<double>("sync_tolerance_sec", sync_tolerance_sec_);
    image_buffer_sec_ = declare_parameter<double>("image_buffer_sec", image_buffer_sec_);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", max_cloud_age_sec_);
    max_scan_duration_sec_ = declare_parameter<double>(
      "max_scan_duration_sec", max_scan_duration_sec_);
    deskew_tf_sampling_sec_ = declare_parameter<double>(
      "deskew_tf_sampling_sec", deskew_tf_sampling_sec_);
    track_match_distance_m_ = declare_parameter<double>("tracking.match_distance_m", track_match_distance_m_);
    track_timeout_sec_ = declare_parameter<double>("tracking.timeout_sec", track_timeout_sec_);
    risk_filter_alpha_ = declare_parameter<double>("tracking.risk_filter_alpha", risk_filter_alpha_);
    const auto minimum_stable_observations = positiveSize(
      declare_parameter<int>(
        "tracking.minimum_stable_observations", static_cast<int>(minimum_stable_observations_)),
      "tracking.minimum_stable_observations");
    if (minimum_stable_observations > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("tracking.minimum_stable_observations exceeds uint32 range");
    }
    minimum_stable_observations_ = static_cast<std::uint32_t>(minimum_stable_observations);
    const auto detector_config = loadDetectorConfig();
    const auto risk_config = loadRiskConfig();
    const auto semantic_config = loadSemanticConfig();
    if (rolling_window_sec_ <= 0.0 || detection_period_sec_ <= 0.0 ||
      local_map_radius_m_ <= detector_config.landing_radius_m + detector_config.safety_margin_m ||
      local_map_radius_m_ <= detector_config.obstacle_search_radius_m ||
      detector_config.obstacle_search_radius_m < risk_config.clearance_good_m ||
      sync_tolerance_sec_ < 0.0 || image_buffer_sec_ <= 0.0 ||
      image_buffer_sec_ < sync_tolerance_sec_ ||
      tf_timeout_sec_ <= 0.0 || max_cloud_age_sec_ <= 0.0 ||
      max_scan_duration_sec_ <= 0.0 ||
      track_match_distance_m_ <= 0.0 || track_timeout_sec_ <= 0.0 ||
      minimum_stable_observations_ == 0U || risk_filter_alpha_ <= 0.0 ||
      risk_filter_alpha_ > 1.0 || deskew_tf_sampling_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid live pipeline timing, radius or tracking parameters");
    }

    detector_ = std::make_unique<ddlzd::Detector>(detector_config);
    risk_classifier_ = std::make_unique<ddlzd::RiskClassifier>(risk_config);
    semantic_projector_ = std::make_unique<SemanticProjector>(semantic_config);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    zones_publisher_ = create_publisher<ddlzd_msgs::msg::LandingZoneArray>(
      "~/zones", rclcpp::QoS(1).reliable());
    markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/markers", rclcpp::QoS(1).reliable());
    debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "~/debug_image", rclcpp::SensorDataQoS().keep_last(1));
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/local_cloud", rclcpp::SensorDataQoS().keep_last(1));
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", error.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

LiveLandingZoneNode::CallbackReturn LiveLandingZoneNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::LifecycleNode::on_activate(state);
  zones_publisher_->on_activate();
  markers_publisher_->on_activate();
  debug_image_publisher_->on_activate();
  map_publisher_->on_activate();
  diagnostics_publisher_->on_activate();

  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, sensor_qos,
    std::bind(&LiveLandingZoneNode::pointCloudCallback, this, std::placeholders::_1));
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, sensor_qos,
    std::bind(&LiveLandingZoneNode::imageCallback, this, std::placeholders::_1));
  camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LiveLandingZoneNode::cameraInfoCallback, this, std::placeholders::_1));
  detection_timer_ = create_wall_timer(
    std::chrono::duration<double>(detection_period_sec_),
    std::bind(&LiveLandingZoneNode::scheduleDetection, this));
  worker_running_.store(true);
  worker_thread_ = std::thread(&LiveLandingZoneNode::workerLoop, this);
  return CallbackReturn::SUCCESS;
}

LiveLandingZoneNode::CallbackReturn LiveLandingZoneNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  detection_timer_.reset();
  cloud_subscription_.reset();
  image_subscription_.reset();
  camera_info_subscription_.reset();
  stopWorker();
  {
    std::scoped_lock lock(cloud_mutex_, image_mutex_, tracks_mutex_);
    cloud_frames_.clear();
    images_.clear();
    camera_info_.reset();
    tracks_.clear();
  }
  visualization_msgs::msg::MarkerArray clear;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear.markers.push_back(marker);
  markers_publisher_->publish(clear);
  zones_publisher_->on_deactivate();
  markers_publisher_->on_deactivate();
  debug_image_publisher_->on_deactivate();
  map_publisher_->on_deactivate();
  diagnostics_publisher_->on_deactivate();
  rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

LiveLandingZoneNode::CallbackReturn LiveLandingZoneNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  detector_.reset();
  risk_classifier_.reset();
  semantic_projector_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  zones_publisher_.reset();
  markers_publisher_.reset();
  debug_image_publisher_.reset();
  map_publisher_.reset();
  diagnostics_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

LiveLandingZoneNode::CallbackReturn LiveLandingZoneNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stopWorker();
  return CallbackReturn::SUCCESS;
}

void LiveLandingZoneNode::stopWorker()
{
  worker_running_.store(false);
  job_condition_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  std::lock_guard<std::mutex> lock(job_mutex_);
  pending_job_.reset();
}

void LiveLandingZoneNode::pointCloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  if (!hasField(*message, "x") || !hasField(*message, "y") || !hasField(*message, "z") ||
    !hasField(*message, "intensity"))
  {
    publishDiagnostic(
      rclcpp::Time(message->header.stamp), diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "PointCloud2 must contain x, y, z and intensity fields", 0U, 0U, 0.0);
    return;
  }
  try {
    CloudFrame frame = transformAndDeskew(*message);
    const rclcpp::Time stamp = frame.stamp;

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!cloud_frames_.empty() && stamp <= cloud_frames_.back().stamp) {
      publishDiagnostic(
        stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Out-of-order or duplicate LiDAR frame rejected", frame.cloud->size(), 0U, 0.0);
      return;
    }
    cloud_frames_.push_back(std::move(frame));
    while (!cloud_frames_.empty() &&
      (stamp - cloud_frames_.front().stamp).seconds() > rolling_window_sec_)
    {
      cloud_frames_.pop_front();
    }
  } catch (const std::exception & error) {
    publishDiagnostic(
      rclcpp::Time(message->header.stamp), diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      std::string("LiDAR transform/deskew failed: ") + error.what(), 0U, 0U, 0.0);
  }
}

LiveLandingZoneNode::CloudFrame LiveLandingZoneNode::transformAndDeskew(
  const sensor_msgs::msg::PointCloud2 & message) const
{
  CloudFrame frame;
  if (message.header.frame_id.empty() || message.width == 0U || message.height == 0U ||
    message.point_step == 0U || static_cast<std::size_t>(message.row_step) <
    static_cast<std::size_t>(message.width) * message.point_step ||
    message.data.size() < static_cast<std::size_t>(message.row_step) * message.height)
  {
    throw std::invalid_argument("malformed or frameless PointCloud2 input");
  }
  frame.stamp = rclcpp::Time(message.header.stamp);
  frame.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  const auto & field_x = requireField(message, "x", sensor_msgs::msg::PointField::FLOAT32);
  const auto & field_y = requireField(message, "y", sensor_msgs::msg::PointField::FLOAT32);
  const auto & field_z = requireField(message, "z", sensor_msgs::msg::PointField::FLOAT32);
  const auto & field_intensity = requireField(
    message, "intensity", sensor_msgs::msg::PointField::FLOAT32);
  for (const auto * field : {&field_x, &field_y, &field_z, &field_intensity}) {
    if (field->offset + sizeof(float) > message.point_step) {
      throw std::invalid_argument("PointCloud2 field offset exceeds point_step");
    }
  }
  if (input_is_motion_compensated_) {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame_, message.header.frame_id, frame.stamp,
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
    sensor_msgs::msg::PointCloud2 transformed;
    tf2::doTransform(message, transformed, transform);
    pcl::fromROSMsg(transformed, *frame.cloud);
    frame.sensor_origin = Eigen::Vector3d(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
    return frame;
  }

  const auto & field_time = requireField(message, "t", sensor_msgs::msg::PointField::UINT32);
  if (field_time.offset + sizeof(std::uint32_t) > message.point_step) {
    throw std::invalid_argument("PointCloud2 t field offset exceeds point_step");
  }
  const bool swap = message.is_bigendian != hostIsBigEndian();
  const std::size_t point_count = static_cast<std::size_t>(message.width) * message.height;
  std::vector<std::uint32_t> offsets_ns;
  offsets_ns.reserve(point_count);
  std::uint32_t maximum_offset_ns = 0U;
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const auto * row_data = message.data.data() + static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const auto * point_data = row_data + static_cast<std::size_t>(column) * message.point_step;
      const auto offset = readScalar<std::uint32_t>(point_data + field_time.offset, swap);
      offsets_ns.push_back(offset);
      maximum_offset_ns = std::max(maximum_offset_ns, offset);
    }
  }

  struct TransformSample
  {
    double offset_sec;
    Eigen::Isometry3d target_from_sensor;
  };
  const double maximum_offset_sec = static_cast<double>(maximum_offset_ns) * 1e-9;
  if (maximum_offset_sec > max_scan_duration_sec_) {
    throw std::invalid_argument("per-point timestamps exceed max_scan_duration_sec");
  }
  const std::size_t interval_count = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(maximum_offset_sec / deskew_tf_sampling_sec_)));
  std::vector<TransformSample> samples;
  samples.reserve(interval_count + 1U);
  for (std::size_t index = 0; index <= interval_count; ++index) {
    const double offset = maximum_offset_sec * static_cast<double>(index) /
      static_cast<double>(interval_count);
    const auto transform = tf_buffer_->lookupTransform(
      target_frame_, message.header.frame_id,
      frame.stamp + rclcpp::Duration::from_seconds(offset),
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
    samples.push_back(TransformSample{offset, tf2::transformToEigen(transform.transform)});
  }
  frame.sensor_origin = samples.back().target_from_sensor.translation();
  frame.cloud->reserve(point_count);

  std::size_t linear_index = 0U;
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const auto * row_data = message.data.data() + static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column, ++linear_index) {
      const auto * point_data = row_data + static_cast<std::size_t>(column) * message.point_step;
      const float x = readScalar<float>(point_data + field_x.offset, swap);
      const float y = readScalar<float>(point_data + field_y.offset, swap);
      const float z = readScalar<float>(point_data + field_z.offset, swap);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double offset = static_cast<double>(offsets_ns[linear_index]) * 1e-9;
      const double sample_position = maximum_offset_sec > 0.0 ?
        offset / maximum_offset_sec * static_cast<double>(interval_count) : 0.0;
      const std::size_t lower_index = std::min(
        interval_count - 1U, static_cast<std::size_t>(std::floor(sample_position)));
      const std::size_t upper_index = std::min(interval_count, lower_index + 1U);
      const auto & lower = samples[lower_index];
      const auto & upper = samples[upper_index];
      const double denominator = upper.offset_sec - lower.offset_sec;
      const double ratio = denominator > 0.0 ?
        std::clamp((offset - lower.offset_sec) / denominator, 0.0, 1.0) : 0.0;
      const Eigen::Quaterniond lower_rotation(lower.target_from_sensor.rotation());
      const Eigen::Quaterniond upper_rotation(upper.target_from_sensor.rotation());
      const Eigen::Quaterniond rotation = lower_rotation.slerp(ratio, upper_rotation);
      const Eigen::Vector3d translation =
        (1.0 - ratio) * lower.target_from_sensor.translation() +
        ratio * upper.target_from_sensor.translation();
      const Eigen::Vector3d transformed = rotation * Eigen::Vector3d(x, y, z) + translation;
      pcl::PointXYZI point;
      point.x = static_cast<float>(transformed.x());
      point.y = static_cast<float>(transformed.y());
      point.z = static_cast<float>(transformed.z());
      point.intensity = readScalar<float>(point_data + field_intensity.offset, swap);
      frame.cloud->push_back(point);
    }
  }
  frame.stamp = frame.stamp + rclcpp::Duration::from_seconds(0.5 * maximum_offset_sec);
  return frame;
}

void LiveLandingZoneNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  const rclcpp::Time stamp(message->header.stamp);
  if (!images_.empty() && stamp <= rclcpp::Time(images_.back()->header.stamp)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Out-of-order or duplicate camera frame rejected");
    return;
  }
  images_.push_back(std::move(message));
  while (!images_.empty() &&
    (stamp - rclcpp::Time(images_.front()->header.stamp)).seconds() > image_buffer_sec_)
  {
    images_.pop_front();
  }
}

void LiveLandingZoneNode::cameraInfoCallback(
  sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
{
  if (message->header.frame_id.empty() || message->width == 0U || message->height == 0U ||
    message->p[0] <= 0.0 || message->p[5] <= 0.0 || message->p[10] <= 0.0)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid CameraInfo rejected");
    return;
  }
  std::lock_guard<std::mutex> lock(image_mutex_);
  camera_info_ = std::move(message);
}

void LiveLandingZoneNode::scheduleDetection()
{
  Job job;
  Eigen::Vector3d newest_origin;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_frames_.empty()) {
      return;
    }
    job.stamp = cloud_frames_.back().stamp;
    newest_origin = cloud_frames_.back().sensor_origin;
    job.sensor_origin = newest_origin;
    if (std::abs((now() - job.stamp).seconds()) > max_cloud_age_sec_) {
      publishDiagnostic(
        job.stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "LiDAR rolling map is stale", 0U, 0U, 0.0);
      return;
    }
    job.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (const auto & frame : cloud_frames_) {
      for (const auto & point : frame.cloud->points) {
        if (std::hypot(
            static_cast<double>(point.x) - newest_origin.x(),
            static_cast<double>(point.y) - newest_origin.y()) <= local_map_radius_m_)
        {
          job.cloud->push_back(point);
        }
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    job.camera_info = camera_info_;
    double best_difference = std::numeric_limits<double>::infinity();
    for (const auto & image : images_) {
      const double difference = std::abs((rclcpp::Time(image->header.stamp) - job.stamp).seconds());
      if (difference < best_difference) {
        best_difference = difference;
        job.image = image;
      }
    }
    if (best_difference > sync_tolerance_sec_) {
      job.image.reset();
    }
  }
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    pending_job_ = std::move(job);  // Latest snapshot wins; stale work is never queued.
  }
  job_condition_.notify_one();
}

void LiveLandingZoneNode::workerLoop()
{
  while (worker_running_.load()) {
    std::optional<Job> job;
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_condition_.wait(lock, [&]() {return !worker_running_.load() || pending_job_.has_value();});
      if (!worker_running_.load()) {
        break;
      }
      job = std::move(pending_job_);
      pending_job_.reset();
    }
    if (job) {
      processJob(*job);
    }
  }
}

void LiveLandingZoneNode::processJob(const Job & job)
{
  const auto start = std::chrono::steady_clock::now();
  auto candidates = detector_->detect(job.cloud);
  const double clearance_boundary_radius =
    local_map_radius_m_ - detector_->config().obstacle_search_radius_m;
  for (auto & candidate : candidates) {
    if ((candidate.center.head<2>() - job.sensor_origin.head<2>()).norm() >
      clearance_boundary_radius)
    {
      candidate.geometry.clearance_observed_fraction = 0.0;
      if (candidate.rejection_reason.empty()) {
        candidate.rejection_reason = "clearance_window_boundary";
      }
    }
  }
  bool camera_fused = false;
  cv::Mat debug_image;
  std::optional<Eigen::Isometry3d> camera_from_target;

  if (job.image && job.camera_info) {
    try {
      if (job.image->header.frame_id != job.camera_info->header.frame_id) {
        throw std::runtime_error("image and CameraInfo frame_id differ");
      }
      const auto transform = tf_buffer_->lookupTransform(
        job.camera_info->header.frame_id, target_frame_, rclcpp::Time(job.image->header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      camera_from_target = tf2::transformToEigen(transform.transform);
      const cv::Mat bgr = cv_bridge::toCvCopy(job.image, "bgr8")->image;
      auto projection = semantic_projector_->project(
        *job.cloud, bgr, *job.camera_info, *camera_from_target);
      debug_image = projection.semantic_bgr;
      for (auto & candidate : candidates) {
        candidate.camera = semantic_projector_->evidenceForCandidate(
          candidate, *job.cloud, projection);
      }
      camera_fused = std::any_of(
        candidates.begin(), candidates.end(), [&](const ddlzd::Candidate & candidate) {
          return candidate.camera.valid &&
                 candidate.camera.coverage_fraction >= minimum_camera_coverage_;
        });
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "RGB fusion unavailable for this update: %s", error.what());
    }
  }

  for (auto & candidate : candidates) {
    risk_classifier_->classify(candidate);
  }
  updateTracks(candidates, job.stamp);
  const double processing_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  publishResults(
    job, candidates, camera_fused, processing_ms, debug_image, camera_from_target);
  publishDiagnostic(
    job.stamp,
    camera_fused ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    camera_fused ? "LiDAR and RGB fusion valid" : "LiDAR valid; RGB evidence unavailable",
    job.cloud->size(), candidates.size(), processing_ms);
}

void LiveLandingZoneNode::updateTracks(
  std::vector<ddlzd::Candidate> & candidates, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(tracks_mutex_);
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(), [&](const Track & track) {
        return (stamp - track.last_seen).seconds() > track_timeout_sec_;
      }),
    tracks_.end());
  std::unordered_set<std::uint64_t> matched;
  for (auto & candidate : candidates) {
    Track * best = nullptr;
    double best_distance = track_match_distance_m_;
    for (auto & track : tracks_) {
      if (matched.count(track.id) != 0U) {
        continue;
      }
      const double distance = (track.center.head<2>() - candidate.center.head<2>()).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best = &track;
      }
    }
    const bool score_valid = !candidate.geometry_valid || std::isfinite(candidate.risk_score);
    if (best == nullptr) {
      Track track;
      track.id = next_track_id_++;
      track.center = candidate.center;
      if (score_valid) {
        track.filtered_risk = candidate.risk_score;
        track.filtered_risk_valid = true;
      }
      track.last_seen = stamp;
      tracks_.push_back(track);
      best = &tracks_.back();
    } else {
      if ((stamp - best->last_seen).seconds() > 1.5 * detection_period_sec_) {
        best->consecutive = 0U;
      }
      best->center = candidate.center;
      if (score_valid) {
        if (!candidate.geometry_valid) {
          best->filtered_risk = 1.0;
          best->filtered_risk_valid = true;
        } else if (best->filtered_risk_valid) {
          best->filtered_risk = risk_filter_alpha_ * candidate.risk_score +
            (1.0 - risk_filter_alpha_) * best->filtered_risk;
        } else {
          best->filtered_risk = candidate.risk_score;
          best->filtered_risk_valid = true;
        }
      }
      best->last_seen = stamp;
    }
    matched.insert(best->id);
    if (candidate.geometry_valid && candidate.category != ddlzd::Category::kUnknown) {
      ++best->consecutive;
    } else {
      best->consecutive = 0U;
    }
    candidate.id = best->id;
    if (score_valid && best->filtered_risk_valid) {
      candidate.risk_score = best->filtered_risk;
    }
    candidate.consecutive_observations = best->consecutive;
    candidate.temporally_stable = best->consecutive >= minimum_stable_observations_;
    if (!candidate.geometry_valid) {
      candidate.category = ddlzd::Category::kRisky;
    } else if (!score_valid || !candidate.temporally_stable) {
      candidate.category = ddlzd::Category::kUnknown;
    } else if (candidate.risk_score <= safest_threshold_) {
      candidate.category = ddlzd::Category::kSafest;
    } else if (candidate.risk_score <= safe_threshold_) {
      candidate.category = ddlzd::Category::kSafe;
    } else {
      candidate.category = ddlzd::Category::kRisky;
    }
  }
}

void LiveLandingZoneNode::publishResults(
  const Job & job, const std::vector<ddlzd::Candidate> & candidates,
  const bool camera_fused, const double processing_ms, const cv::Mat & debug_image,
  const std::optional<Eigen::Isometry3d> & camera_from_target)
{
  ddlzd_msgs::msg::LandingZoneArray output;
  output.header.stamp = job.stamp;
  output.header.frame_id = target_frame_;
  if (job.image) {
    output.camera_stamp = job.image->header.stamp;
  }
  output.camera_fused = camera_fused;
  output.processing_time_ms = static_cast<float>(processing_ms);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  cv::Mat annotated = debug_image.empty() ? cv::Mat() : debug_image.clone();
  int marker_id = 0;
  for (const auto & candidate : candidates) {
    ddlzd_msgs::msg::LandingZone zone;
    zone.id = candidate.id;
    zone.pose.position.x = candidate.center.x();
    zone.pose.position.y = candidate.center.y();
    zone.pose.position.z = candidate.center.z();
    const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), candidate.normal);
    zone.pose.orientation.x = orientation.x();
    zone.pose.orientation.y = orientation.y();
    zone.pose.orientation.z = orientation.z();
    zone.pose.orientation.w = orientation.w();
    zone.normal.x = candidate.normal.x();
    zone.normal.y = candidate.normal.y();
    zone.normal.z = candidate.normal.z();
    zone.radius = static_cast<float>(candidate.radius);
    zone.risk_score = static_cast<float>(candidate.risk_score);
    zone.category = static_cast<std::uint8_t>(candidate.category);
    zone.geometry_valid = candidate.geometry_valid;
    zone.camera_valid = candidate.camera.valid &&
      candidate.camera.coverage_fraction >= minimum_camera_coverage_;
    zone.temporally_stable = candidate.temporally_stable;
    zone.consecutive_observations = candidate.consecutive_observations;
    zone.slope_deg = static_cast<float>(candidate.geometry.slope_deg);
    zone.point_density_m2 = static_cast<float>(candidate.geometry.density_m2);
    zone.relief_m = static_cast<float>(candidate.geometry.relief_m);
    zone.roughness_m = static_cast<float>(candidate.geometry.roughness_m);
    zone.obstacle_count = candidate.geometry.obstacle_count;
    zone.nearest_obstacle_clearance_m = static_cast<float>(
      candidate.geometry.nearest_obstacle_clearance_m);
    zone.observed_fraction = static_cast<float>(candidate.geometry.observed_fraction);
    zone.clearance_observed_fraction = static_cast<float>(
      candidate.geometry.clearance_observed_fraction);
    zone.camera_coverage_fraction = static_cast<float>(candidate.camera.coverage_fraction);
    zone.tree_fraction = static_cast<float>(candidate.camera.tree_fraction);
    zone.tree_score = static_cast<float>(candidate.camera.tree_score);
    zone.grass_fraction = static_cast<float>(candidate.camera.grass_fraction);
    zone.texture_risk = static_cast<float>(candidate.camera.texture_risk);
    zone.rejection_reason = candidate.rejection_reason;
    output.zones.push_back(zone);

    visualization_msgs::msg::Marker circle;
    circle.header = output.header;
    circle.ns = "landing_zone_footprints";
    circle.id = marker_id++;
    circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
    circle.action = visualization_msgs::msg::Marker::ADD;
    circle.scale.x = 0.10;
    circle.color = categoryColor(candidate.category);
    circle.lifetime = rclcpp::Duration::from_seconds(2.5 * detection_period_sec_);
    for (int segment = 0; segment <= 72; ++segment) {
      const double angle = 2.0 * kPi * static_cast<double>(segment) / 72.0;
      const Eigen::Vector3d point = pointOnHorizontalFootprint(candidate, angle, 0.10);
      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.x();
      marker_point.y = point.y();
      marker_point.z = point.z();
      circle.points.push_back(marker_point);
    }
    markers.markers.push_back(circle);

    visualization_msgs::msg::Marker text;
    text.header = output.header;
    text.ns = "landing_zone_labels";
    text.id = marker_id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = candidate.center.x();
    text.pose.position.y = candidate.center.y();
    text.pose.position.z = candidate.center.z() + 0.8;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.35;
    text.color = categoryColor(candidate.category);
    text.text = std::to_string(candidate.id) + " " + categoryName(candidate.category) +
      " R=" + std::to_string(candidate.risk_score).substr(0, 4);
    text.lifetime = circle.lifetime;
    markers.markers.push_back(text);

    if (!annotated.empty() && camera_from_target && job.camera_info) {
      std::vector<std::optional<cv::Point>> image_circle(72U);
      for (int segment = 0; segment < 72; ++segment) {
        const double angle = 2.0 * kPi * static_cast<double>(segment) / 72.0;
        const Eigen::Vector3d point = pointOnHorizontalFootprint(candidate, angle);
        cv::Point pixel;
        if (semantic_projector_->projectPoint(
            point, *job.camera_info, *camera_from_target, pixel))
        {
          image_circle[static_cast<std::size_t>(segment)] = pixel;
        }
      }
      const auto color = candidate.category == ddlzd::Category::kSafest ? cv::Scalar(0, 255, 0) :
        (candidate.category == ddlzd::Category::kSafe ? cv::Scalar(0, 165, 255) :
        (candidate.category == ddlzd::Category::kRisky ? cv::Scalar(0, 0, 255) :
        cv::Scalar(160, 160, 160)));
      for (std::size_t segment = 0U; segment < image_circle.size(); ++segment) {
        const std::size_t next = (segment + 1U) % image_circle.size();
        if (image_circle[segment] && image_circle[next]) {
          cv::line(
            annotated, *image_circle[segment], *image_circle[next], color, 3, cv::LINE_AA);
        }
      }
    }
  }

  zones_publisher_->publish(output);
  markers_publisher_->publish(markers);
  sensor_msgs::msg::PointCloud2 cloud_message;
  pcl::toROSMsg(*job.cloud, cloud_message);
  cloud_message.header = output.header;
  map_publisher_->publish(cloud_message);
  if (!annotated.empty() && job.image) {
    auto debug_message = cv_bridge::CvImage(job.image->header, "bgr8", annotated).toImageMsg();
    debug_image_publisher_->publish(*debug_message);
  }
}

void LiveLandingZoneNode::publishDiagnostic(
  const rclcpp::Time & stamp, const int level, const std::string & message,
  const std::size_t points, const std::size_t candidates, const double processing_ms)
{
  if (!diagnostics_publisher_ || !diagnostics_publisher_->is_activated()) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = static_cast<std::uint8_t>(level);
  status.name = get_fully_qualified_name() + std::string(": fusion");
  status.hardware_id = "ouster_gremsy_fusion";
  status.message = message;
  status.values.push_back(keyValue("map_points", std::to_string(points)));
  status.values.push_back(keyValue("candidates", std::to_string(candidates)));
  status.values.push_back(keyValue("processing_ms", std::to_string(processing_ms)));
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

}  // namespace ddlzd_ros
