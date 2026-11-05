#include "ddlzd_ros/live_landing_zone_node.hpp"

#include "ddlzd_ros/geodesy.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
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

std::size_t datatypeSize(const std::uint8_t datatype)
{
  if (datatype == sensor_msgs::msg::PointField::FLOAT32 ||
    datatype == sensor_msgs::msg::PointField::UINT32)
  {
    return sizeof(std::uint32_t);
  }
  throw std::invalid_argument("unsupported point timestamp datatype");
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

std_msgs::msg::ColorRGBA categoryColor(const ddlzd::Category category, const float alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.a = alpha;
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

std_msgs::msg::ColorRGBA selectedColor(const float alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 0.20F;
  color.g = 1.0F;
  color.b = 0.20F;
  color.a = alpha;
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

std::uint8_t evidenceState(const ddlzd::Candidate & candidate)
{
  using Message = ddlzd_msgs::msg::LandingZone;
  if (!candidate.geometry_valid || !candidate.rejection_reason.empty()) {
    return Message::EVIDENCE_INVALID;
  }
  if (!std::isfinite(candidate.risk_score)) {
    return Message::EVIDENCE_INSUFFICIENT;
  }
  if (!candidate.temporally_stable) {
    return Message::EVIDENCE_PENDING;
  }
  return Message::EVIDENCE_CONFIRMED;
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

std::size_t nonNegativeSize(const int value, const std::string & parameter)
{
  if (value < 0) {
    throw std::invalid_argument(parameter + " must be a non-negative integer");
  }
  return static_cast<std::size_t>(value);
}

std::uint32_t positiveUint32(const int value, const std::string & parameter)
{
  const auto converted = positiveSize(value, parameter);
  if (converted > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument(parameter + " exceeds uint32 range");
  }
  return static_cast<std::uint32_t>(converted);
}

template<typename MessageT>
std::optional<MessageT> nearestSample(
  const std::deque<MessageT> & samples, const rclcpp::Time & stamp, const double tolerance_sec)
{
  const MessageT * best = nullptr;
  double best_difference = tolerance_sec;
  for (const auto & sample : samples) {
    const double difference = std::abs((rclcpp::Time(sample.header.stamp) - stamp).seconds());
    if (difference <= best_difference) {
      best_difference = difference;
      best = &sample;
    }
  }
  return best == nullptr ? std::nullopt : std::optional<MessageT>(*best);
}

double horizontalUncertainty(const sensor_msgs::msg::NavSatFix & fix)
{
  if (fix.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double a = fix.position_covariance[0];
  const double d = fix.position_covariance[4];
  const double b = 0.5 * (fix.position_covariance[1] + fix.position_covariance[3]);
  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(d) || a < 0.0 || d < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double discriminant = std::sqrt(std::max(0.0, 0.25 * (a - d) * (a - d) + b * b));
  return std::sqrt(std::max(0.0, 0.5 * (a + d) + discriminant));
}

void fillCandidatePose(const ddlzd::Candidate & candidate, geometry_msgs::msg::Pose & pose)
{
  pose.position.x = candidate.center.x();
  pose.position.y = candidate.center.y();
  pose.position.z = candidate.center.z();
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
    Eigen::Vector3d::UnitZ(), candidate.normal);
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
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
  algorithm_ = declare_parameter<std::string>(
    "detector.algorithm", "region_growing_exact_radius");
  if (algorithm_ != "region_growing_exact_radius") {
    throw std::invalid_argument(
            "detector.algorithm must be region_growing_exact_radius in this branch");
  }
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
  config.obstacle_min_points = nonNegativeSize(
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
  return config;
}

LiveLandingZoneNode::CallbackReturn LiveLandingZoneNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    operating_mode_ = declare_parameter<std::string>("operating_mode", "ranking");
    if (operating_mode_ != "ranking" && operating_mode_ != "sticky") {
      throw std::invalid_argument("operating_mode must be ranking or sticky");
    }
    const auto local_frame_convention = declare_parameter<std::string>(
      "local_frame_convention", "enu");
    if (local_frame_convention != "enu") {
      throw std::invalid_argument("GNSS conversion requires local_frame_convention=enu");
    }
    sensor_model_ = declare_parameter<std::string>("sensor.model", "");
    if (sensor_model_ != "vlp16" && sensor_model_ != "os1_rev7" && sensor_model_ != "os1_rev8") {
      throw std::invalid_argument("sensor.model must be vlp16, os1_rev7 or os1_rev8");
    }
    point_time_field_ = declare_parameter<std::string>("sensor.point_time_field", "");
    const auto point_time_datatype = declare_parameter<std::string>(
      "sensor.point_time_datatype", "");
    if (point_time_datatype == "float32") {
      point_time_datatype_ = sensor_msgs::msg::PointField::FLOAT32;
    } else if (point_time_datatype == "uint32") {
      point_time_datatype_ = sensor_msgs::msg::PointField::UINT32;
    } else {
      throw std::invalid_argument("sensor.point_time_datatype must be float32 or uint32");
    }
    point_time_scale_sec_ = declare_parameter<double>("sensor.point_time_scale_sec", 0.0);
    point_cloud_topic_ = declare_parameter<std::string>("point_cloud_topic", "");
    local_pose_topic_ = declare_parameter<std::string>(
      "local_pose_topic", "/mavros/local_position/pose");
    gnss_topic_ = declare_parameter<std::string>(
      "gnss_topic", "/mavros/global_position/global");
    target_frame_ = declare_parameter<std::string>("target_frame", "");
    input_is_motion_compensated_ = declare_parameter<bool>("input_is_motion_compensated", false);
    if (point_cloud_topic_.empty() || local_pose_topic_.empty() || gnss_topic_.empty() ||
      target_frame_.empty() || point_time_field_.empty())
    {
      throw std::invalid_argument(
              "point cloud, local pose, GNSS, target frame and point-time bindings are required");
    }

    rolling_window_sec_ = declare_parameter<double>("rolling_window_sec", rolling_window_sec_);
    detection_period_sec_ = declare_parameter<double>("detection_period_sec", detection_period_sec_);
    local_map_radius_m_ = declare_parameter<double>("local_map_radius_m", local_map_radius_m_);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", max_cloud_age_sec_);
    max_scan_duration_sec_ = declare_parameter<double>(
      "max_scan_duration_sec", max_scan_duration_sec_);
    deskew_tf_sampling_sec_ = declare_parameter<double>(
      "deskew_tf_sampling_sec", deskew_tf_sampling_sec_);
    track_match_distance_m_ = declare_parameter<double>("tracking.match_distance_m", track_match_distance_m_);
    track_timeout_sec_ = declare_parameter<double>("tracking.timeout_sec", track_timeout_sec_);
    risk_filter_alpha_ = declare_parameter<double>("tracking.risk_filter_alpha", risk_filter_alpha_);
    position_filter_alpha_ = declare_parameter<double>(
      "tracking.position_filter_alpha", position_filter_alpha_);
    minimum_stable_observations_ = positiveUint32(
      declare_parameter<int>(
        "tracking.minimum_stable_observations", static_cast<int>(minimum_stable_observations_)),
      "tracking.minimum_stable_observations");

    StickySelectorConfig selector_config;
    selector_config.dropout_timeout_sec = declare_parameter<double>(
      "selection.dropout_timeout_sec", selector_config.dropout_timeout_sec);
    selector_config.max_missed_detections = positiveUint32(
      declare_parameter<int>(
        "selection.max_missed_detections",
        static_cast<int>(selector_config.max_missed_detections)),
      "selection.max_missed_detections");
    selector_config.soft_release_observations = positiveUint32(
      declare_parameter<int>(
        "selection.soft_release_observations",
        static_cast<int>(selector_config.soft_release_observations)),
      "selection.soft_release_observations");

    gnss_sync_tolerance_sec_ = declare_parameter<double>(
      "gnss.sync_tolerance_sec", gnss_sync_tolerance_sec_);
    telemetry_retention_sec_ = declare_parameter<double>(
      "gnss.sample_retention_sec", telemetry_retention_sec_);
    require_known_gnss_covariance_ = declare_parameter<bool>(
      "gnss.require_known_covariance", require_known_gnss_covariance_);
    max_horizontal_uncertainty_m_ = declare_parameter<double>(
      "gnss.max_horizontal_uncertainty_m", max_horizontal_uncertainty_m_);
    ranking_alpha_ = static_cast<float>(declare_parameter<double>(
      "visualization.ranking_alpha", ranking_alpha_));
    sticky_background_alpha_ = static_cast<float>(declare_parameter<double>(
      "visualization.sticky_background_alpha", sticky_background_alpha_));
    selected_alpha_ = static_cast<float>(declare_parameter<double>(
      "visualization.selected_alpha", selected_alpha_));

    const auto detector_config = loadDetectorConfig();
    const auto risk_config = loadRiskConfig();
    if (rolling_window_sec_ <= 0.0 || detection_period_sec_ <= 0.0 ||
      local_map_radius_m_ <= detector_config.landing_radius_m + detector_config.safety_margin_m ||
      local_map_radius_m_ <= detector_config.obstacle_search_radius_m ||
      detector_config.obstacle_search_radius_m < risk_config.clearance_good_m ||
      tf_timeout_sec_ <= 0.0 || max_cloud_age_sec_ <= 0.0 ||
      max_scan_duration_sec_ <= 0.0 || point_time_scale_sec_ <= 0.0 ||
      track_match_distance_m_ <= 0.0 || track_timeout_sec_ <= 0.0 ||
      risk_filter_alpha_ <= 0.0 || risk_filter_alpha_ > 1.0 ||
      position_filter_alpha_ <= 0.0 || position_filter_alpha_ > 1.0 ||
      deskew_tf_sampling_sec_ <= 0.0 || gnss_sync_tolerance_sec_ <= 0.0 ||
      telemetry_retention_sec_ <= gnss_sync_tolerance_sec_ || max_horizontal_uncertainty_m_ <= 0.0 ||
      ranking_alpha_ <= 0.0F || ranking_alpha_ > 1.0F ||
      sticky_background_alpha_ <= 0.0F || sticky_background_alpha_ > 1.0F ||
      selected_alpha_ <= 0.0F || selected_alpha_ > 1.0F)
    {
      throw std::invalid_argument("invalid pipeline, tracking, GNSS or visualization parameter");
    }

    detector_ = std::make_unique<ddlzd::Detector>(detector_config);
    risk_classifier_ = std::make_unique<ddlzd::RiskClassifier>(risk_config);
    selector_ = std::make_unique<StickySelector>(selector_config);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    zones_publisher_ = create_publisher<ddlzd_msgs::msg::LandingZoneArray>(
      "~/zones", rclcpp::QoS(1).reliable());
    selected_publisher_ = create_publisher<ddlzd_msgs::msg::SelectedLandingZone>(
      "~/selected", rclcpp::QoS(1).reliable());
    selected_gnss_publisher_ = create_publisher<geographic_msgs::msg::GeoPointStamped>(
      "~/selected_gnss", rclcpp::QoS(1).reliable());
    markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/markers", rclcpp::QoS(1).reliable());
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
  selected_publisher_->on_activate();
  selected_gnss_publisher_->on_activate();
  markers_publisher_->on_activate();
  map_publisher_->on_activate();
  diagnostics_publisher_->on_activate();

  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(20);
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, sensor_qos,
    std::bind(&LiveLandingZoneNode::pointCloudCallback, this, std::placeholders::_1));
  local_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    local_pose_topic_, sensor_qos,
    std::bind(&LiveLandingZoneNode::localPoseCallback, this, std::placeholders::_1));
  gnss_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    gnss_topic_, sensor_qos,
    std::bind(&LiveLandingZoneNode::gnssCallback, this, std::placeholders::_1));
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
  local_pose_subscription_.reset();
  gnss_subscription_.reset();
  stopWorker();
  {
    std::scoped_lock lock(cloud_mutex_, tracks_mutex_, telemetry_mutex_);
    cloud_frames_.clear();
    tracks_.clear();
    local_poses_.clear();
    gnss_fixes_.clear();
    last_scheduled_sequence_ = 0U;
  }
  selector_->reset();
  visualization_msgs::msg::MarkerArray clear;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear.markers.push_back(marker);
  markers_publisher_->publish(clear);
  zones_publisher_->on_deactivate();
  selected_publisher_->on_deactivate();
  selected_gnss_publisher_->on_deactivate();
  markers_publisher_->on_deactivate();
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
  selector_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  zones_publisher_.reset();
  selected_publisher_.reset();
  selected_gnss_publisher_.reset();
  markers_publisher_.reset();
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

void LiveLandingZoneNode::localPoseCallback(
  geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
{
  if (message->header.frame_id.empty() ||
    !std::isfinite(message->pose.position.x) ||
    !std::isfinite(message->pose.position.y) ||
    !std::isfinite(message->pose.position.z) ||
    !std::isfinite(message->pose.orientation.x) ||
    !std::isfinite(message->pose.orientation.y) ||
    !std::isfinite(message->pose.orientation.z) ||
    !std::isfinite(message->pose.orientation.w) ||
    std::hypot(
      std::hypot(message->pose.orientation.x, message->pose.orientation.y),
      std::hypot(message->pose.orientation.z, message->pose.orientation.w)) < 1.0e-6)
  {
    return;
  }
  const rclcpp::Time stamp(message->header.stamp);
  std::lock_guard<std::mutex> lock(telemetry_mutex_);
  if (!local_poses_.empty() && stamp < rclcpp::Time(local_poses_.back().header.stamp)) {
    return;
  }
  local_poses_.push_back(*message);
  while (!local_poses_.empty() &&
    (stamp - rclcpp::Time(local_poses_.front().header.stamp)).seconds() > telemetry_retention_sec_)
  {
    local_poses_.pop_front();
  }
}

void LiveLandingZoneNode::gnssCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr message)
{
  const rclcpp::Time stamp(message->header.stamp);
  std::lock_guard<std::mutex> lock(telemetry_mutex_);
  if (!gnss_fixes_.empty() && stamp < rclcpp::Time(gnss_fixes_.back().header.stamp)) {
    return;
  }
  gnss_fixes_.push_back(*message);
  while (!gnss_fixes_.empty() &&
    (stamp - rclcpp::Time(gnss_fixes_.front().header.stamp)).seconds() > telemetry_retention_sec_)
  {
    gnss_fixes_.pop_front();
  }
}

void LiveLandingZoneNode::pointCloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  if (!hasField(*message, "x") || !hasField(*message, "y") || !hasField(*message, "z") ||
    !hasField(*message, "intensity"))
  {
    publishDiagnostic(
      rclcpp::Time(message->header.stamp), diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "PointCloud2 must contain x, y, z and float32 intensity fields", 0U, 0U, 0.0);
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
    frame.sequence = next_cloud_sequence_++;
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

  const auto & field_time = requireField(message, point_time_field_, point_time_datatype_);
  if (field_time.offset + datatypeSize(point_time_datatype_) > message.point_step) {
    throw std::invalid_argument("PointCloud2 timestamp field offset exceeds point_step");
  }
  const bool swap = message.is_bigendian != hostIsBigEndian();
  const std::size_t point_count = static_cast<std::size_t>(message.width) * message.height;
  std::vector<double> offsets_sec;
  offsets_sec.reserve(point_count);
  double maximum_offset_sec = 0.0;
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const auto * row_data = message.data.data() + static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const auto * point_data = row_data + static_cast<std::size_t>(column) * message.point_step;
      double offset = 0.0;
      if (point_time_datatype_ == sensor_msgs::msg::PointField::FLOAT32) {
        offset = static_cast<double>(readScalar<float>(point_data + field_time.offset, swap)) *
          point_time_scale_sec_;
      } else {
        offset = static_cast<double>(
          readScalar<std::uint32_t>(point_data + field_time.offset, swap)) *
          point_time_scale_sec_;
      }
      if (!std::isfinite(offset) || offset < 0.0 || offset > max_scan_duration_sec_) {
        throw std::invalid_argument("per-point timestamp is outside max_scan_duration_sec");
      }
      offsets_sec.push_back(offset);
      maximum_offset_sec = std::max(maximum_offset_sec, offset);
    }
  }

  struct TransformSample
  {
    double offset_sec;
    Eigen::Isometry3d target_from_sensor;
  };
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
      const double offset = offsets_sec[linear_index];
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

void LiveLandingZoneNode::scheduleDetection()
{
  Job job;
  Eigen::Vector3d newest_origin;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_frames_.empty() || cloud_frames_.back().sequence <= last_scheduled_sequence_) {
      return;
    }
    job.stamp = cloud_frames_.back().stamp;
    job.input_sequence = cloud_frames_.back().sequence;
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
    last_scheduled_sequence_ = job.input_sequence;
  }
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    pending_job_ = std::move(job);
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
      try {
        processJob(*job);
      } catch (const std::exception & error) {
        publishDiagnostic(
          job->stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          std::string("Landing-zone processing failed: ") + error.what(),
          job->cloud ? job->cloud->size() : 0U, 0U, 0.0);
      }
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
    risk_classifier_->classify(candidate);
  }
  updateTracks(candidates, job.stamp);
  rankCandidates(candidates);

  SelectionResult selection;
  if (operating_mode_ == "sticky") {
    selection = selector_->update(candidates, job.stamp.seconds());
  } else {
    selector_->reset();
    selection.reason = "ranking_mode_only";
  }

  const double processing_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  publishResults(job, candidates, selection, processing_ms);
  publishDiagnostic(
    job.stamp,
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "LiDAR geometry classification valid",
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

  struct Match
  {
    double distance;
    std::size_t track_index;
    std::size_t candidate_index;
  };
  std::vector<Match> possible_matches;
  for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
      const double distance = (
        tracks_[track_index].center.head<2>() -
        candidates[candidate_index].center.head<2>()).norm();
      if (distance < track_match_distance_m_) {
        possible_matches.push_back(Match{distance, track_index, candidate_index});
      }
    }
  }
  std::sort(
    possible_matches.begin(), possible_matches.end(), [](const Match & lhs, const Match & rhs) {
      return lhs.distance < rhs.distance;
    });
  std::vector<std::optional<std::size_t>> candidate_tracks(candidates.size());
  std::vector<bool> track_matched(tracks_.size(), false);
  for (const auto & match : possible_matches) {
    if (!track_matched[match.track_index] && !candidate_tracks[match.candidate_index]) {
      track_matched[match.track_index] = true;
      candidate_tracks[match.candidate_index] = match.track_index;
    }
  }

  for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
    auto & candidate = candidates[candidate_index];
    if (!candidate_tracks[candidate_index]) {
      Track track;
      track.id = next_track_id_++;
      track.center = candidate.center;
      track.last_seen = stamp;
      tracks_.push_back(track);
      candidate_tracks[candidate_index] = tracks_.size() - 1U;
    }
    auto & track = tracks_[*candidate_tracks[candidate_index]];
    if ((stamp - track.last_seen).seconds() > 1.5 * detection_period_sec_) {
      track.consecutive = 0U;
    }
    track.center = position_filter_alpha_ * candidate.center +
      (1.0 - position_filter_alpha_) * track.center;
    candidate.center = track.center;

    const bool numeric_score = candidate.geometry_valid && std::isfinite(candidate.risk_score);
    if (!candidate.geometry_valid) {
      track.filtered_risk = 1.0;
      track.filtered_risk_valid = true;
    } else if (numeric_score) {
      track.filtered_risk = track.filtered_risk_valid ?
        risk_filter_alpha_ * candidate.risk_score +
        (1.0 - risk_filter_alpha_) * track.filtered_risk : candidate.risk_score;
      track.filtered_risk_valid = true;
    }
    track.last_seen = stamp;

    if (numeric_score && candidate.rejection_reason.empty()) {
      ++track.consecutive;
    } else {
      track.consecutive = 0U;
    }
    candidate.id = track.id;
    if (numeric_score && track.filtered_risk_valid) {
      candidate.risk_score = track.filtered_risk;
    }
    candidate.consecutive_observations = track.consecutive;
    candidate.temporally_stable = track.consecutive >= minimum_stable_observations_;
    if (!candidate.geometry_valid || !candidate.rejection_reason.empty()) {
      candidate.category = ddlzd::Category::kRisky;
    } else if (!numeric_score || !candidate.temporally_stable) {
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

void LiveLandingZoneNode::rankCandidates(std::vector<ddlzd::Candidate> & candidates) const
{
  std::stable_sort(
    candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
      const bool lhs_scored = std::isfinite(lhs.risk_score);
      const bool rhs_scored = std::isfinite(rhs.risk_score);
      if (lhs_scored != rhs_scored) {
        return lhs_scored;
      }
      const double lhs_risk = lhs_scored ? lhs.risk_score : 1.0;
      const double rhs_risk = rhs_scored ? rhs.risk_score : 1.0;
      if (lhs_risk != rhs_risk) {
        return lhs_risk < rhs_risk;
      }
      if (lhs.geometry.nearest_obstacle_clearance_m !=
        rhs.geometry.nearest_obstacle_clearance_m)
      {
        return lhs.geometry.nearest_obstacle_clearance_m >
               rhs.geometry.nearest_obstacle_clearance_m;
      }
      return lhs.id < rhs.id;
    });
}

ddlzd_msgs::msg::SelectedLandingZone LiveLandingZoneNode::makeSelectedMessage(
  const Job & job, const SelectionResult & selection) const
{
  ddlzd_msgs::msg::SelectedLandingZone output;
  output.header.stamp = job.stamp;
  output.header.frame_id = target_frame_;
  output.generation = selection.generation;
  output.selection_state = static_cast<std::uint8_t>(selection.state);
  output.selection_valid = selection.valid();
  output.gnss_valid = false;
  output.reason = selection.reason;
  output.sensor_model = sensor_model_;
  output.latitude = std::numeric_limits<double>::quiet_NaN();
  output.longitude = std::numeric_limits<double>::quiet_NaN();
  output.gnss_horizontal_uncertainty_m = std::numeric_limits<double>::quiet_NaN();
  output.risk_score = std::numeric_limits<float>::quiet_NaN();
  if (!selection.candidate) {
    return output;
  }

  const auto & candidate = *selection.candidate;
  output.id = candidate.id;
  fillCandidatePose(candidate, output.local_pose);
  output.radius = static_cast<float>(candidate.radius);
  output.risk_score = static_cast<float>(candidate.risk_score);
  output.category = static_cast<std::uint8_t>(candidate.category);
  output.consecutive_observations = candidate.consecutive_observations;
  if (!selection.valid()) {
    return output;
  }

  std::optional<geometry_msgs::msg::PoseStamped> local_pose;
  std::optional<sensor_msgs::msg::NavSatFix> gnss_fix;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    local_pose = nearestSample(local_poses_, job.stamp, gnss_sync_tolerance_sec_);
    gnss_fix = nearestSample(gnss_fixes_, job.stamp, gnss_sync_tolerance_sec_);
  }
  if (!local_pose || !gnss_fix) {
    output.reason = "selected_but_missing_synchronized_pose_or_gnss";
    return output;
  }
  if (std::abs(
      (rclcpp::Time(local_pose->header.stamp) -
      rclcpp::Time(gnss_fix->header.stamp)).seconds()) > gnss_sync_tolerance_sec_)
  {
    output.reason = "selected_but_pose_and_gnss_are_not_synchronized";
    return output;
  }
  if (gnss_fix->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
    !std::isfinite(gnss_fix->latitude) || gnss_fix->latitude < -90.0 ||
    gnss_fix->latitude > 90.0 || !std::isfinite(gnss_fix->longitude) ||
    gnss_fix->longitude < -180.0 || gnss_fix->longitude > 180.0)
  {
    output.reason = "selected_but_gnss_fix_is_invalid";
    return output;
  }

  const double uncertainty = horizontalUncertainty(*gnss_fix);
  output.gnss_horizontal_uncertainty_m = uncertainty;
  if ((require_known_gnss_covariance_ && !std::isfinite(uncertainty)) ||
    (std::isfinite(uncertainty) && uncertainty > max_horizontal_uncertainty_m_))
  {
    output.reason = "selected_but_gnss_uncertainty_is_unacceptable";
    return output;
  }

  try {
    geometry_msgs::msg::PoseStamped pose_in_target;
    if (local_pose->header.frame_id == target_frame_) {
      pose_in_target = *local_pose;
    } else {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame_, local_pose->header.frame_id, rclcpp::Time(local_pose->header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      tf2::doTransform(*local_pose, pose_in_target, transform);
    }
    const double east = candidate.center.x() - pose_in_target.pose.position.x;
    const double north = candidate.center.y() - pose_in_target.pose.position.y;
    output.vehicle_relative_enu.x = east;
    output.vehicle_relative_enu.y = north;
    output.vehicle_relative_enu.z = candidate.center.z() - pose_in_target.pose.position.z;
    const auto coordinate = offsetEnuToWgs84(
      gnss_fix->latitude, gnss_fix->longitude, east, north);
    output.latitude = coordinate.latitude;
    output.longitude = coordinate.longitude;
    output.gnss_valid = true;
    output.reason = "confirmed";
  } catch (const std::exception & error) {
    output.reason = std::string("selected_but_coordinate_conversion_failed: ") + error.what();
  }
  return output;
}

void LiveLandingZoneNode::publishResults(
  const Job & job, const std::vector<ddlzd::Candidate> & candidates,
  const SelectionResult & selection, const double processing_ms)
{
  ddlzd_msgs::msg::LandingZoneArray output;
  output.header.stamp = job.stamp;
  output.header.frame_id = target_frame_;
  output.operating_mode = operating_mode_;
  output.sensor_model = sensor_model_;
  output.processing_time_ms = static_cast<float>(processing_ms);

  const auto selected_id = selection.candidate ?
    std::optional<std::uint64_t>(selection.candidate->id) : std::nullopt;
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  int marker_id = 0;

  auto append_markers = [&](
      const ddlzd::Candidate & candidate, const bool is_selected,
      const std::string & label, const std::uint32_t rank) {
      const float alpha = is_selected ? selected_alpha_ :
        (operating_mode_ == "sticky" ? sticky_background_alpha_ : ranking_alpha_);
      const auto color = is_selected ? selectedColor(alpha) :
        categoryColor(candidate.category, alpha);
      visualization_msgs::msg::Marker circle;
      circle.header = output.header;
      circle.ns = "landing_zone_footprints";
      circle.id = marker_id++;
      circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.scale.x = is_selected ? 0.22 : 0.10;
      circle.color = color;
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
      text.scale.z = is_selected ? 0.48 : 0.35;
      text.color = color;
      text.text = is_selected ? label :
        "#" + std::to_string(rank) + " " + categoryName(candidate.category) +
        " R=" + (std::isfinite(candidate.risk_score) ?
        std::to_string(candidate.risk_score).substr(0, 4) : "n/a");
      text.lifetime = circle.lifetime;
      markers.markers.push_back(text);
    };

  bool selected_rendered = false;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto & candidate = candidates[index];
    const bool is_selected = selected_id && candidate.id == *selected_id;
    selected_rendered = selected_rendered || is_selected;
    ddlzd_msgs::msg::LandingZone zone;
    zone.id = candidate.id;
    zone.rank = static_cast<std::uint32_t>(index + 1U);
    fillCandidatePose(candidate, zone.pose);
    zone.normal.x = candidate.normal.x();
    zone.normal.y = candidate.normal.y();
    zone.normal.z = candidate.normal.z();
    zone.radius = static_cast<float>(candidate.radius);
    zone.risk_score = static_cast<float>(candidate.risk_score);
    zone.category = static_cast<std::uint8_t>(candidate.category);
    zone.evidence_state = evidenceState(candidate);
    zone.geometry_valid = candidate.geometry_valid;
    zone.temporally_stable = candidate.temporally_stable;
    zone.selected = is_selected;
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
    zone.rejection_reason = candidate.rejection_reason;
    output.zones.push_back(zone);

    const std::string selected_label = selection.state == SelectionState::kCoasting ?
      "selected (temporarily unverified)" : "selected";
    append_markers(candidate, is_selected, selected_label, zone.rank);
  }
  if (selection.candidate && !selected_rendered) {
    append_markers(
      *selection.candidate, true, "selected (temporarily unverified)", 0U);
  }

  const auto selected_message = makeSelectedMessage(job, selection);
  zones_publisher_->publish(output);
  selected_publisher_->publish(selected_message);
  if (selected_message.selection_valid && selected_message.gnss_valid) {
    geographic_msgs::msg::GeoPointStamped coordinate;
    coordinate.header.stamp = selected_message.header.stamp;
    coordinate.header.frame_id = "wgs84";
    coordinate.position.latitude = selected_message.latitude;
    coordinate.position.longitude = selected_message.longitude;
    coordinate.position.altitude = std::numeric_limits<double>::quiet_NaN();
    selected_gnss_publisher_->publish(coordinate);
  }
  markers_publisher_->publish(markers);
  sensor_msgs::msg::PointCloud2 cloud_message;
  pcl::toROSMsg(*job.cloud, cloud_message);
  cloud_message.header = output.header;
  map_publisher_->publish(cloud_message);
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
  status.name = get_node_base_interface()->get_fully_qualified_name() + std::string(": lidar");
  status.hardware_id = sensor_model_;
  status.message = message;
  status.values.push_back(keyValue("sensor_model", sensor_model_));
  status.values.push_back(keyValue("operating_mode", operating_mode_));
  status.values.push_back(keyValue("map_points", std::to_string(points)));
  status.values.push_back(keyValue("candidates", std::to_string(candidates)));
  status.values.push_back(keyValue("processing_ms", std::to_string(processing_ms)));
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

}  // namespace ddlzd_ros
