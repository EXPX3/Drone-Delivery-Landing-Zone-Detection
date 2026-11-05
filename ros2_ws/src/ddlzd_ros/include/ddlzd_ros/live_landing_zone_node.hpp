#pragma once

#include "ddlzd_core/detector.hpp"
#include "ddlzd_core/risk.hpp"
#include "ddlzd_ros/sticky_selector.hpp"

#include <ddlzd_msgs/msg/landing_zone_array.hpp>
#include <ddlzd_msgs/msg/selected_landing_zone.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ddlzd_ros
{

class LiveLandingZoneNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LiveLandingZoneNode(const rclcpp::NodeOptions & options);
  ~LiveLandingZoneNode() override;

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  struct CloudFrame
  {
    rclcpp::Time stamp;
    std::uint64_t sequence{0U};
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Vector3d sensor_origin{Eigen::Vector3d::Zero()};
  };

  struct Job
  {
    rclcpp::Time stamp;
    std::uint64_t input_sequence{0U};
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Vector3d sensor_origin{Eigen::Vector3d::Zero()};
  };

  struct Track
  {
    std::uint64_t id{0U};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double filtered_risk{1.0};
    bool filtered_risk_valid{false};
    std::uint32_t consecutive{0U};
    rclcpp::Time last_seen;
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void localPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr message);
  void gnssCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr message);
  [[nodiscard]] CloudFrame transformAndDeskew(
    const sensor_msgs::msg::PointCloud2 & message) const;
  void scheduleDetection();
  void workerLoop();
  void processJob(const Job & job);
  void updateTracks(std::vector<ddlzd::Candidate> & candidates, const rclcpp::Time & stamp);
  void rankCandidates(std::vector<ddlzd::Candidate> & candidates) const;
  [[nodiscard]] ddlzd_msgs::msg::SelectedLandingZone makeSelectedMessage(
    const Job & job, const SelectionResult & selection) const;
  void publishResults(
    const Job & job, const std::vector<ddlzd::Candidate> & candidates,
    const SelectionResult & selection, double processing_ms);
  void publishDiagnostic(
    const rclcpp::Time & stamp, int level, const std::string & message,
    std::size_t points, std::size_t candidates, double processing_ms);
  void stopWorker();

  ddlzd::DetectorConfig loadDetectorConfig();
  ddlzd::RiskConfig loadRiskConfig();

  std::string point_cloud_topic_;
  std::string local_pose_topic_;
  std::string gnss_topic_;
  std::string target_frame_;
  std::string algorithm_;
  std::string operating_mode_;
  std::string sensor_model_;
  std::string point_time_field_;
  std::uint8_t point_time_datatype_{0U};
  double point_time_scale_sec_{0.0};
  double rolling_window_sec_{3.0};
  double detection_period_sec_{0.5};
  double local_map_radius_m_{30.0};
  double tf_timeout_sec_{0.10};
  double max_cloud_age_sec_{0.5};
  double max_scan_duration_sec_{0.5};
  double track_match_distance_m_{1.0};
  double track_timeout_sec_{2.0};
  double risk_filter_alpha_{0.35};
  double position_filter_alpha_{0.35};
  std::uint32_t minimum_stable_observations_{3U};
  double safest_threshold_{0.33};
  double safe_threshold_{0.62};
  bool input_is_motion_compensated_{false};
  double deskew_tf_sampling_sec_{0.002};
  double gnss_sync_tolerance_sec_{0.50};
  double telemetry_retention_sec_{5.0};
  bool require_known_gnss_covariance_{false};
  double max_horizontal_uncertainty_m_{10.0};
  float ranking_alpha_{0.90F};
  float sticky_background_alpha_{0.22F};
  float selected_alpha_{1.0F};

  std::unique_ptr<ddlzd::Detector> detector_;
  std::unique_ptr<ddlzd::RiskClassifier> risk_classifier_;
  std::unique_ptr<StickySelector> selector_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscription_;
  rclcpp::TimerBase::SharedPtr detection_timer_;

  rclcpp_lifecycle::LifecyclePublisher<ddlzd_msgs::msg::LandingZoneArray>::SharedPtr zones_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<ddlzd_msgs::msg::SelectedLandingZone>::SharedPtr selected_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr
    selected_gnss_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    markers_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;

  std::mutex cloud_mutex_;
  std::deque<CloudFrame> cloud_frames_;
  std::uint64_t next_cloud_sequence_{1U};
  std::uint64_t last_scheduled_sequence_{0U};

  std::atomic_bool worker_running_{false};
  std::thread worker_thread_;
  std::mutex job_mutex_;
  std::condition_variable job_condition_;
  std::optional<Job> pending_job_;

  std::mutex tracks_mutex_;
  std::vector<Track> tracks_;
  std::uint64_t next_track_id_{1U};

  mutable std::mutex telemetry_mutex_;
  std::deque<geometry_msgs::msg::PoseStamped> local_poses_;
  std::deque<sensor_msgs::msg::NavSatFix> gnss_fixes_;
};

}  // namespace ddlzd_ros
