#pragma once

#include "ddlzd_core/detector.hpp"
#include "ddlzd_core/risk.hpp"

#include <ddlzd_msgs/msg/landing_zone_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Vector3d sensor_origin{Eigen::Vector3d::Zero()};
  };

  struct Job
  {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Vector3d sensor_origin{Eigen::Vector3d::Zero()};
  };

  struct Track
  {
    std::uint64_t id{0};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double filtered_risk{1.0};
    bool filtered_risk_valid{false};
    std::uint32_t consecutive{0};
    rclcpp::Time last_seen;
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  [[nodiscard]] CloudFrame transformAndDeskew(
    const sensor_msgs::msg::PointCloud2 & message) const;
  void scheduleDetection();
  void workerLoop();
  void processJob(const Job & job);
  void updateTracks(std::vector<ddlzd::Candidate> & candidates, const rclcpp::Time & stamp);
  void publishResults(
    const Job & job, const std::vector<ddlzd::Candidate> & candidates,
    double processing_ms);
  void publishDiagnostic(
    const rclcpp::Time & stamp, int level, const std::string & message,
    std::size_t points, std::size_t candidates, double processing_ms);
  void stopWorker();

  ddlzd::DetectorConfig loadDetectorConfig();
  ddlzd::RiskConfig loadRiskConfig();

  std::string point_cloud_topic_;
  std::string target_frame_;
  std::string algorithm_;
  double rolling_window_sec_{3.0};
  double detection_period_sec_{0.5};
  double local_map_radius_m_{30.0};
  double tf_timeout_sec_{0.10};
  double max_cloud_age_sec_{0.5};
  double max_scan_duration_sec_{0.5};
  double track_match_distance_m_{1.0};
  double track_timeout_sec_{2.0};
  double risk_filter_alpha_{0.35};
  std::uint32_t minimum_stable_observations_{3U};
  double safest_threshold_{0.33};
  double safe_threshold_{0.62};
  bool input_is_motion_compensated_{false};
  double deskew_tf_sampling_sec_{0.002};

  std::unique_ptr<ddlzd::Detector> detector_;
  std::unique_ptr<ddlzd::RiskClassifier> risk_classifier_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::TimerBase::SharedPtr detection_timer_;

  rclcpp_lifecycle::LifecyclePublisher<ddlzd_msgs::msg::LandingZoneArray>::SharedPtr zones_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  std::mutex cloud_mutex_;
  std::deque<CloudFrame> cloud_frames_;

  std::atomic_bool worker_running_{false};
  std::thread worker_thread_;
  std::mutex job_mutex_;
  std::condition_variable job_condition_;
  std::optional<Job> pending_job_;

  std::mutex tracks_mutex_;
  std::vector<Track> tracks_;
  std::uint64_t next_track_id_{1U};
};

}  // namespace ddlzd_ros
