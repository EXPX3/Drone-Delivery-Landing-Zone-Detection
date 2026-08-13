#include "ddlzd_ros/live_landing_zone_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  auto node = std::make_shared<ddlzd_ros::LiveLandingZoneNode>(rclcpp::NodeOptions{});
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
