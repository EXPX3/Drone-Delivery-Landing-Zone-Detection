"""Compatibility entry point for the former Ouster-only launch.

Despite the historical filename, this launch has no Gremsy dependency.
New deployments should use live_lidar.launch.py directly.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "live_lidar.launch.py")),
        launch_arguments={
            "sensor_model": LaunchConfiguration("sensor_model"),
            "operating_mode": LaunchConfiguration("operating_mode"),
            "sensor_hostname": LaunchConfiguration("sensor_hostname"),
            "lidar_mode": LaunchConfiguration("lidar_mode"),
            "timestamp_mode": LaunchConfiguration("timestamp_mode"),
            "target_frame": LaunchConfiguration("target_frame"),
            "config_file": LaunchConfiguration("config_file"),
            "local_pose_topic": LaunchConfiguration("local_pose_topic"),
            "gnss_topic": LaunchConfiguration("gnss_topic"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "sensor_model", default_value="os1_rev7",
            description="os1_rev7 or os1_rev8"),
        DeclareLaunchArgument("operating_mode", default_value="ranking"),
        DeclareLaunchArgument("sensor_hostname", description="Required Ouster hostname or IP"),
        DeclareLaunchArgument("lidar_mode", default_value="1024x10"),
        DeclareLaunchArgument("timestamp_mode", default_value="TIME_FROM_ROS_TIME"),
        DeclareLaunchArgument("target_frame", description="Required gravity-aligned ENU frame"),
        DeclareLaunchArgument(
            "config_file", default_value=str(package_share / "config" / "live_fusion.yaml")),
        DeclareLaunchArgument("local_pose_topic", default_value="/mavros/local_position/pose"),
        DeclareLaunchArgument("gnss_topic", default_value="/mavros/global_position/global"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        launch,
    ])
