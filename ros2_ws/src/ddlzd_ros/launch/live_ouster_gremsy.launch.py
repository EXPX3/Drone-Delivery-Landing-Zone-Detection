from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    ouster_share = Path(get_package_share_directory("ouster_ros"))

    sensor_hostname = LaunchConfiguration("sensor_hostname")
    lidar_mode = LaunchConfiguration("lidar_mode")
    timestamp_mode = LaunchConfiguration("timestamp_mode")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    target_frame = LaunchConfiguration("target_frame")

    ouster = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(str(ouster_share / "launch" / "sensor.launch.xml")),
        launch_arguments={
            "sensor_hostname": sensor_hostname,
            "lidar_mode": lidar_mode,
            "timestamp_mode": timestamp_mode,
            "point_type": "original",
            "organized": "false",
            "proc_mask": "PCL|IMU",
            "viz": "false",
            "attempt_reconnect": "true",
        }.items(),
    )

    detector = LifecycleNode(
        package="ddlzd_ros",
        executable="live_landing_zone_node",
        name="live_landing_zone",
        namespace="landing_zone",
        output="screen",
        parameters=[
            str(package_share / "config" / "live_fusion.yaml"),
            {
                "point_cloud_topic": "/ouster/points",
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "target_frame": target_frame,
                "input_is_motion_compensated": False,
            },
        ],
    )

    configure_detector = RegisterEventHandler(
        OnProcessStart(
            target_action=detector,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(detector),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )
    activate_detector = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=detector,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(detector),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
            handle_once=True,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_hostname",
                description="Required OS1 hostname or IP address.",
            ),
            DeclareLaunchArgument(
                "lidar_mode",
                description="Required OS1 mode supported by the installed sensor, such as 1024x10.",
            ),
            DeclareLaunchArgument(
                "timestamp_mode",
                description="Required Ouster time source configured for the vehicle time architecture.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                description="Required rectified Gremsy BGR/RGB sensor_msgs/Image topic.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                description="Required calibrated sensor_msgs/CameraInfo topic matching image_topic.",
            ),
            DeclareLaunchArgument(
                "target_frame",
                description="Required gravity-aligned local frame with timestamped LiDAR and gimbal/camera TF.",
            ),
            ouster,
            detector,
            configure_detector,
            activate_detector,
        ]
    )
