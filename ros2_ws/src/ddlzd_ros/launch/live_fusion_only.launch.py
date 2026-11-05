"""Run only the detector against an externally provided PointCloud2 topic."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


SENSORS = {"vlp16", "os1_rev7", "os1_rev8"}


def _as_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _setup(context):
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    sensor = LaunchConfiguration("sensor_model").perform(context)
    if sensor not in SENSORS:
        raise RuntimeError(f"sensor_model must be one of {sorted(SENSORS)}")
    mode = LaunchConfiguration("operating_mode").perform(context)
    if mode not in {"ranking", "sticky"}:
        raise RuntimeError("operating_mode must be ranking or sticky")
    target_frame = LaunchConfiguration("target_frame").perform(context)
    if not target_frame:
        raise RuntimeError("target_frame is required and must be a gravity-aligned ENU frame")

    detector = LifecycleNode(
        package="ddlzd_ros", executable="live_landing_zone_node",
        name="live_landing_zone", namespace="landing_zone", output="screen",
        parameters=[
            LaunchConfiguration("config_file").perform(context),
            str(package_share / "config" / "sensors" / f"{sensor}.yaml"),
            {
                "operating_mode": mode,
                "point_cloud_topic": LaunchConfiguration("point_cloud_topic").perform(context),
                "target_frame": target_frame,
                "local_pose_topic": LaunchConfiguration("local_pose_topic").perform(context),
                "gnss_topic": LaunchConfiguration("gnss_topic").perform(context),
                "input_is_motion_compensated": _as_bool(
                    LaunchConfiguration("input_is_motion_compensated").perform(context)),
            },
        ],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="landing_zone_rviz", output="screen",
        arguments=["-d", str(package_share / "rviz" / "live_landing_zones.rviz"),
                   "-f", target_frame],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    configure = RegisterEventHandler(OnProcessStart(
        target_action=detector,
        on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(detector),
            transition_id=Transition.TRANSITION_CONFIGURE))],
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=detector, goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(detector),
            transition_id=Transition.TRANSITION_ACTIVATE))],
        handle_once=True,
    ))
    return [detector, rviz, configure, activate]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    return LaunchDescription([
        DeclareLaunchArgument("sensor_model", description="vlp16, os1_rev7, or os1_rev8"),
        DeclareLaunchArgument("operating_mode", default_value="ranking"),
        DeclareLaunchArgument("point_cloud_topic", description="Required PointCloud2 input topic"),
        DeclareLaunchArgument("target_frame", description="Required gravity-aligned ENU frame"),
        DeclareLaunchArgument("local_pose_topic", default_value="/mavros/local_position/pose"),
        DeclareLaunchArgument("gnss_topic", default_value="/mavros/global_position/global"),
        DeclareLaunchArgument("input_is_motion_compensated", default_value="false"),
        DeclareLaunchArgument(
            "config_file", default_value=str(package_share / "config" / "live_fusion.yaml")),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
