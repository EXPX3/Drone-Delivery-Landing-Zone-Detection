import json
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _as_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _setup(context):
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    package_prefix = Path(get_package_prefix("ddlzd_ros"))
    bag_path = LaunchConfiguration("bag_path").perform(context)
    if not bag_path:
        raise RuntimeError("bag_path is required")
    sensor_request = LaunchConfiguration("sensor_model").perform(context)
    point_request = LaunchConfiguration("point_cloud_topic").perform(context)
    helper = package_prefix / "lib" / "ddlzd_ros" / "detect_bag_sensor.py"
    command = [str(helper), bag_path, "--sensor", sensor_request]
    if point_request:
        command.extend(["--point-topic", point_request])
    try:
        resolved = json.loads(subprocess.check_output(command, text=True).strip())
    except subprocess.CalledProcessError as error:
        raise RuntimeError("unable to resolve bag sensor; see detector message above") from error

    sensor = resolved["sensor_model"]
    point_topic = resolved["point_cloud_topic"]
    profile = package_share / "config" / "sensors" / f"{sensor}.yaml"
    target_frame = LaunchConfiguration("target_frame").perform(context)
    if not target_frame:
        raise RuntimeError("target_frame is required and must be a gravity-aligned ENU frame")
    mode = LaunchConfiguration("operating_mode").perform(context)
    if mode not in {"ranking", "sticky"}:
        raise RuntimeError("operating_mode must be ranking or sticky")

    detector = LifecycleNode(
        package="ddlzd_ros", executable="live_landing_zone_node",
        name="live_landing_zone", namespace="landing_zone", output="screen",
        parameters=[
            LaunchConfiguration("config_file").perform(context),
            str(profile),
            {
                "sensor.model": sensor,
                "operating_mode": mode,
                "point_cloud_topic": point_topic,
                "target_frame": target_frame,
                "local_pose_topic": LaunchConfiguration("local_pose_topic").perform(context),
                "gnss_topic": LaunchConfiguration("gnss_topic").perform(context),
                "input_is_motion_compensated": _as_bool(
                    LaunchConfiguration("input_is_motion_compensated").perform(context)),
                "use_sim_time": True,
            },
        ],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="landing_zone_rviz", output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", str(package_share / "rviz" / "live_landing_zones.rviz"), "-f", target_frame],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    player = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", bag_path, "--clock", "--rate",
            LaunchConfiguration("playback_rate").perform(context),
        ],
        output="screen",
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
    stop_when_done = RegisterEventHandler(OnProcessExit(
        target_action=player,
        on_exit=[Shutdown(reason="bag playback completed")],
    ))
    return [detector, rviz, configure, activate, TimerAction(period=2.0, actions=[player]), stop_when_done]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", description="ROS 2 bag directory or MCAP file"),
        DeclareLaunchArgument("sensor_model", default_value="auto", description="auto, vlp16, os1_rev7, or os1_rev8"),
        DeclareLaunchArgument("point_cloud_topic", default_value="", description="Required only when bag auto-detection is ambiguous"),
        DeclareLaunchArgument("operating_mode", default_value="ranking", description="ranking or sticky"),
        DeclareLaunchArgument("target_frame", description="Required gravity-aligned ENU frame"),
        DeclareLaunchArgument("config_file", default_value=str(package_share / "config" / "live_fusion.yaml")),
        DeclareLaunchArgument("local_pose_topic", default_value="/mavros/local_position/pose"),
        DeclareLaunchArgument("gnss_topic", default_value="/mavros/global_position/global"),
        DeclareLaunchArgument("input_is_motion_compensated", default_value="false"),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
