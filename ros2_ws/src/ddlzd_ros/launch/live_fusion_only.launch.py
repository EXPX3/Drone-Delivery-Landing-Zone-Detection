from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    detector = LifecycleNode(
        package="ddlzd_ros",
        executable="live_landing_zone_node",
        name="live_landing_zone",
        namespace="landing_zone",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "point_cloud_topic": LaunchConfiguration("point_cloud_topic"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "input_is_motion_compensated": ParameterValue(
                    LaunchConfiguration("input_is_motion_compensated"), value_type=bool
                ),
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="landing_zone_rviz",
        output="screen",
        arguments=[
            "-d", str(package_share / "rviz" / "live_landing_zones.rviz"),
            "-f", LaunchConfiguration("target_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=detector,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(detector),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))],
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=detector,
            goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(detector),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
            handle_once=True,
        )
    )
    required = [
        DeclareLaunchArgument("point_cloud_topic", description="Required PointCloud2 input topic."),
        DeclareLaunchArgument("image_topic", description="Required rectified camera Image topic."),
        DeclareLaunchArgument("camera_info_topic", description="Required calibrated CameraInfo topic."),
        DeclareLaunchArgument("target_frame", description="Required gravity-aligned local frame."),
        DeclareLaunchArgument(
            "input_is_motion_compensated",
            description="Required: true only when the upstream point cloud is already deskewed; false requires Ouster t.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=str(package_share / "config" / "live_fusion.yaml"),
            description="Complete algorithm, timing, fusion, and tracking parameter YAML.",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]
    return LaunchDescription(required + [detector, rviz, configure, activate])
