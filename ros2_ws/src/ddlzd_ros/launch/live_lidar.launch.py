from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import FrontendLaunchDescriptionSource
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
        raise RuntimeError(f"sensor_model must be one of {sorted(SENSORS)}, got {sensor!r}")
    mode = LaunchConfiguration("operating_mode").perform(context)
    if mode not in {"ranking", "sticky"}:
        raise RuntimeError("operating_mode must be ranking or sticky")

    target_frame = LaunchConfiguration("target_frame").perform(context)
    if not target_frame:
        raise RuntimeError("target_frame is required and must be a gravity-aligned ENU frame")
    profile = package_share / "config" / "sensors" / f"{sensor}.yaml"
    config_file = LaunchConfiguration("config_file").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    rviz_config_arg = LaunchConfiguration("rviz_config").perform(context)
    rviz_config = Path(rviz_config_arg) if rviz_config_arg else (
        package_share / "rviz" / "live_landing_zones.rviz")
    input_motion_compensated = _as_bool(
        LaunchConfiguration("input_is_motion_compensated").perform(context))

    actions = []
    if sensor == "vlp16":
        device_ip = LaunchConfiguration("velodyne_ip").perform(context)
        if not device_ip:
            raise RuntimeError("velodyne_ip is required for sensor_model=vlp16")
        driver_share = Path(get_package_share_directory("velodyne_driver"))
        pointcloud_share = Path(get_package_share_directory("velodyne_pointcloud"))
        driver_params = yaml.safe_load(
            (driver_share / "config" / "VLP16-velodyne_driver_node-params.yaml").read_text()
        )["velodyne_driver_node"]["ros__parameters"]
        driver_params.update({
            "device_ip": device_ip,
            "frame_id": LaunchConfiguration("lidar_frame").perform(context),
            "model": "VLP16",
        })
        transform_params = yaml.safe_load(
            (pointcloud_share / "config" / "VLP16-velodyne_transform_node-params.yaml").read_text()
        )["velodyne_transform_node"]["ros__parameters"]
        transform_params.update({
            "calibration": str(pointcloud_share / "params" / "VLP16db.yaml"),
            "model": "VLP16",
            "organize_cloud": False,
        })
        actions.extend([
            Node(
                package="velodyne_driver", executable="velodyne_driver_node",
                name="velodyne_driver_node", output="screen", parameters=[driver_params]),
            Node(
                package="velodyne_pointcloud", executable="velodyne_transform_node",
                name="velodyne_transform_node", output="screen", parameters=[transform_params]),
        ])
        point_topic = "/velodyne_points"
        mount_frame = LaunchConfiguration("lidar_frame").perform(context)
    else:
        sensor_hostname = LaunchConfiguration("sensor_hostname").perform(context)
        if not sensor_hostname:
            raise RuntimeError("sensor_hostname is required for Ouster sensors")
        ouster_share = Path(get_package_share_directory("ouster_ros"))
        # FW 3.1 omits the WINDOW channel from the single-return profile while
        # the Rev8-capable driver expects it. LEGACY retains per-point time and
        # is the compatible wire profile for the Rev7 sensor.
        udp_profile_lidar = "LEGACY" if sensor == "os1_rev7" else ""
        actions.append(IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(str(ouster_share / "launch" / "sensor.launch.xml")),
            launch_arguments={
                "sensor_hostname": sensor_hostname,
                "lidar_mode": LaunchConfiguration("lidar_mode").perform(context),
                "timestamp_mode": LaunchConfiguration("timestamp_mode").perform(context),
                "udp_profile_lidar": udp_profile_lidar,
                "sensor_frame": "os_sensor",
                "lidar_frame": "os_lidar",
                # Publish the cloud in Ouster's housing-centric FLU frame.
                # The driver applies its metadata-defined lidar->sensor
                # 180-degree-Z conversion before publishing.
                "point_cloud_frame": "os_sensor",
                "pub_static_tf": "true",
                # The Ouster launch also has an operating_mode argument. Keep
                # it independent of this detector's ranking/sticky mode.
                "operating_mode": "NORMAL",
                "point_type": "original",
                "organized": "false",
                "proc_mask": "PCL|IMU",
                "viz": "false",
                "attempt_reconnect": "true",
            }.items(),
        ))
        point_topic = "/ouster/points"
        mount_frame = LaunchConfiguration("ouster_mount_frame").perform(context)

    if _as_bool(LaunchConfiguration("publish_lidar_static_tf").perform(context)):
        if sensor in {"os1_rev7", "os1_rev8"}:
            # Alta-X -> os_sensor, specified in the housing-centric Ouster
            # frame.  +X_sensor=-Y_altax, +Y_sensor=-Z_altax,
            # +Z_sensor=+X_altax.
            rotation_args = [
                "--qx", "0.500000", "--qy", "-0.500000",
                "--qz", "0.500000", "--qw", "-0.500000",
            ]
        else:
            rotation_args = [
                "--roll", LaunchConfiguration("lidar_roll").perform(context),
                "--pitch", LaunchConfiguration("lidar_pitch").perform(context),
                "--yaw", LaunchConfiguration("lidar_yaw").perform(context),
            ]
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_mount_static_tf",
            output="screen",
            arguments=[
                "--frame-id", LaunchConfiguration("base_frame").perform(context),
                "--child-frame-id", mount_frame,
                "--x", LaunchConfiguration("lidar_x").perform(context),
                "--y", LaunchConfiguration("lidar_y").perform(context),
                "--z", LaunchConfiguration("lidar_z").perform(context),
                *rotation_args,
            ],
        ))

    detector = LifecycleNode(
        package="ddlzd_ros",
        executable="live_landing_zone_node",
        name="live_landing_zone",
        namespace="landing_zone",
        output="screen",
        parameters=[
            config_file,
            str(profile),
            {
                "operating_mode": mode,
                "point_cloud_topic": point_topic,
                "target_frame": target_frame,
                "local_pose_topic": LaunchConfiguration("local_pose_topic").perform(context),
                "gnss_topic": LaunchConfiguration("gnss_topic").perform(context),
                "input_is_motion_compensated": input_motion_compensated,
                "use_sim_time": False,
            },
        ],
    )
    rviz_arguments = ["-d", str(rviz_config)]
    if not rviz_config_arg:
        rviz_arguments += ["-f", target_frame]
    rviz = Node(
        package="rviz2", executable="rviz2", name="landing_zone_rviz", output="screen",
        arguments=rviz_arguments,
        condition=IfCondition(use_rviz),
    )
    configure = RegisterEventHandler(OnProcessStart(
        target_action=detector,
        on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(detector),
            transition_id=Transition.TRANSITION_CONFIGURE))],
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=detector,
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(detector),
            transition_id=Transition.TRANSITION_ACTIVATE))],
        handle_once=True,
    ))
    return actions + [detector, rviz, configure, activate]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ddlzd_ros"))
    arguments = [
        DeclareLaunchArgument("sensor_model", description="vlp16, os1_rev7, or os1_rev8"),
        DeclareLaunchArgument("operating_mode", default_value="ranking", description="ranking or sticky"),
        DeclareLaunchArgument("target_frame", description="Required gravity-aligned ENU frame, normally map"),
        DeclareLaunchArgument("config_file", default_value=str(package_share / "config" / "live_fusion.yaml")),
        DeclareLaunchArgument("local_pose_topic", default_value="/mavros/local_position/pose"),
        DeclareLaunchArgument("gnss_topic", default_value="/mavros/global_position/global"),
        DeclareLaunchArgument("input_is_motion_compensated", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config", default_value="",
            description="Optional RViz config override (e.g. os_sensor raw-cloud view)"),
        DeclareLaunchArgument("sensor_hostname", default_value=""),
        DeclareLaunchArgument("lidar_mode", default_value="1024x10"),
        DeclareLaunchArgument("timestamp_mode", default_value="TIME_FROM_ROS_TIME"),
        DeclareLaunchArgument("velodyne_ip", default_value=""),
        DeclareLaunchArgument("lidar_frame", default_value="velodyne"),
        DeclareLaunchArgument("ouster_mount_frame", default_value="os_sensor"),
        DeclareLaunchArgument("publish_lidar_static_tf", default_value="false"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.0"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])
