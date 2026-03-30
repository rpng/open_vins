from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


launch_args = [
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="tum_vi",
        description="euroc_mav, tum_vi, rpng_aruco, kaist",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined from 'config'.",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="number of cameras: 1 = mono, 2 = stereo, >2 = binocular",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if more than 1 camera, track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="bag_start",
        default_value="0.0",
        description="start time (seconds) into the bag; e.g. mh1: 40, mh2: 35",
    ),
    DeclareLaunchArgument(
        name="dataset",
        default_value="dataset-room1_512_16",
        description="dataset name, e.g. V1_01_easy, V2_02_medium, dataset-room1_512_16",
    ),
    DeclareLaunchArgument(
        name="bag",
        default_value="",
        description="path to the rosbag2 directory (or bag file). Overrides config+dataset if set.",
    ),
    DeclareLaunchArgument(
        name="dosave",
        default_value="false",
        description="record estimated trajectory to path_est",
    ),
    DeclareLaunchArgument(
        name="dotime",
        default_value="false",
        description="record timing statistics to path_time",
    ),
    DeclareLaunchArgument(
        name="path_est",
        default_value="/tmp/traj_estimate.txt",
        description="output path for estimated trajectory when dosave is true",
    ),
    DeclareLaunchArgument(
        name="path_time",
        default_value="/tmp/traj_timing.txt",
        description="output path for timing stats when dotime is true",
    ),
    DeclareLaunchArgument(
        name="dolivetraj",
        default_value="false",
        description="visualize aligned groundtruth path",
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value="",
        description="path to groundtruth file (csv ASL format). If empty and dolivetraj, derived from config+dataset.",
    ),
    DeclareLaunchArgument(
        name="debug",
        default_value="false",
        description="run with gdb debugger",
    ),
]


def launch_setup(context):
    config = LaunchConfiguration("config").perform(context)
    dataset = LaunchConfiguration("dataset").perform(context)
    bag_arg = LaunchConfiguration("bag").perform(context)

    # resolve config_path
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        if not os.path.isdir(configs_dir):
            return [
                LogInfo(msg="ERROR: ov_msckf config directory not found: {} - not starting.".format(configs_dir))
            ]
        available_configs = os.listdir(configs_dir)
        if config not in available_configs:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - available: {} - not starting.".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
        config_path = os.path.join(configs_dir, config, "estimator_config.yaml")
    if not os.path.isfile(config_path):
        return [
            LogInfo(msg="ERROR: config_path does not exist: {} - not starting.".format(config_path))
        ]

    # resolve bag path: explicit 'bag' arg, or construct from config + dataset (same as ROS1 default)
    if bag_arg:
        path_to_bag = bag_arg
    else:
        path_to_bag = os.path.join("/home/patrick/datasets", config, dataset)

    # resolve path_gt for dolivetraj (optional)
    path_gt = LaunchConfiguration("path_gt").perform(context)
    if not path_gt and LaunchConfiguration("dolivetraj").perform(context) == "true":
        try:
            ov_data_share = get_package_share_directory("ov_data")
            path_gt = os.path.join(ov_data_share, config, dataset + ".txt")
        except Exception:
            path_gt = ""

    # serial node parameters (same as ROS1 serial.launch)
    serial_params = [
        {"path_bag": path_to_bag},
        {"bag_start": LaunchConfiguration("bag_start")},
        {"bag_durr": -1.0},
        {"verbosity": LaunchConfiguration("verbosity")},
        {"config_path": config_path},
        {"use_stereo": LaunchConfiguration("use_stereo")},
        {"max_cameras": LaunchConfiguration("max_cameras")},
        {"record_timing_information": LaunchConfiguration("dotime")},
        {"record_timing_filepath": LaunchConfiguration("path_time")},
    ]
    if path_gt:
        serial_params.append({"path_gt": path_gt})

    serial_node = Node(
        package="ov_msckf",
        executable="ros2_serial_msckf",
        name="ov_msckf",
        output="screen",
        parameters=serial_params,
        prefix=['gdb -ex run --args'] if LaunchConfiguration("debug") == "true" else [],
    )

    actions = [serial_node]

    # optional: record trajectory to file (equivalent to ROS1 group if="$(arg dosave)")
    recorder_node = Node(
        package="ov_eval",
        executable="pose_to_file",
        name="recorder_estimate",
        output="screen",
        condition=IfCondition(LaunchConfiguration("dosave")),
        parameters=[
            {"topic": "/poseimu"},
            {"topic_type": "PoseWithCovarianceStamped"},
            {"output": LaunchConfiguration("path_est")},
        ],
    )
    actions.append(recorder_node)

    # ensure batch runs continue: stop the launch once the serial player exits.
    # without this, pose_to_file keeps spinning and ros2 launch never returns.
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=serial_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="serial node finished")),
                ],
            )
        )
    )

    #TODO(will): Add ROS2 support for live_align_trajectory, so dolivetraj 
    #            activates visualization of the ground truth trajectory.

    return actions


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
