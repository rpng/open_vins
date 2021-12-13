from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

launch_args = [
    DeclareLaunchArgument(name='namespace',         default_value='',           description='namespace'),
    DeclareLaunchArgument(name='ov_enable',         default_value='true',       description='enable openvins node'),
    DeclareLaunchArgument(name='rviz_enable',       default_value='true',       description='enable rviz node'),
    DeclareLaunchArgument(name='config',            default_value='euroc_mav',  description='euroc_mav, tum_vi, rpng_aruco...'),
    DeclareLaunchArgument(name='verbosity',         default_value='INFO',       description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT'),
    DeclareLaunchArgument(name='use_stereo',        default_value='true',       description=''),
    DeclareLaunchArgument(name='max_cameras',       default_value='2',          description='')
]


def launch_setup(context):
    configs_dir=os.path.join(get_package_share_directory('ov_msckf'),'..','config')
    available_configs = os.listdir(configs_dir)
    config = LaunchConfiguration('config').perform(context)
    if not config in available_configs:
        return[LogInfo(msg='ERROR: unknown config: \'{}\' - Available configs are: {} - not starting OpenVINS'.format(config,', '.join(available_configs)))]
    config_path = os.path.join(get_package_share_directory('ov_msckf'),'config',config,'estimator_config.yaml')
    node1 = Node(package = 'ov_msckf',
                 executable = 'run_subscribe_msckf',
                 condition = IfCondition(LaunchConfiguration('ov_enable')),
                 namespace = LaunchConfiguration('namespace'),
                 parameters =[{'verbosity': LaunchConfiguration('verbosity')},
                              {'use_stereo': LaunchConfiguration('use_stereo')},
                              {'max_cameras': LaunchConfiguration('max_cameras')},
                              {'config_path': config_path}])

    node2 = Node(package = 'rviz2',
                 executable = 'rviz2',
                 condition = IfCondition(LaunchConfiguration('rviz_enable')),
                 arguments = ['-d'+os.path.join(get_package_share_directory('ov_msckf'),'launch','display_ros2.rviz'), '--ros-args', '--log-level', 'warn'])

    return [node1, node2]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
