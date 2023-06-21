import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("dobot_description"),
            "urdf",
            "dobot_moveit_desc.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "dobot_moveit_config", "config/dobot_palitra.srdf"
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'dobot_moveit_config', 'config/kinematics.yaml'
    )
    use_sim_time = {
        'use_sim_time': True
    }

    core_config = os.path.join(
        get_package_share_directory('manipulator_ctrl_pkg'),
        'config',
        'config.yaml'
    )

    dobot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('dobot_moveit_config'), 'launch'),
            '/dobot_moveit.launch.py']),
        launch_arguments={
            'robot_ip': '192.168.6.1',
            'use_fake_hardware': 'True',
            'fake_sensor_commands': 'True'
        }.items()
    )

    manipulator = Node(
        package="manipulator_ctrl_pkg",
        executable="controller",
        output="screen",
        parameters=[robot_description, robot_description_semantic,
                    kinematics_yaml, use_sim_time, core_config]
    )

    arAlgos = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('cobot_ar_pkg'), 'launch'),
            '/start.launch.py'
        ]),
    )

    return LaunchDescription([
        arAlgos,
        dobot,
        manipulator
    ])
