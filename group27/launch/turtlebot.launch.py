from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

# This function is needed

from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory('group27'),
        'config',
        'params.yaml'
    )
    # use sim_time:=true to use simulation time
    ld = LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above
    ])

    broadcaster_pub = Node(
        package="group27",
        executable="broadcaster_pub",
        parameters=[parameter_file, {'use_sim_time': True}],
        output='screen',
    )
    listener_sub = Node(
        package="group27",
        executable="listener_sub",
        parameters=[parameter_file, {'use_sim_time': True}],
        output='screen',
    )
    battery = Node(
        package="group27",
        executable="battery",
        parameters=[parameter_file, {'use_sim_time': True}],
        output='screen',
    )
   
    ld.add_action(broadcaster_pub)
    ld.add_action(listener_sub)
    ld.add_action(battery)
    # ld.add_action(gazebo_launch)

    return ld
