#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# NOTE: This code is just a modified version of the turtlebot3_gazebo empty_world launch file

def generate_launch_description():
    # launch file location
    launch_file_dir = os.path.join(
        get_package_share_directory('sprint1'),
        'launch'
    )
    
    # gazebo file location
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # define world
    world = os.path.join(
        os.path.dirname(__file__),
        '../worlds/test_world.world'
    )

    # set default parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='9.0')
    y_pose = LaunchConfiguration('y_pose', default='-8.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='1.570796')

    # assuming to be a gazebo thing
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros,
            'launch',
            'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': world}.items()
    )

    # assuming to be a gazebo thing
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros,
            'launch',
            'gzclient.launch.py'
            )
        )
    )

    # need to include
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # spawns in turtlebot at predefined location
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose
        }.items()
    )

    ld = LaunchDescription([Node(
            package='sprint1',
            namespace='example_node',
            executable='example_node',
            name='sim'
        )
    ])

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld