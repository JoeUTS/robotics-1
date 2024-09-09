#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # settings
    robotLaunch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_autostart = LaunchConfiguration('autostart', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2')
    y_pose = LaunchConfiguration('y_pose', default='0')
    lifecycle_nodes = ['map_server']
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('sprint2'),
        'rviz',
        'test_config.rviz'
    )
    slam_params_file = os.path.join(
        get_package_share_directory('sprint2'),
        'config',
        'mapper_params_online_async.yaml'
    )
    map_file = os.path.join(
        get_package_share_directory('sprint2'),
        'map',
        'test_map_serial.yaml'
    )

    # nodes
    example_node = Node(
        package="sprint2",
        executable="example_node"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    map_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename':map_file}, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': use_autostart},
                        {'node_names': lifecycle_nodes}])

    # gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # robot
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robotLaunch, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robotLaunch, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # add actions to launch
    ld.add_action(example_node)
    ld.add_action(rviz_node)
    ld.add_action(slam_node)
    ld.add_action(map_node)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld