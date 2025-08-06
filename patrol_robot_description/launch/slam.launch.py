#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # 包路径
    pkg_path = FindPackageShare(package='patrol_robot_description').find('patrol_robot_description')
    
    # URDF文件路径
    xacro_file = os.path.join(pkg_path, 'urdf', 'patrol_robot.urdf.xacro')
    
    # 世界文件路径
    world_file = os.path.join(pkg_path, 'worlds', 'maze_world.world')
    
    # 处理URDF文件
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 设置环境变量
    gazebo_env = {
        'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib',
        'GAZEBO_MODEL_PATH': '/opt/ros/humble/share'
    }

    # 启动Gazebo
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver', 
            '--verbose',
            '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen',
        additional_env=gazebo_env
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env=gazebo_env
    )

    # 机器人状态发布
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 生成机器人
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'patrol_robot',
                    '-x', '0',
                    '-y', '0', 
                    '-z', '0.2'
                ],
                output='screen'
            )
        ]
    )

    # SLAM节点 (slam_toolbox)
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping'
        }]
    )

    # RViz2可视化
    rviz_config = os.path.join(pkg_path, 'config', 'slam_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        slam_node,
        rviz_node,
    ])