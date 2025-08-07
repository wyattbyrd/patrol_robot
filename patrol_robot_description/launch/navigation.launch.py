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
    
    # 配置文件路径
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2', 'nav2_params.yaml')
    amcl_config_file = os.path.join(pkg_path, 'config', 'nav2', 'amcl_config.yaml')
    map_file = os.path.join(pkg_path, 'maps', 'my_first_map.yaml')
    world_file = os.path.join(pkg_path, 'worlds', 'maze_world.world')
    
    # URDF文件路径
    xacro_file = os.path.join(pkg_path, 'urdf', 'patrol_robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Gazebo环境变量
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
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
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
                    '-z', '0.1',
                    '-R', '0',      # 设置初始朝向
                    '-P', '0',
                    '-Y', '0'
                ],
                output='screen'
            )
        ]
    )

    # 地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': True
        }]
    )

    # AMCL定位 - 优化参数
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'scan_topic': 'scan',
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            # 增加粒子数量，提高定位精度
            'max_particles': 3000,
            'min_particles': 1000,
            # 设置合理的初始位置不确定性
            'initial_pose': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'yaw': 0.0
            },
            'set_initial_pose': True,  # 设置初始位置
            # 提高定位稳定性
            'update_min_d': 0.1,      # 移动0.1m后更新
            'update_min_a': 0.1,      # 旋转0.1rad后更新
            'transform_tolerance': 0.5,
            'recovery_alpha_fast': 0.1,
            'recovery_alpha_slow': 0.001
        }]
    )
    # Navigation2 Stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare(package='nav2_bringup').find('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'slam': 'false'  # 使用已保存的地图，不进行SLAM
        }.items()
    )

    # 生命周期管理
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # RViz2可视化
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
            )
        ]
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        map_server,
        amcl,
        lifecycle_manager,
        nav2_bringup,
        rviz_node,
    ])