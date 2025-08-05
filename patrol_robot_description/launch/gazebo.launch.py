
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # 包路径
    pkg_path = FindPackageShare(package='patrol_robot_description').find('patrol_robot_description')
    
    # URDF文件路径
    xacro_file = os.path.join(pkg_path, 'urdf', 'patrol_robot.urdf.xacro')
    
    # 处理URDF文件
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 设置环境变量
    gazebo_env = {
        'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib',
        'GAZEBO_MODEL_PATH': '/opt/ros/humble/share'
    }

    # 启动Gazebo服务器（使用系统默认的空世界）
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver', 
            '--verbose',
            '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        additional_env=gazebo_env
    )
    
    # 启动Gazebo客户端
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env=gazebo_env
    )

    # 发布机器人状态
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 延迟启动spawn_entity（等Gazebo完全启动）
    spawn_entity = TimerAction(
        period=3.0,  # 等待3秒
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

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
    ])