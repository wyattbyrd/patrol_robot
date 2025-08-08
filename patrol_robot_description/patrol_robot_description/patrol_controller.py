#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import String
from enum import Enum
import time
import math

class PatrolState(Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    OBSERVING = "observing"
    PAUSED = "paused"
    ERROR = "error"

class PatrolController(Node):
    def __init__(self):
        super().__init__('patrol_controller')
        
        # 巡逻状态
        self.current_state = PatrolState.IDLE
        self.current_waypoint_index = 0
        
        # 预定义巡逻路径点（"回"字走廊的4个关键点）
        self.waypoints = [
            {'x': 0.0, 'y': 2.2, 'name': 'North Corridor'},      # 北走廊中段
            {'x': 2.2, 'y': 0.0, 'name': 'East Corridor'},       # 东走廊中段  
            {'x': 0.0, 'y': -2.2, 'name': 'South Corridor'},     # 南走廊中段
            {'x': -2.2, 'y': 0.0, 'name': 'West Corridor'}       # 西走廊中段
        ]
        
        # Action客户端
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # 发布者：巡逻状态
        self.status_publisher = self.create_publisher(String, 'patrol_status', 10)
        
        # 定时器：状态管理和监控
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.patrol_timer = None
        
        # 巡逻参数
        self.observation_time = 3.0  # 每个点观察3秒
        self.patrol_loop_enabled = True
        
        self.get_logger().info('巡逻控制器初始化完成')
        self.get_logger().info(f'预设巡逻点数量: {len(self.waypoints)}')
        
    def create_pose_stamped(self, x, y, yaw=0.0):
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 将yaw角度转换为四元数
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def start_patrol(self):
        """开始巡逻"""
        if self.current_state in [PatrolState.PATROLLING, PatrolState.OBSERVING]:
            self.get_logger().warn('巡逻已在进行中')
            return False
            
        self.current_state = PatrolState.PATROLLING
        self.current_waypoint_index = 0
        self.get_logger().info('开始巡逻任务')
        
        # 等待导航服务可用
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('导航服务不可用')
            self.current_state = PatrolState.ERROR
            return False
            
        self._navigate_to_next_waypoint()
        return True
    
    def stop_patrol(self):
        """停止巡逻"""
        self.current_state = PatrolState.IDLE
        self.get_logger().info('巡逻任务已停止')
        
        # 取消当前导航目标
        if self.nav_to_pose_client.get_goal_handle() is not None:
            self.nav_to_pose_client.cancel_goal()
    
    def pause_patrol(self):
        """暂停巡逻"""
        if self.current_state == PatrolState.PATROLLING:
            self.current_state = PatrolState.PAUSED
            self.get_logger().info('巡逻已暂停')
    
    def resume_patrol(self):
        """恢复巡逻"""
        if self.current_state == PatrolState.PAUSED:
            self.current_state = PatrolState.PATROLLING
            self.get_logger().info('巡逻已恢复')
            self._navigate_to_next_waypoint()
    
    def _navigate_to_next_waypoint(self):
        """导航到下一个巡逻点"""
        if self.current_waypoint_index >= len(self.waypoints):
            if self.patrol_loop_enabled:
                self.current_waypoint_index = 0  # 循环巡逻
                self.get_logger().info('完成一轮巡逻，开始新一轮')
            else:
                self.get_logger().info('巡逻任务完成')
                self.current_state = PatrolState.IDLE
                return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        target_pose = self.create_pose_stamped(waypoint['x'], waypoint['y'])
        
        # 创建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        self.get_logger().info(f'导航到巡逻点 {self.current_waypoint_index + 1}: {waypoint["name"]} '
                              f'({waypoint["x"]:.1f}, {waypoint["y"]:.1f})')
        
        # 发送导航目标
        self.nav_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        self.nav_future.add_done_callback(self._nav_goal_response_callback)
    
    def _nav_feedback_callback(self, feedback_msg):
        """导航反馈回调"""
        feedback = feedback_msg.feedback
        # 可以在这里添加导航进度的处理逻辑
        pass
    
    def _nav_goal_response_callback(self, future):
        """导航目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            self.current_state = PatrolState.ERROR
            return
        
        self.get_logger().info('导航目标已接受')
        self._goal_result_future = goal_handle.get_result_async()
        self._goal_result_future.add_done_callback(self._nav_result_callback)
    
    def _nav_result_callback(self, future):
        """导航结果回调"""
        result = future.result().result
        
        if result:
            waypoint = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f'成功到达巡逻点: {waypoint["name"]}')
            
            # 切换到观察状态
            self.current_state = PatrolState.OBSERVING
            self.get_logger().info(f'开始观察模式，持续 {self.observation_time} 秒')
            
            # 设置观察计时器
            if self.patrol_timer is not None:
                self.patrol_timer.cancel()
            self.patrol_timer = self.create_timer(
                self.observation_time, 
                self._observation_complete
            )
        else:
            self.get_logger().error('导航失败')
            self.current_state = PatrolState.ERROR
    
    def _observation_complete(self):
        """观察完成，继续下一个巡逻点"""
        if self.patrol_timer is not None:
            self.patrol_timer.cancel()
            self.patrol_timer = None
        
        if self.current_state == PatrolState.OBSERVING:
            self.get_logger().info('观察完成，继续巡逻')
            self.current_state = PatrolState.PATROLLING
            self.current_waypoint_index += 1
            self._navigate_to_next_waypoint()
    
    def publish_status(self):
        """发布巡逻状态"""
        status_msg = String()
        if self.current_state == PatrolState.PATROLLING:
            waypoint = self.waypoints[self.current_waypoint_index]
            status_msg.data = f"PATROLLING to {waypoint['name']} ({self.current_waypoint_index + 1}/{len(self.waypoints)})"
        elif self.current_state == PatrolState.OBSERVING:
            waypoint = self.waypoints[self.current_waypoint_index]
            status_msg.data = f"OBSERVING at {waypoint['name']}"
        else:
            status_msg.data = f"{self.current_state.value.upper()}"
        
        self.status_publisher.publish(status_msg)
    
    def get_status_info(self):
        """获取详细状态信息"""
        return {
            'state': self.current_state.value,
            'current_waypoint': self.current_waypoint_index,
            'total_waypoints': len(self.waypoints),
            'loop_enabled': self.patrol_loop_enabled
        }

def main(args=None):
    rclpy.init(args=args)
    
    patrol_controller = PatrolController()
    
    try:
        # 启动巡逻（可以通过服务或话题控制）
        patrol_controller.get_logger().info('巡逻控制器启动，等待指令...')
        patrol_controller.get_logger().info('提示: 可以调用 start_patrol() 方法开始巡逻')
        
        rclpy.spin(patrol_controller)
    except KeyboardInterrupt:
        patrol_controller.get_logger().info('收到退出信号')
    finally:
        patrol_controller.stop_patrol()
        patrol_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()