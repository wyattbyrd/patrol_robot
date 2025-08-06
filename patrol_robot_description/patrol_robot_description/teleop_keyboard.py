#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios

msg = """
控制机器人移动:
---------------------------
   u    i    o
   j    k    l
   m    ,    .

q/z : 增加/减少最大速度 10%
w/x : 增加/减少线速度 10%
e/c : 增加/减少角速度 10%

空格键 : 强制停止

CTRL-C 退出
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.status = 0
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

def main():
    rclpy.init()
    
    teleop = TeleopKeyboard()
    teleop.settings = termios.tcgetattr(sys.stdin)
    
    print(msg)
    print(teleop.vels(teleop.speed, teleop.turn))
    
    try:
        while True:
            key = teleop.get_key()
            if key in moveBindings.keys():
                teleop.x = moveBindings[key][0]
                teleop.th = moveBindings[key][1]
            elif key in speedBindings.keys():
                teleop.speed = teleop.speed * speedBindings[key][0]
                teleop.turn = teleop.turn * speedBindings[key][1]
                print(teleop.vels(teleop.speed, teleop.turn))
                if teleop.status == 14:
                    print(msg)
                teleop.status = (teleop.status + 1) % 15
            elif key == ' ':
                teleop.x = 0
                teleop.th = 0
            else:
                if key == '\x03':  # Ctrl-C
                    break
                    
            twist = Twist()
            twist.linear.x = teleop.x * teleop.speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = teleop.th * teleop.turn
            teleop.publisher_.publish(twist)
                
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        teleop.publisher_.publish(twist)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop.settings)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()