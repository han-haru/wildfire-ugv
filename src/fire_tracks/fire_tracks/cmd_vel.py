#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        # 파라미터 선언 (선속도, 각속도)
        self.declare_parameter('x', 20)   # m/s
        self.declare_parameter('wz', 0.0)  # rad/s
        self.declare_parameter('rate', 10) # Hz

        # 파라미터 불러오기
        self.x = float(self.get_parameter('x').value)
        self.wz = float(self.get_parameter('wz').value)
        self.rate = float(self.get_parameter('rate').value)

        # 퍼블리셔 생성
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 주기 타이머 생성
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.wz
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing cmd_vel: v={self.x:.2f}, wz={self.wz:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
