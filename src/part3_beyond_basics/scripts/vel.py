#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MyVelocityController(Node):
    def __init__(self):
        super().__init__("velocity_controller")
        self.publisher = self.create_publisher(
            Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(
            2, self.time_cb)
        self.subscriber = self.create_subscription(
            LaserScan, "/scan", self.sub_cb, 10)
        self.rate = self.create_rate(
            frequency=15, clock=self.get_clock())
        self.vel = Twist()
    
    def time_cb(self):
        self.get_logger().info(
            f"Publishing velocities:\n"
            f"  linear = {self.vel.linear.x} m/s\n"
            f"  angular = {self.vel.angular.z} rad/s."
        )
    
    def sub_cb(self, data: LaserScan):
        left = min(data.ranges[0:10])
        right = min(data.ranges[-10:])
        
        self.vel.linear.x = 0.1
        self.vel.angular.z = 0.3 if left > right else -0.3
        self.publisher.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    node = MyVelocityController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()