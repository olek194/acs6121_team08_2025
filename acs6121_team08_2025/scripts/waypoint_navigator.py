#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient

STUCK_TIMEOUT = 8.0               # seconds

class WaypointNavigator(Node):
    REL_WPS = [(-1.5, 1.5), (-0.5, 1.5), (0.5, 1.5), (1.5, 1.5),
               (1.5, 0.5), (1.5, -0.5), (1.5, -1.5), (0.5, -1.5),
               (-0.5, -1.5), (-1.5, -1.5), (-1.5, -0.5), (-1.5, 0.5)]

    def __init__(self):
        super().__init__('waypoint_navigator')

        # action, odom, timer
        self._client  = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._odom_sub= self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._timer   = self.create_timer(1.0, self._check_stuck)

        # state
        self.initialised = False
        self.waypoints   = []
        self.current_idx = 0
        self.goal_start  = None
        self._goal_handle= None
        self._pending_wps= []

    # ---------- ROS callbacks ----------
    def _odom_cb(self, msg):
        if self.initialised:
            return          # already have first pose

        ix, iy = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.waypoints = [(ix+dx, iy+dy) for dx,dy in self.REL_WPS]
        self.initialised = True
        self.get_logger().info("Initial pose acquired. Sending waypoints.")
        self._send_wps(self.waypoints)

    def _fb_cb(self, fb_msg):
        self.current_idx = fb_msg.feedback.current_waypoint

    def _goal_resp_cb(self, future):
        self._goal_handle = future.result()

    def _check_stuck(self):
        if self.goal_start is None or self._goal_handle is None:
            return                      # nothing active

        elapsed = (self.get_clock().now() - self.goal_start).nanoseconds * 1e-9
        if elapsed < STUCK_TIMEOUT:
            return

        self.get_logger().warn(f"Stuck at wp {self.current_idx} for >{STUCK_TIMEOUT}s.")
        self._pending_wps = self.waypoints[self.current_idx + 1:]

        if not self._pending_wps:
            self.get_logger().info("No remaining waypoints – done.")
            self._timer.cancel()
            return

        # ask Nav2 to cancel FIRST, then send the shorter list
        self._goal_handle.cancel_goal_async().add_done_callback(self._cancel_done_cb)

    def _cancel_done_cb(self, future):
        # Update master list and reset indices
        self.waypoints   = self._pending_wps
        self._pending_wps= []
        self.current_idx = 0
        self.goal_start  = None
        self.get_logger().info("Previous goal cancelled – sending remaining waypoints.")
        self._send_wps(self.waypoints)

    # ---------- helpers ----------
    def _send_wps(self, wp_list):
        goal = FollowWaypoints.Goal()
        goal.poses = [self._pose(x,y) for x,y in wp_list]

        self._client.wait_for_server()
        self.goal_start = self.get_clock().now()
        self.current_idx= 0            # reset for new goal

        send_future = self._client.send_goal_async(goal, feedback_callback=self._fb_cb)
        send_future.add_done_callback(self._goal_resp_cb)

    def _pose(self,x,y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x,y,1.0
        return p

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointNavigator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
