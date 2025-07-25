#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
#from prius_msgs.msg import Control
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Subscriptions and Publishers
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.cmd_prius = self.create_publisher(Control, '/prius/control', 10)

        # Parameters and state
        self.timer = self.create_timer(0.1, self.control_loop)
        self.path = np.array([])
        self.pose = None
        self.yaw = 0.0
        self.speed = 0.0
        
        self.declare_parameter('max_steering', 1.22)
        self.max_steering = self.get_parameter('max_steering').get_parameter_value().double_value
        
        self.declare_parameter('look_ahead_dist', 2.5)
        self.look_ahead_dist = self.get_parameter('look_ahead_dist').get_parameter_value().double_value
        
        self.declare_parameter('kpp', 0.5)
        self.kpp = self.get_parameter('kpp').get_parameter_value().double_value
        
        self.declare_parameter('angle_offset', 1.5707963)
        self.angle_offset = self.get_parameter('angle_offset').get_parameter_value().double_value
        
        self.declare_parameter('plot_path', False)
        self.plot_path = self.get_parameter('plot_path').get_parameter_value().bool_value
        self.real_path = []

    def path_callback(self, msg: Path):
        self.path = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
        
        
    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose
        q = self.pose.orientation
        self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def control_loop(self):
        if self.path.size == 0 or self.pose is None:
            return
        if self.plot_path :
            self.real_path.append([self.pose.position.x, self.pose.position.y])
        position = np.array([self.pose.position.x, self.pose.position.y])
        # Find closest point on path
        dists = np.linalg.norm(self.path - position, axis=1)
        closest_id = np.argmin(dists)

        # Find the lookahead point
        lookahead_point = None
        for i in range(closest_id, len(self.path)):
            dist = np.linalg.norm(self.path[i] - position)
            if dist >= self.look_ahead_dist:
                lookahead_point = self.path[i]
                break

        if lookahead_point is None:
            self.get_logger().warn('No lookahead point found.')
            # Publish command
            cmd = Twist()
            cmd.linear.x = 0.0  # Break
            self.cmd_pub.publish(cmd)
            if self.plot_path :
                self.get_logger().info('End of path reached')
                plt.plot(self.path[:,0], self.path[:,1], label='target path', color='black')
                self.real_path = np.array(self.real_path)
                plt.plot(self.real_path[:,0], self.real_path[:,1], label='real path', color='red')
                plt.title('Path')
                plt.xlabel('x')
                plt.ylabel('y')
                #plt.axis('equal')
                plt.grid(True)
                plt.legend()
                plt.show(block=False)
                plt.pause(0.001)
                self.plot_path = False
            return


        path_pos = self.path[closest_id]
        dx = lookahead_point[0] - position[0]
        dy = lookahead_point[1] - position[1]
        
        alpha = np.arctan2(dy, dx) - (self.yaw - self.angle_offset)
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        # Pure Pursuit steering formula
        steer = np.arctan2(2 * self.look_ahead_dist * np.sin(alpha), self.kpp * self.speed + 1e-5)
        steer = np.clip(steer, -self.max_steering, self.max_steering)

        # Publish command
        cmd = Twist()
        cmd.linear.x = 1.0  # constant speed
        cmd.angular.z = steer
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

