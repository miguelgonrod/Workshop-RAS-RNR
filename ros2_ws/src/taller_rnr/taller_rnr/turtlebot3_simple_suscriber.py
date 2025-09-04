# First we need to indicade the compiler for the linux system:

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Secondly we import the ROS2 library for python compilation:
import rclpy

# For any kind of publisher/suscriber node code, it's important to import the specifically messages objects:
from nav_msgs.msg import Odometry

# You could use functions or classes to write ROS2 nodes.
def main(args=None):
    
    # (First block of code)
    rclpy.init(args=args)

    # (Second block of code)
    node = rclpy.create_node('turtlebot3_minimal_subscriber')

    # (Third block of code):
    def listener_callback(msg: Odometry):
        
        # (Fourth block of code):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        node.get_logger().info(
            f'Recibido Odom -> '
            f'pos(x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}), '
            f'ori(x={ori.x:.2f}, y={ori.y:.2f}, z={ori.z:.2f}, w={ori.w:.2f}), '
            f'linear(x={lin.x:.2f}, y={lin.y:.2f}, z={lin.z:.2f}), '
            f'angular(x={ang.x:.2f}, y={ang.y:.2f}, z={ang.z:.2f})'
        )

    # (Fiveth block of code):
    subscription = node.create_subscription(
        Odometry,
        '/odom',
        listener_callback,
        10
    )

    # (Sixth block of code):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
