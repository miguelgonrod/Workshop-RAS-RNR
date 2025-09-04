# First we need to indicade the compiler for the linux system:

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Secondly we import the ROS2 library for python compilation:
import rclpy

# For any kind of publisher/suscriber node code, it's important to import the specifically messages objects:
from geometry_msgs.msg import TwistStamped

# You could use functions or classes to write ROS2 nodes.
def main(args=None):
    
    # (First block of code):
    rclpy.init(args=args)
    
    
    # (Second block of code):
    node = rclpy.create_node('turtlebot3_minimal_publisher')
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    
    # (Third block of code):
    msg = TwistStamped()
    
    def timer_callback():
        
        # (Fourth block of code):
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # (Fiveth block of code):
        msg.twist.linear.x = 0.4
        
        # (Sixth block of code):
        publisher.publish(msg)
        node.get_logger().info(
            f'Publicando TwistStamped -> linear.x={msg.twist.linear.x}'
        )
        
    # (Seventh block of code):
    node.create_timer(0.5, timer_callback)
    
    
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