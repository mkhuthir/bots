#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray    


class Publisher(Node):

    def __init__(self):
        super().__init__('py_node')
        self.pub = self.create_publisher(Float64MultiArray, '/py_topic/commands', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 30.0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data=[self.i]
        self.pub.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)
        self.i *= -1


def main(args=None):
    rclpy.init(args=args)

    my_node = Publisher()

    rclpy.spin(my_node)
    
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
