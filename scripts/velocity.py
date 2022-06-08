#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray    


class cartPub(Node):
    def __init__(self):
        super().__init__('cartVelocityNode')
        self.pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

def main(args=None):
    rclpy.init(args=args)

    msg = Float64MultiArray()
    msg.data = [1.0]

    node = cartPub()
        
    node.get_logger().info("Node Created !")

    node.create_rate(1) # Rate in Hz

    try:
        while rclpy.ok():
            
            node.pub.publish(msg)
            node.get_logger().info('Published: "%s"' % msg.data)
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print('Keyboard interrupted !')
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
