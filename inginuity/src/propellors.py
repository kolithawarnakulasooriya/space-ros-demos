#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PropellorNode(Node):

    def __init__(self):
        super().__init__('propellor_node')
        self.propellor_publisher = self.create_publisher(Float64MultiArray, '/propellor_controller/commands', 1)
        timer_period = 0.5 
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.v = 10.0
    
    def timer_callback(self):
        target_vel = Float64MultiArray()
        target_vel.data = [self.v, -self.v]
        self.propellor_publisher.publish(target_vel)
            

def main(args=None):
    rclpy.init(args=args)

    propellor_node = PropellorNode()
    rclpy.spin(propellor_node)

    propellor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()