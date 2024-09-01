#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool,Empty
from std_srvs.srv._set_bool import SetBool_Request, SetBool_Response

class PropellorNode(Node):

    def __init__(self):
        super().__init__('propellor_node')
        self.propellor_publisher = self.create_publisher(Float64MultiArray, '/propellor_controller/commands', 1)
        self.timer_period = 1.0 
        self.timer = None
        self.motor_up_speed = 500.0
        self.motor_down_speed = 500.0

        # Here define services
        self.create_service(SetBool, '/start_rotors', self.start_rotors)
        self.create_service(SetBool, '/stop_rotors', self.stop_rotors)
        self.create_subscription(Float64MultiArray, '/rotor_speed', self.handle_speed_change, qos_profile=1)

    def handle_speed_change(self, msg: Float64MultiArray):
        self.get_logger().info(f"upper rotter: {msg.data[0]}, Lower Rotter: {msg.data[1]}")
        self.motor_up_speed = msg.data[0]
        self.motor_down_speed = msg.data[0]
    
    def timer_callback(self):
        self.publish_rotor_val([-self.motor_up_speed, self.motor_down_speed])

    def publish_rotor_val(self, value):
        target_vel = Float64MultiArray()
        target_vel.data = value
        self.propellor_publisher.publish(target_vel)

    def start_rotors(self, request: SetBool_Request, response: SetBool_Response):

        if self.timer:
            return response
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        return response
    
    def stop_rotors(self, request: SetBool_Request, response: SetBool_Response):
        
        if not self.timer:
            return response
        
        self.timer.cancel()
        self.timer.destroy()
        self.timer = None
        self.publish_rotor_val([0.0, 0.0])

        return response
            

def main(args=None):
    rclpy.init(args=args)

    propellor_node = PropellorNode()
    rclpy.spin(propellor_node)

    propellor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()