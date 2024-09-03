#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool,Empty
from std_srvs.srv._set_bool import SetBool_Request, SetBool_Response
from geometry_msgs.msg import Twist
from collections import deque

class PropellorNode(Node):

    def __init__(self):
        super().__init__('propellor_node')
        self.propellor_publisher = self.create_publisher(Float64MultiArray, '/propellor_controller/commands', 1)
        self.elev_publisher = self.create_publisher(Float64MultiArray, '/elevation_controller/commands', 1)
        self.timer_period = 0.5 
        self.timer = None
        self.motor_up_speed = 150.0
        self.motor_down_speed = 250.0 + 100.0
        self.twist_queue = deque()

        # Here define services
        self.create_service(SetBool, '/start_rotors', self.start_rotors)
        self.create_service(SetBool, '/stop_rotors', self.stop_rotors)
        self.sub_cmd_vel = self.create_subscription(msg_type=Twist, topic='/cmd_vel', callback=self.handle_cmd_vel, qos_profile=1)
        self.sub_cmd_vel

        self.create_timer(self.timer_period, self.timer_cmd_Vel)

    def handle_cmd_vel(self, msg: Twist):
        self.twist_queue.append(msg)
            
    def timer_cmd_Vel(self):
        if len(self.twist_queue) <= 0:
            return
        msg: Twist = self.twist_queue.pop()
        if(msg.linear.z > 0):
            target_vel = Float64MultiArray()
            target_vel.data = [msg.linear.z]
            self.elev_publisher.publish(target_vel)
    
    def timer_callback(self):
        self.publish_rotor_val([self.motor_up_speed, -self.motor_down_speed])

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