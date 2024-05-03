import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

        
class PBVS_ActuatorNode(Node):
    def __init__(self):
        super().__init__('pbvs_actuator_node')
        self.declare_parameter('actuatorParams', [1, 1, -2])
        self.subscription = self.create_subscription(
            Float64,
            'controller_data',
            self.apply_velocity,
            10
        )
        
        actuatorParams = self.get_parameter('actuatorParams').get_parameter_value().float_array_value
        self.camera = CentralCamera.Default(pose = SE3.Trans(actuatorParams[0][0], actuatorParams[0][1], actuatorParams[0][2]))
        self.lmbda = 0.05
        
        self.subscription  # prevent unused variable warning
        
    def apply_velocity(self, T_delta): # T_delta is the received message no es un float
        Td = T_delta.interp1(self.lmbda)  
        self.camera.pose @= Td
        

def main(args=None):
    rclpy.init(args=args)
    PBVS_actuator_node = PBVS_actuatorNode()
    rclpy.spin(PBVS_actuator_node)
    PBVS_actuator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()