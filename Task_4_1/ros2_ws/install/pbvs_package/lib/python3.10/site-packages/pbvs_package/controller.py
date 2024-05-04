import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy as np

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

        
class PBVS_ControllerNode(Node):
    def __init__(self):
        super().__init__('pbvs_controller_node')
        self.declare_parameter('controllerParams', [0., 0., 1.])
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'feature_data',
            self.compute_TDelta,
            10
        )
        self.publisher_ = self.create_publisher(Float64, 'controller_data', 10)
        
        controllerParams = self.get_parameter('controllerParams').get_parameter_value().float_array_value
        self.lmbda = 0
        self.eterm = 0.05
        self.pose_d = SE3(controllerParams[0], controllerParams[1], controllerParams[2])
        
        self.subscription  # prevent unused variable warning
        
    def compute_TDelta(self, Te_C_G ):
        Te_C_G = SE3(np.array(Te_C_G)) # convert to numpy array and then to a spatial math pose
        T_delta = Te_C_G * self.pose_d.inv()
        T_delta = T_delta.A.tolist # convert to numpy array and then to list
        msg = Float64MultiArray() 
        msg.data = T_delta
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    PBVS_controller_node = PBVS_ControllerNode()
    rclpy.spin(PBVS_controller_node)
    PBVS_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()