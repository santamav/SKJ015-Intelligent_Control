import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

import time

class Actuator(Node):
    def __init__(self):
        super().__init__('actuator')
        
        self.camera = CentralCamera.Default(pose=SE3.Trans(1, 1, -3) * SE3.Rz(0.6));
        self.lmbda = 0.05
        
        # Prepare publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'camera_pose', 10)
        
        # Prepare subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'v',
            self.apply_velocity,
            10
        )
        
    def apply_velocity(self, msg):
        # Reshape data to 1x6 matrix
        data = np.array(msg.data).reshape(1, 6)
        
        self.Td = SE3.Delta(data)
        self.camera.pose @= self.Td
        
        # Send new pose
        msg = Float64MultiArray()
        msg.data = self.camera.pose.A.flatten().tolist()
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    actuator = Actuator()
    
    rclpy.spin(actuator)
    
    actuator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()