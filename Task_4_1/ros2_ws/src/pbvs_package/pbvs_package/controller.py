import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy as np

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *
import numpy as np
        
class PBVS_ControllerNode(Node):
    def __init__(self):
        super().__init__('pbvs_controller_node')
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'feature_data',
            self.compute_TDelta,
            10
        )
        self.publisher_ = self.create_publisher(Float64, 'controller_data', 10)
        
        self.lmbda = 0
        self.eterm = 0.05
        self.pose_d = SE3.Tz(1)
        
        self.subscription  # prevent unused variable warning
        
    def compute_TDelta(self, msg ):
        # Reshape data to 4x4 matrix
        self.get_logger().info(f'Publishing: {msg.data}')
        data = np.array(msg.data).reshape(4,4)
        
        Te_C_G = SE3(data) # convert to numpy array and then to a spatial math pose
        T_delta = Te_C_G * self.pose_d.inv()
        T_delta = T_delta.A.flatten().tolist # convert to numpy array and then to list
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