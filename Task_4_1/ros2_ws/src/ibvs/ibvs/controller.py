import rclpy
from rclpy.node import Node

import rclpy.signals
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

import sys

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.lmbda = 0.05
        self.eterm = 0
        self.depth = 1
        self.camera = CentralCamera.Default(pose=SE3.Trans(1, 1, -3) * SE3.Rz(0.6));
        self.p_star = 200 * np.array([[-1, -1, 1, 1], [-1, 1, 1, -1]]) + np.c_[self.camera.pp]
        
        # Prepare subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'feature_data',
            self.compute_verlocity,
            10
        )
        
        self.camera_sub = self.create_subscription(
            Float64MultiArray,
            'camera_pose',
            self.update_camera,
            10
        )
        
        # Prepare the publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'v', 10)
        
    def compute_verlocity(self, msg):
        # Reshape uv data to 2x2 matrix
        uv = np.array(msg.data).reshape(2, 4)
        self.get_logger().info('Received uv: "%s"' % uv)
        # Compute velocity
        self.e = uv - self.p_star
        self.e = self.e.flatten('F')
        self.J = self.camera.visjac_p(uv, self.depth)
        
        try:
            v = -self.lmbda * np.linalg.pinv(self.J) @ self.e
        except np.linalg.LinAlgError:
            sys.exit()
        
        # Compose message
        msg = Float64MultiArray()
        msg.data = v.flatten().tolist()
        # Publish message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing v: "%s"' % msg.data)
        
    def update_camera(self, msg):
        # Recompose data to 4x4 matrix
        data = np.array(msg.data).reshape(4, 4)
        # Update camera pose
        self.camera.pose = SE3(data)

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()