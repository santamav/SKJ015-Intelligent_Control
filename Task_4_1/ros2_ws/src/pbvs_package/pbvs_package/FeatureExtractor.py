import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy as np

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

class PBVS_FeatureNode(Node):

    def __init__(self): #, camera, p, pose_g=None):
        super().__init__('pbvs_feature_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'feature_data', 10)
        
        self.camera = CentralCamera.Default(pose = SE3.Trans(1, 1, -2))
        self.P = mkgrid(2, 0.5)
        self.pose_g = SE3.Trans(-1, -1, 2)
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'pose',
            self.update_camera_pose,
            10
        )
        
        self.update_camera_pose(self.camera.pose.A.flatten().tolist())
        
    def update_camera_pose(self, pose):
        # Reshape data to 4x4 matrix
        data = np.array(pose).reshape(4,4)
        #self.get_logger().info(f'Publishing: {data}')

        # Update camera pose
        self.camera.pose = SE3(data)
        
        # Compute pose
        self.uv = self.camera.project_point(self.P, objpose=self.pose_g)
        # Estimate the camera pose
        Te_C_G = self.camera.estpose(self.P, self.uv, frame="camera") 
        # convert to numpy array, flatten and then to a list
        Te_C_G = Te_C_G.A.flatten().tolist()
        # Prepare and send the message
        msg = Float64MultiArray() 
        msg.data = Te_C_G
        self.get_logger().info(f'Publishing: {msg.data}')
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    PBVS_feature_node = PBVS_FeatureNode()
    rclpy.spin(PBVS_feature_node)
    PBVS_feature_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()