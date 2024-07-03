import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

import time


class FeatureExtractor(Node):
    def __init__(self):
        super().__init__('feature_extractor')
        
        # IBVS feature extractor properties
        self.camera = CentralCamera.Default(pose=SE3.Trans(1, 1, -3) * SE3.Rz(0.6));
        self.P = mkgrid(2, side=0.5, pose=SE3.Tz(2));
        self.pose_g = SE3.Trans(-1, -1, 2) # Goal position
        # Prepare publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'feature_data', 10)
        # Prepare subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'camera_pose',
            self.update_camera_pose,
            10
        )
        
        self.uv_history = []
        
        # Wait for the rest of the nodes to start listening
        start_time = time.time()
        while time.time() - start_time < 1:
            pass
        
        # First call
        self.compute_pose()
        
        
    def compute_pose(self):
        # Compute pose
        uv = self.camera.project_point(self.P)
        # Store uv in file
        self.uv_history.append(uv)
        np.save('/home/sjk015/Documents/SKJ015-Intelligent_Control/Task_4_1/outputs/IBVS/uv.npy', np.array(self.uv_history))
        # Compose message
        msg = Float64MultiArray()
        msg.data = uv.flatten().tolist()
        # Publish message
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing uv: "%s"' % msg.data)
        
    def update_camera_pose(self, msg):
        # Recompose data to 4x4 matrix
        data = np.array(msg.data).reshape(4, 4)
        # Update camera pose
        self.camera.pose = SE3(data)
        # compute new pose
        self.compute_pose()
        
        
def main(args=None):
    rclpy.init(args=args)

    feature_extractor = FeatureExtractor()

    rclpy.spin(feature_extractor)

    feature_extractor.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()