import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

class FeatureExtractor(Node):
    def __init__(self):
        super().__init__('feature_extractor')
        
        self.camera = CentralCamera.Default(pose = SE3.Trans(1, 1, -2))
        self.P = mkgrid(2, 0.5)
        self.pose_g = SE3.Trans(-1, -1, 2)
        # Prepare publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'feature_data', 10)
        # Prepare subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'camera_pose',
            self.update_camera_pose,
            10
        )
        
        # First call
        self.compute_pose()
        
        
    def compute_pose(self):
        # Compute pose
        self.uv = self.camera.project_point(self.P, objpose=self.pose_g)
        # Estimate the camera pose
        Te_C_G = self.camera.estpose(self.P, self.uv, frame="camera") 
        # convert to numpy array, flatten and then to a list
        Te_C_G = Te_C_G.A.flatten().tolist()
        # Compose message
        msg = Float64MultiArray()
        msg.data = Te_C_G
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
    def update_camera_pose(self, msg):
        # Reshape data to 4x4 matrix
        data = np.array(msg.data).reshape(4,4)
        self.get_logger().info('Received: "%s"' % data)
        # Apply new pose to the camera
        self.camera.pose = SE3(data)
        # Call the compute pose function
        self.compute_pose()

def main(args = None):
    rclpy.init(args=args) # initialize the ROS2 client library
    feature_extractor = FeatureExtractor()
    rclpy.spin(feature_extractor)
    feature_extractor.destroy_node()
    rclpy.shutdown() # shutdown the ROS2 client library

if __name__ == '__main__':
    main()
