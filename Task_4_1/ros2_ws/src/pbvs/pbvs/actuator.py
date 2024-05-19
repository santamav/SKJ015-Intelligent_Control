import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

class Actuator(Node):
    def __init__(self):
        super().__init__('actuator')
        
        self.camera = CentralCamera.Default(pose = SE3.Trans(1, 1, -2))
        self.lmbda = 0.05
        
        # Prepare publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'camera_pose', 10)
        
        # Prepare subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'delta',
            self.apply_velocity,
            10
        )
        
    def apply_velocity(self, msg):
        # Reshape data to 4x4 matrix
        data = np.array(msg.data).reshape(4,4)
        self.get_logger().info('Received: "%s"' % data)
        
        # Compute the new delta
        T_delta = SE3(data)
        Td = T_delta.interp1(self.lmbda)
        self.camera.pose @= Td
        self.get_logger().info('New camera pose: "%s"' % self.camera.pose.A)
        
        # Send new pose
        msg = Float64MultiArray()
        msg.data = self.camera.pose.A.flatten().tolist()
        self.publisher_.publish(msg)
        
        
def main(args = None):
    rclpy.init(args=args) # initialize the ROS2 client library
    actuator = Actuator()
    rclpy.spin(actuator)
    actuator.destroy_node()
    rclpy.shutdown() # shutdown the ROS2 client library
    
if __name__ == '__main__':
    main()