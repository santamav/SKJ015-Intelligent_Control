import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.lmbda = 0.05
        self.eterm = 0.001
        self.pose_d = SE3(0, 0, 1)
        
        # Prepare subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'feature_data',
            self.compute_delta,
            10
        )
        
        # Prepare the publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'delta', 10)
        
    def compute_delta(self, msg):
        # Reshape data to 4x4 matrix
        data = np.array(msg.data).reshape(4,4)
        #self.get_logger().info('Received: "%s"' % data)
        
        # Compute the new delta
        Te_C_G = SE3(data)
        T_delta = Te_C_G * self.pose_d.inv()
        T_delta = T_delta.A.flatten().tolist()
        
                # Check if the error is less than the threshold
        if(np.linalg.norm(T_delta) >= self.eterm):
            # Compose message
            msg = Float64MultiArray()
            msg.data = T_delta
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.publisher_.publish(msg)

def main(args = None):
    rclpy.init(args=args) # initialize the ROS2 client library
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown() # shutdown the ROS2 client library

if __name__ == '__main__':
    main()