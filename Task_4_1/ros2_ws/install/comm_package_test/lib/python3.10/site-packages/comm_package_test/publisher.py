import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy as np

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        # This class should read a Float64 array from the parameters and publish it
        self.pose = np.identity(4)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'pose', 10)
        timer_period = 0.5 # Would this be like the step function?
        self.timer = self.create_timer(timer_period, self.step)
        self.step_count = 0
        
    def step(self):
        msg = Float64MultiArray()
        msg.data = self.pose.flatten().tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.step_count += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = PublisherNode()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()