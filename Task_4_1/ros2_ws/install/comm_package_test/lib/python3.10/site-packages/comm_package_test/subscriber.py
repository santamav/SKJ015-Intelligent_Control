import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy as np

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'pose',
            self.listener_callback,
            10)
        self.subscription
        
    def listener_callback(self, msg):
        data = np.array(msg.data).reshape(4,4)
        self.get_logger().info(f'Received: {data}')
        
        
def main(args=None):
    rclpy.init(args=args)
    subscriber = SubscriberNode()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()