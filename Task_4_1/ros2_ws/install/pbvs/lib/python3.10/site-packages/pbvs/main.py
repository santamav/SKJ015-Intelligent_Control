import rclpy
from rclpy.node import Node

# Import the the modules from the pbvs module
from feature_extractor import FeatureExtractor

def main(args = None):
    rclpy.init(args=args) # initialize the ROS2 client library
    feature_extractor = FeatureExtractor()
    rclpy.spin(feature_extractor)
    feature_extractor.destroy_node()
    rclpy.shutdown() # shutdown the ROS2 client library

if __name__ == '__main__':
    main()