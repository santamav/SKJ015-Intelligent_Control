import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

class PBVS_FeatureNode(Node):

    def __init__(self): #, camera, p, pose_g=None):
        super().__init__('pbvs_feature_node')
        self.declare_parameter('featureParams', [['1.', '1.', '-2.'], ['2.', '0.5'], ['-1.', '-1.', '2.']])
        self.publisher_ = self.create_publisher(Float64MultiArray, 'feature_data', 10)
        
        featureParams = self.get_parameter('featureParams').get_parameter_value().float_array_value
        self.camera = CentralCamera.Default(pose = SE3.Trans(featureParams[0][0], featureParams[0][1], featureParams[0][2]))
        self.P = mkgrid(featureParams[1][0], featureParams[1][1])
        self.pose_g = SE3.Trans(featureParams[2][0], featureParams[2][1], featureParams[2][2])

        # comute pose
        self.uv = self.camera.project_point(self.P, objpose=self.pose_g)
        Te_C_G = self.camera.estpose(self.P, self.uv, frame="camera") 
        Te_C_G = Te_C_G.A.tolist # convert to numpy array and then to list
        msg = Float64MultiArray() 
        msg.data = Te_C_G
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    PBVS_feature_node = PBVS_FeatureNode()
    rclpy.spin(PBVS_feature_node)
    PBVS_feature_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()