import rclpy
from rclpy.node import Node
import VSM_PBVS as pbvs
from std_msgs.msg import Float64
import numpy as np

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

class PBVS_FeatureNode(Node):

    def __init__(self, camera, p, pose_g=None):
        super().__init__('pbvs_feature_node')
        self.publisher_ = self.create_publisher(Float64, 'feature_data', 10)
        
        self.camera = camera
        self.P = p
        self.pose_g = pose_g


    def compute_pose(self):
        self.uv = self.camera.project_point(self.P, objpose=self.pose_g)
        Te_C_G = self.camera.estpose(self.P, self.uv, frame="camera")
        
        msg = Float64()
        msg.data = Te_C_G
        self.publisher_.publish(msg)
        
class PBVS_ControllerNode(Node):
    def __init__(self, eterm=0, lmbda=0.05):
        super().__init__('pbvs_controller_node')
        self.subscription = self.create_subscription(
            Float64,
            'feature_data',
            self.compute_TDelta,
            10
        )
        self.publisher_ = self.create_publisher(Float64, 'controller_data', 10)
        
        self.lmbda = lmbda
        self.eterm = eterm
        self.pose_d = SE3(0, 0, 1)
        
    def compute_TDelta(self, Te_C_G):
        T_delta = Te_C_G * self.pose_d.inv()
        
        msg = Float64()
        msg.data = T_delta
        self.publisher_.publish(msg)
        
        
class PBVS_ActuatorNode(Node):
    def __init__(self, camera, lmbda=0.05):
        super().__init__('pbvs_actuator_node')
        self.subscription = self.create_subscription(
            Float64,
            'controller_data',
            self.apply_velocity,
            10
        )
        
        self.camera = camera
        self.lmbda = lmbda
        
    def apply_velocity(self, T_delta):
        Td = T_delta.interp1(self.lmbda)  
        self.camera.pose @= Td