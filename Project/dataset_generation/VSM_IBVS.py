import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import math
from math import pi
np.set_printoptions(linewidth=120, formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

np.random.seed(0)

from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

# Main class for the IBVS simulation
class VSM_IBVS(VisualServo):
    def __init__(
        self,
        camera,
        eterm = 0.005,
        lmbda = 0.05,
        depth = None,
        depthtest = False,
        vmax = None,
        smoothstart = None,
        **kwargs
    ):
        super().__init__(camera, type="point", title="IBVS simulation", **kwargs)

        self.actuator = IBVS_Actuator(camera, lmbda)
        self.featureextractor = IBVS_FeatureExtractor(camera, self.P, self.p_star)
        self.controller = IBVS_Controller(camera, eterm, lmbda)

        self.lmbda = lmbda
        self.eterm = eterm
        self.depth = depth
        self.depthtest = depthtest
        self.vmax = vmax
        self.smoothstart = smoothstart
        
        # Init variables history
        self.uv_hist = []
        self.p_start_hist = [] 
        self.v_hist = []
        
    def init(self):
        """
        Initialize IBVS simulation.

        Implicitly called by ``run`` to initialize state variables.

        :seealso: :meth:`run` :meth:`VisualServo.init`
        """
        # initialize the vservo variables
        super().init()

        self.vel_prev = None
        self.uv_prev = None
        self.e0 = None
        
    def step(self, t):
        status = 0
        uv = self.featureextractor.compute_pose()
        hist = self._history()
        
        v = self.controller.compute_velocity(uv, self.featureextractor.p_star)
        if v is None:
            return -1

        if self.vmax is not None:
            if np.linalg.norm(v) > self.vmax:
                v = smbase.unitvec(v) * self.vmax
                
        self.actuator.apply_velocity(v)
        
        # Update history
        hist.p = uv
        vel = self.actuator.Td.delta()
        hist.vel = vel
        hist.e = self.controller.e
        hist.enorm = np.linalg.norm(self.controller.e)
        hist.jcond = np.linalg.cond(self.controller.J)
        hist.pose = self.camera.pose
        
        self.uv_hist.append(uv)
        self.p_start_hist.append(self.featureextractor.p_star)
        self.v_hist.append(vel)

        self.history.append(hist)


        return status
    

# IBVS Feature Extractor module
class IBVS_FeatureExtractor:
    def __init__(self, camera, P, p_star):
        self.camera = camera
        self.P = P
        self.p_star = p_star

    def compute_pose(self):
        uv = self.camera.project_point(self.P)
        return uv
    

# IBVS Controller module
class IBVS_Controller:
    def __init__(self, camera, eterm=0, lmbda=0.05, depth=1):
        self.lmbda = lmbda
        self.eterm = eterm
        self.depth = depth
        self.camera = camera
        
    def compute_velocity(self, uv, p_star):
        self.e = uv - p_star
        self.e = self.e.flatten('F')
        self.J = self.camera.visjac_p(uv, self.depth)
        
        try:
            v = -self.lmbda * np.linalg.pinv(self.J) @ self.e
        except np.linalg.LinAlgError:
            return None
        
        return v
    
    
# IBVS Actuator module
class IBVS_Actuator:
    def __init__(self, camera, lmbda=0.05):
        self.camera = camera
        self.lmbda = lmbda

    def apply_velocity(self, velocity):
        self.Td = SE3.Delta(velocity)
        self.camera.pose @= self.Td