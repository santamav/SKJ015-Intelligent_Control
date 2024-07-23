# imports

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

# Main clas for the PBVS simulation
class VSM_PBVS(VisualServo):
    def __init__(self, camera, eterm=1e-3, lmbda=0.5, **kwargs):
        super().__init__(camera, type="point", title="PBVS simulation", **kwargs)
        
        self.actuator = PBVS_Actuator(camera)
        self.featurextractor = PBVS_FeatureExtractor(camera, self.P, self.pose_g)
        self.controller = PBVS_Controller(eterm, lmbda)
        
        if self.pose_d is None:
            self.pose_d = SE3(0, 0, 1)
        
    def step(self, t):
        status = 0
        # Call modules
        Te_C_G = self.featurextractor.compute_pose()
        T_delta = self.controller.compute_TDelta(Te_C_G, self.pose_d)
        self.actuator.apply_velocity(T_delta)
        
        # Update history
        hist = self._history()
        hist.p = self.featurextractor.uv
        hist.Te_C_G = Te_C_G
        hist.T_delta = T_delta
        hist.vel = T_delta.delta()
        hist.pose = self.camera.pose
        
        self.history.append(hist)
        
        # Check if the error is less than the threshold
        if(np.linalg.norm(T_delta) < self.controller.eterm):
            status = 1
        
        return status
    
    def plot_p(self):
        """
        Plot feature trajectory from simulation

        Show image feature points versus time.

        :seealso: :meth:`plot_vel` :meth:`self.plot_pose` :meth:`plot_jcond` :meth:`plot_z` :meth:`plot_error`
        """

        if len(self.history) == 0:
            return

        if self.type != "point":
            print("Can only plot image plane trajectories for point-based IBVS")
            return

        # result is a vector with row per time step, each row is u1, v1, u2, v2 ...
        for i in range(self.npoints):
            u = [h.p[0, i] for h in self.history]  # get data for i'th point
            v = [h.p[1, i] for h in self.history]
            plt.plot(u, v, "b")

        # mark the initial target shape
        smbase.plot_polygon(
            self.history[0].p,
            "o--",
            close=True,
            markeredgecolor="k",
            markerfacecolor="w",
            label="initial",
        )

        # mark the goal target shape
        if isinstance(self, IBVS):
            smbase.plot_polygon(
                self.p_star,
                "k*:",
                close=True,
                markeredgecolor="k",
                markerfacecolor="k",
                label="goal",
            )

        if isinstance(self, VSM_PBVS):
            p = self.camera.project_point(self.P, pose=self.pose_d.inv())
            smbase.plot_polygon(
                p,
                "k*:",
                close=True,
                markeredgecolor="k",
                markerfacecolor="k",
                label="goal",
            )

# PBVS Feature Extractor module
class PBVS_FeatureExtractor:
    def __init__(self, camera, P, pose_g=None):
        self.camera = camera
        self.P = P
        self.pose_g = pose_g

    def compute_pose(self):
        self.uv = self.camera.project_point(self.P, objpose=self.pose_g)
        Te_C_G = self.camera.estpose(self.P, self.uv, frame="camera")
        return Te_C_G
    
# PBVS Controller module  
class PBVS_Controller:
    def __init__(self, eterm=0, lmbda=0.05):
        self.lmbda = lmbda
        self.eterm = eterm
        self.pose_d = None

    def compute_TDelta(self, Te_C_G, pose_d):
        T_delta = Te_C_G * pose_d.inv()
        return T_delta  
    
# PBVS Actuator module
class PBVS_Actuator:
    def __init__(self, camera, lmbda=0.05):
        self.camera = camera
        self.lmbda = lmbda

    def apply_velocity(self, T_delta):
        Td = T_delta.interp1(self.lmbda)  
        self.camera.pose @= Td