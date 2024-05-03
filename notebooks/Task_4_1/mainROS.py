
from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *

cameraParams = [1, 1, -2]
#camera = CentralCamera.Default(pose = SE3.Trans(1, 1, -2))
camera = CentralCamera.Default(pose = SE3.Trans(1, 1, -2))

#4 points-goal
Pparams = [2, 0.5]
#P = mkgrid(2, 0.5)

pose_gParams = [-1, -1, 2]
#pose_g=SE3.Trans(-1, -1, 2)

pose_dParams = [1] #[0, 0, 1]
#pose_d=SE3.Tz(1) #SE3(0, 0, 1)

plotvol=[-1, 2, -1, 2, -3, 2.5]

#pbvs = V_PBVS(camera=camera, P=P, pose_g=pose_g, pose_d=pose_d, plotvol=plotvol)
#pbvs.run(20) # Run the simulation for 20 steps, activaete the loop that iterate the function step 20 times.


