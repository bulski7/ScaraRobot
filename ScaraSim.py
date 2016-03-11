import sys
sys.path.append('C:\Dropbox\Grad School\ME 481 Computer-Aided Analysis of Machine Dynamics\Project\Python Code')
import time
import numpy as np
from scara5 import FiveBar
scara = FiveBar();
scara.L = [2,2,2,2,2.5];
scara.th = [1.6,.7,-0.7,-1.6,np.pi]
scara.ShowPosture()
scara.SetEndEffectorPosition(1.5,1.5)
scara.ShowPosture()