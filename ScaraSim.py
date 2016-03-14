import sys
sys.path.append('C:\Dropbox\Grad School\ME 481 Computer-Aided Analysis of Machine Dynamics\Project\Python Code')
import time
import numpy as np
import math
from scara5 import FiveBar

#Create the Five Bar model
scara = FiveBar();
scara.L = [2.5,2,2,2,2];

#Move the end effector at CV
for t in np.linspace(0,2*math.pi,100):
    scara.SetEndEffectorPosition(1.5 + math.cos(t),2 + math.sin(t));
    scara.ShowPosture();