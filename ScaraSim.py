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
for t in np.linspace(0,1.25,500):
    scara.SetEndEffectorPosition(0 + 4*math.cos(t),0 + 4*math.sin(t));
    scara.ShowPosture();
    
for t in np.linspace(math.pi - 1.25 ,-math.pi,500):
    scara.SetEndEffectorPosition(2.5 + 4*math.cos(t),0 + 4*math.sin(t));
    scara.ShowPosture();
    