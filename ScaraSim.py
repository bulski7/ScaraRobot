import sys
sys.path.append('C:\Dropbox\Grad School\ME 481 Computer-Aided Analysis of Machine Dynamics\Project\Python Code')
import time
import numpy as np
import math
from scara5 import FiveBar

#Create the Five Bar model
scara = FiveBar();
scara.L = [2.5,2,2,2,2];
scara.th = [0,0.5,1.8,1.9,1.8];
scara.ShowPosture();
print(scara.DriveArmsIntersect());

#Move the end effector at CV
for t in np.linspace(0,2*math.pi,50):
    scara.SetEndEffectorPosition(2,0 + .5*t);
    scara.ShowPosture();
    time.sleep(.2);
    
# for t in np.linspace(math.pi - 1.25 ,-math.pi,50):
#     scara.SetEndEffectorPosition(2.5 + 2*math.cos(t),0 + 2*math.sin(t));
#     scara.ShowPosture();
    