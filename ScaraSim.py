import sys
sys.path.append('C:\Dropbox\Grad School\ME 481 Computer-Aided Analysis of Machine Dynamics\Project\Python Code')
import time
import numpy as np
import math
import random
from scara5 import FiveBar

#Create the Five Bar model
scara = FiveBar();
scara.L = [2.5,2,2,2,2];

#Forward Kinematics Test
for th1 in np.linspace(0,2*math.pi,50):
    for th4 in np.linspace(0,2*math.pi,50):
        scara.SetDriveArmPositions(th1,th4);
        if(not scara.DriveArmsIntersect()):
            scara.ShowPosture();

#Inverse Kinematics Test
#Move the end effector at CV
for x in np.linspace(-2,4.5,50):
    for y in np.linspace(-4,4,50):
        scara.SetEndEffectorPosition(x,y);
        scara.ShowPosture();
    
# while(True):
#     scara.SetEndEffectorPosition(random.uniform(-2,4.5),random.uniform(-4,4));
#     scara.ShowPosture();    