import sys
sys.path.append('C:\Dropbox\Grad School\ME 481 Computer-Aided Analysis of Machine Dynamics\Project\Python Code')
import time
import numpy as np
import math
import random
import matplotlib.pyplot as plt
from scara5 import FiveBar

#Create the Five Bar model
scara = FiveBar();
scara.L = [45,100,100,100,100];

###Forward Kinematics Test
##for th1 in np.linspace(0,2*math.pi,50):
##    for th4 in np.linspace(0,2*math.pi,50):
##        if(not scara.SetDriveArmPositions(th1,th4) and not scara.DriveArmsIntersect()):
##            scara.ShowPosture();
##            
##plt.axes().set_aspect('equal')
###plt.xlim([-2, 4.5]);
###plt.ylim([-4, 4]);
##plt.show()

#Inverse Kinematics Test
#Move the end effector at CV
 #for x in np.linspace(-2,4.5,50):
for y in np.linspace(197,100,50):
    scara.SetEndEffectorPosition(22.5,y);

for x in np.linspace(22.5,42.5,50):
    scara.SetEndEffectorPosition(x,y);

for y in np.linspace(100, 150 ,50):
    scara.SetEndEffectorPosition(x,y);

for x in np.linspace(42.5,22.5,50):
    scara.SetEndEffectorPosition(x,y);



plt.show()
    
 #while(True):
  #   scara.SetEndEffectorPosition(random.uniform(-2,4.5),random.uniform(-4,4));
    #  scara.ShowPosture();    
