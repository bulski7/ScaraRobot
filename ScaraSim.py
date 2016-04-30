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

##t = np.linspace(0,4*math.pi,1000);
##x = np.zeros([1000,1]);
##y = np.zeros([1000,1]);
##
##x = 22.5 + 20*math.sin(2*t);
##y = 50   + 20*math.cos(4*t);

##for t in t:
    #scara.SetEndEffectorPosition(x,y);
x = 22.5;
for y in np.linspace(196, 130 ,5):
    scara.SetEndEffectorPosition(x,y);

time.sleep(5);

for x in np.linspace(22.5,42.5,2):
    scara.SetEndEffectorPosition(x,y);

for y in np.linspace(130, 150 ,2):
    scara.SetEndEffectorPosition(x,y);

for x in np.linspace(42.5,22.5,2):
    scara.SetEndEffectorPosition(x,y);

for y in np.linspace(150, 130 ,2):
    scara.SetEndEffectorPosition(x,y);



#plt.show()
    
 #while(True):
  #   scara.SetEndEffectorPosition(random.uniform(-2,4.5),random.uniform(-4,4));
    #  scara.ShowPosture();    
