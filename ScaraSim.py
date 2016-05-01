import sys
sys.path.append('C:\Dropbox\Grad School\ME 481 Computer-Aided Analysis of Machine Dynamics\Project\Python Code')
import time
import numpy as np
import math
import random
import matplotlib.pyplot as plt
from scara5 import FiveBar


#Create the Five Bar model - assume it is initialized with arms at pi/2
scara = FiveBar();
scara.L = [45,100,100,100,100];
scara.th = [0, math.pi/2, math.pi/2, math.pi/2, math.pi/2];
scara.x[2]  = 45/2;
scara.y[2]  = 197.5;  
#scara.ShowPosture();

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
y = 197.5;
#for y in np.linspace(196, 96 ,2):
scara.SetEndEffectorPosition(x,y,1);

scara.SetEndEffectorPosition(x,130,4);

#time.sleep(5);
#scara.PenDown()
time.sleep(0.2);

pointCount = 0;
for t in np.linspace(-25*math.pi,25*math.pi,3000):
    pointCount += 1;
    scara.SetEndEffectorPosition(15*(math.sin(3.02*t)-math.sin(2.02*t))+22.5,15*(math.sin(4.02*t)-math.sin(6.02*t))+130,0.8);
    if(pointCount == 1):
        scara.PenDown();

##Draw 20x20 square
##while(True):
##    #for x in np.linspace(22.5,42.5,2):
##    scara.SetEndEffectorPosition(42.5,130);
##
##    #for y in np.linspace(130, 150 ,2):
##    scara.SetEndEffectorPosition(42.5,150);
##
##    #for x in np.linspace(42.5,22.5,2):
##    scara.SetEndEffectorPosition(22.5,150);
##
##    #for y in np.linspace(150, 130 ,2):
##    scara.SetEndEffectorPosition(22.5,130);
    

scara.PenUp();


#plt.show()
    
 #while(True):
  #   scara.SetEndEffectorPosition(random.uniform(-2,4.5),random.uniform(-4,4));
    #  scara.ShowPosture();    
