from math import atan, sqrt, asin, sin, cos, acos
import math
from KinHelp import GetAngle, Distance

import numpy as np
import matplotlib.pyplot as plt

class FiveBar:
#5 Bar robot object

    def __init__(self):
        self.L = [1,1,1,1,1];   #link length vector
        self.th = [0,0,0,0,0];  #theta vector
   
    #Set the link angles to achieve a given X,Y of the end effector
    #Using geometric equations
    def SetEndEffectorPosition(self, EndEffectorX, EndEffectorY):
        xc = EndEffectorX; 
        yc = EndEffectorY;
        
        F = sqrt(xc**2 + yc**2);
        G = sqrt((self.L[0]-xc)**2 + yc**2);
        
        self.th[1] = GetAngle(self.L[0],F,G) + GetAngle(self.L[1],F,self.L[2]);
        self.th[4] = math.pi - (GetAngle(self.L[0],G,F) + GetAngle(self.L[4],G,self.L[3]));
        
        self.th[2] = (GetAngle(self.L[1],self.L[2],F)-(math.pi-self.th[1]));
        self.th[3] = math.pi - GetAngle(self.L[3],self.L[4],G)-self.th[4];
    
    def ShowPosture(self):
        plt.cla();
        x = [0]*5;
        y = [0]*5;
        for i in [1,2]:
            x[i] = x[i-1] + self.L[i]*cos(self.th[i]);
            y[i] = y[i-1] + self.L[i]*sin(self.th[i]);
            
            
        last_i = 0;            
        for i in [4,3]:
            x[i] = x[last_i] + self.L[last_i]*cos(self.th[last_i]);
            y[i] = y[last_i] + self.L[last_i]*sin(self.th[last_i]);
            last_i = i;
        
        print(Distance(x[3],y[3],x[2],y[2]));
        plt.plot(x,y,'--ro');
        plt.axes().set_aspect('equal')
        plt.xlim([-2, 4.5]);
        plt.ylim([-4, 4]);
        plt.draw();
        
    