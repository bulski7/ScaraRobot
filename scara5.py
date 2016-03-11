from math import atan, sqrt, asin, sin, cos
import numpy as np
import matplotlib.pyplot as plt

class FiveBar:
#5 Bar robot object

    def __init__(self):
        self.L = [1,1,1,1,1];   #link length vector
        self.th = [1,1,1,1,1];  #theta vector
        # self.Ex = 0;            #end effector x
        # self.Ey = 0;            #end effector y

    #Set the link angles to achieve a given X,Y of the end effector
    #Using geometric equations
    def SetEndEffectorPosition(self, EndEffectorX, EndEffectorY):
        xc = EndEffectorX;
        yc = EndEffectorY;

        #calculate theta 1
        A = xc;
        B = yc;
        C = (self.L[1]**2 - self.L[2]**2 + xc**2 + yc**2)/(2*self.L[1]);
        self.th[1] = 2*atan((-B-sqrt(A**2 + B**2 - C**2))/(-A-C));
        
        #Calculate theta 4
        A = xc - self.L[0];
        B = yc;
        C = (self.L[4]**2 + self.L[0]**2 - self.L[3]**2 - 2*xc*self.L[0] + xc**2 + yc**2)/(2*self.L[4]);
        self.th[4] = 2*atan((-B+sqrt(A**2 + B**2 - C**2))/(-A-C));

        #Calculate theta 3
        A = 2*self.L[3]*self.L[4]*sin(self.th[4])-2*self.L[1]*self.L[3]*cos(self.th[1]);
        B = 2*self.L[3]*self.L[0]-2*self.L[1]*self.L[3]*cos(self.th[1]) + 2*self.L[3]*self.L[4]*cos(self.th[4]);
        C =(self.L[1]**2 - self.L[2]**2 + self.L[3]**2 + self.L[4]**2 + self.L[0]**2 - self.L[1]*self.L[4]*sin(self.th[1])*sin(self.th[4]) - 2*self.L[1]*self.L[0]*cos(self.th[1])
            + 2*self.L[4]*self.L[0]*cos(self.th[4]) - 2*self.L[1]*self.L[4]*cos(self.th[1])*cos(self.th[4])  );
            
        #calculate theta 2
        self.th[2] = asin((self.L[3]*sin(self.th[3]) + self.L[4]*sin(self.th[4]) - self.L[1]*sin(self.th[1]))/self.L[2]);
    
    def ShowPosture(self):
        x = [0]*5;
        y = [0]*5;
        
        for i in range(1,5):
            x[i] = x[i-1] + self.L[i-1]*cos(self.th[i-1]);
            y[i] = y[i-1] + self.L[i-1]*sin(self.th[i-1]);
        
        plt.plot(x,y,'r--');
        plt.show();