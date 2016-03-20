from math import atan, sqrt, asin, sin, cos, acos, tan
import math
from KinHelp import GetAngle, Distance, sign, GetAngleByPoints

import numpy as np
import matplotlib.pyplot as plt

class FiveBar:
#5 Bar robot object

    def __init__(self):
        self.L = [1,1,1,1,1];   #link length vector
        self.th = [0,0,0,0,0];  #theta vector
        self.x = [0,0,0,0,0];
        self.y = [0,0,0,0,0];
   
    #Set the link angles to achieve a given X,Y of the end effector
    #Using geometric equations
    def SetEndEffectorPosition(self, EndEffectorX, EndEffectorY):
        xc = EndEffectorX; 
        yc = EndEffectorY;
        
        try:
            F = sqrt(xc**2 + yc**2);
            G = sqrt((self.L[0]-xc)**2 + yc**2);
            
            thF = math.atan2(yc,xc);
            th1F = GetAngle(self.L[1],F,self.L[2]);
            th1 = [0,0];
            th1[0] = thF + th1F;
            th1[1] = thF - th1F;
            
            thG = math.atan2(yc,xc - self.L[0]);
            th4G = GetAngle(self.L[4],G,self.L[3]);
            th4  = [0,0];
            th4[0] = thG + th4G;
            th4[1] = thG - th4G;
            
            #Select the first set of angles that do not intersect
            for self.th[1] in th1:
                for self.th[4] in th4:
                    if(not self.DriveArmsIntersect()):
                        break;
                else:
                    continue;
                break;
                    
            # self.th[1] = (GetAngle(self.L[0],F,G) + GetAngle(self.L[1],F,self.L[2]));
            self.x[1] = self.L[1]*cos(self.th[1]);
            self.y[1] = self.L[1]*sin(self.th[1]);
            self.th[2] = GetAngleByPoints(self.x[1],self.y[1],xc,yc);
             
            # self.th[4] = (math.pi - (GetAngle(self.L[0],G,F) + GetAngle(self.L[4],G,self.L[3])));
            self.x[4] = self.L[0] + self.L[4]*cos(self.th[4]);
            self.y[4] = self.L[4]*sin(self.th[4]);
            self.th[3] = GetAngleByPoints(self.x[4],self.y[4],xc,yc);
            
        except ValueError:
            print(self.th[1], "no solution\n");
            return 1;
        
    def ShowPosture(self):
        #plt.cla();
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
        
        #print(Distance(x[3],y[3],x[2],y[2]));
        plt.plot(x[2],y[2],'--rs',markersize = 1);
        #plt.axes().set_aspect('equal')
        #plt.xlim([-2, 4.5]);
        #plt.ylim([-4, 4]);
        #plt.draw();
    
    def SetDriveArmPositions(self, th1, th4):
        self.th[1] = th1;
        self.th[4] = th4;
        self.x[2] = self.L[1]*cos(self.th[1]);
        self.y[2] = self.L[1]*sin(self.th[1]);
        
        self.x[3] = self.L[0] + self.L[4]*cos(self.th[4]);
        self.y[3] = self.L[4]*sin(self.th[4]);
        
        dist2to3 = Distance(self.x[2],self.y[2],self.x[3],self.y[3])
        if( dist2to3 > (self.L[2] + self.L[3])):
            print("No solution\n");
            return 1;
        
        if(self.DriveArmsIntersect()):
            print("Drive Arms Intersect\n");
            return 1;
            
        self.th[2] = GetAngle(dist2to3,self.L[2],self.L[3]) + GetAngleByPoints(self.x[2],self.y[2],self.x[3],self.y[3]);
            
    def DriveArmsIntersect(self):
        #Set up the linear algebra equations
        A = np.array([[tan(self.th[1]), -1],[tan(self.th[4]), -1]]);
        B = np.array([0, self.L[0]*tan(self.th[4])]);
        
        #compute the determinant
        det = np.linalg.det(A);
        
        #find the intersection point
        if(det == 0): #if parallel
            #check that the arms are not pointing at each other and within the thickness of the beam (mesured in rads for now)
            if(cos(self.th[1]) > 0 and abs(sin(self.th[1])) < 0.05 and cos(self.th[4]) < 0 and abs(sin(self.th[4])) < 0.05):
                return True;
            else:
                return False;
        else:
            X = np.linalg.solve(A,B);
            xInt = X[0];
            yInt = X[1];
        
        #if the intersection point is within the radii of both arms and they are
        #pointed towards each other, there will be a collision
        
        IntDist1 = Distance(xInt,yInt,0,0);
        IntDist4 = Distance(xInt,yInt,self.L[0],0);
        
        OnePointsTowardsFour = cos(self.th[1])>0;
        FourPointsTowardsOne = cos(self.th[4])<0;
        
        
        if(OnePointsTowardsFour and FourPointsTowardsOne and IntDist1 < self.L[1] and IntDist4 < self.L[4]):
            return True;
        else:
            return False;