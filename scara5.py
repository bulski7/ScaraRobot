from math import atan, sqrt, asin, sin, cos, acos, tan
import time, math
from KinHelp import GetAngle, Distance, sign, GetAngleByPoints
import RPi.GPIO as GPIO
import numpy as np
import threading
import matplotlib.pyplot as plt
from StepperMotor import StepperMotor

class MoveThread(threading.Thread):
    def __init__(self, threadID, motor, position, velocity):
        threading.Thread.__init__(self)
        self.motor    = motor;
        self.threadID = threadID
        self.position = position
        self.velocity = velocity
        
    def run(self):
        #print("Starting " + self.threadID)
        self.motor.Move(self.position, self.velocity)
        #print("Exiting " + self.threadID)
        
class FiveBar:
#5 Bar robot object

    def __init__(self):
        self.L = [1,1,1,1,1];   #link length vector
        self.th = [0,0,0,0,0];  #theta vector
        self.x = [0,0,0,0,0];
        self.y = [0,0,0,0,0];

        self.moveVelocity = 0.1; #rad/s

        #add the stepper motor objects
        self.MotorA = StepperMotor(13,19,6400);
        self.MotorB = StepperMotor(5,6,6400);

        #add the pen down output, true = pen down
        GPIO.setup(26, GPIO.OUT);
       
    def PenDown(self):
        GPIO.output(26,True);

    def PenUp(self):
        GPIO.output(26,False);
           
    #Set the link angles to achieve a given X,Y of the end effector
    #Using geometric equations
    def SetEndEffectorPosition(self, EndEffectorX, EndEffectorY, MoveTime = 2):
        xc = EndEffectorX; 
        yc = EndEffectorY;

        #calculate the component velocities of the move to achieve the move time
        #move distance in cartesian coords
        #dx = xc - self.x[2];
        #dy = yc - self.y[2];
        

        

        #move velocities in cartesian coords
        #xd = dx/MoveTime;
        #yd = dy/MoveTime;

        #print("xd: ",xd)
        #print("yd: ",yd)
              
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
            th4[0] = thG - th4G;
            th4[1] = thG + th4G;

            prevth1 = self.th[1];
            prevth4 = self.th[4];
            
            #Select the first set of angles that do not intersect
            for self.th[1] in th1:
                for self.th[4] in th4:
                    if(not self.DriveArmsIntersect()):
                        break;
                else:
                    continue;
                break;

            #calculate the change in theta so the move can be completed in commanded time period
            dth1 = self.th[1] - prevth1;
            dth4 = self.th[4] - prevth4;

            #dont bother to move if it's already at that location
            if(dth1 == 0 and dth4 == 0):
                return;

            #calculate the angular velocity of each motor to complete the move in commanded time period along time-optimized path
            th1d = dth1/MoveTime
            th4d = dth4/MoveTime
            
                    
            # self.th[1] = (GetAngle(self.L[0],F,G) + GetAngle(self.L[1],F,self.L[2]));
            self.x[1] = self.L[1]*cos(self.th[1]);
            self.y[1] = self.L[1]*sin(self.th[1]);
            self.th[2] = GetAngleByPoints(self.x[1],self.y[1],xc,yc);
             
            # self.th[4] = (math.pi - (GetAngle(self.L[0],G,F) + GetAngle(self.L[4],G,self.L[3])));
            self.x[4] = self.L[0] + self.L[4]*cos(self.th[4]);
            self.y[4] = self.L[4]*sin(self.th[4]);
            self.th[3] = GetAngleByPoints(self.x[4],self.y[4],xc,yc);

            #calculate the angular velocity to have both motors reach the destination at the same time
            #th1d = (xd + yd*math.tan(self.th[2]))/(self.L[1]*math.sin(self.th[1]) - self.L[1]*math.cos(self.th[1])*math.tan(self.th[2]))
            #th4d = (xd + yd*math.tan(self.th[3]))/(self.L[4]*math.sin(self.th[4]) - self.L[4]*math.cos(self.th[4])*math.tan(self.th[3]))

            #print("th1d: ",  th1d)
            #print("th4d: ",  th4d)
            # Move the motors to that position - sequentially...
            #self.MotorA.Move(self.th[4],self.moveVelocity)
            #self.MotorB.Move(self.th[1],self.moveVelocity)

            #create new thread objs
            thread1 = MoveThread("1", self.MotorA, self.th[4], th4d);
            thread2 = MoveThread("2", self.MotorB, self.th[1], th1d);

            #start the threads
            thread1.start()
            thread2.start()          

            # wait until the threads terminate
            thread1.join()
            thread2.join()
            

            #self.x[2] = EndEffectorX
            #self.y[2] = EndEffectorY
            
            return 0;
            
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
        
        print(Distance(x[3],y[3],x[2],y[2]));
        plt.plot(x,y,'--r');
        plt.axes().set_aspect('equal')
        #plt.xlim([-200, 450]);
        #plt.ylim([-400, 400]);
        plt.show();
    
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

        # Move the motors to that position
        #self.MotorA.Move(self.th[3],self.moveVelocity)
        #self.MotorB.Move(self.th[2],self.moveVelocity)
            
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

    def Step(self, Motor):
        if(Motor == "A"):
            GPIO.output(self.MotorAPulsePin, False);
            time.sleep(10e-6);
            GPIO.output(self.MotorAPulsePin,True);
        elif (Motor == "B"):
            GPIO.output(self.MotorBPulsePin, False);
            time.sleep(10e-6);
            GPIO.output(self.MotorBPulsePin,True);
        else:
            print("cannot step. invalid motor\n");
        return

    def SetDirection(self, Motor, Direction):
        if(Motor == "A"):
            if(Direction > 0):
                GPIO.output(self.MotorADirectionPin, False);
            else:
                GPIO.output(self.MotorADirectionPin, True);
        elif(Motor == "B"):
            if(Direction > 0):
                GPIO.output(self.MotorBDirectionPin, False);
            else:
                GPIO.output(self.MotorBDirectionPin, True);
        return

    
            
            
    
