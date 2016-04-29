import RPi.GPIO as GPIO
import time, math

class StepperMotor:
    def __init__(self, DirectionPin, PulsePin, NumSteps):
        #define pinout
        self.DirectionPin = DirectionPin;
        self.PulsePin = PulsePin;
        
        #how many steps per revolution
        self.NumSteps = NumSteps;

        #store the position in radians, assume 0 because no homing method yet
        self.position = math.pi/2;

        #setup GPIO
        GPIO.setmode(GPIO.BCM);
        GPIO.setwarnings(False);

        #Setup Motor pins as outputs
        GPIO.setup(self.PulsePin, GPIO.OUT);
        GPIO.setup(self.DirectionPin, GPIO.OUT);

        #Start the pulse pin high
        GPIO.output(self.PulsePin,True);
        
    def Step(self):
        GPIO.output(self.PulsePin,False);
        time.sleep(10e-6);
        GPIO.output(self.PulsePin, True);
        time.sleep(10e-6);        
        return

    def SetDirection(self, Direction):
        if(Direction < 0):
            GPIO.output(self.DirectionPin, False);
        else:
            GPIO.output(self.DirectionPin, True);

    def Move(self, Position, Velocity):

        #convert to absolute position
        Position = Position - self.position;
        
        self.SetDirection(math.copysign(1,Position))
        numPulses = self.NumSteps*(abs(Position)/(2*math.pi))
        stepTime = 2*math.pi/Velocity/self.NumSteps;
        
        #print("NumPulses: ", numPulses)
        #print("StepTime: ", stepTime)
        print("Goal angle: ", Position)

        for i in range(1,int(numPulses)+1):
            self.Step();
            time.sleep(stepTime);

        self.position = self.position + Position;#numPulses*2*math.pi/self.NumSteps*self.;
        return
    
    
