import RPi.GPIO as GPIO
import time, math

class StepperMotor:
    def __init__(self, DirectionPin, PulsePin, NumSteps):
        #define pinout
        self.DirectionPin = DirectionPin;
        self.PulsePin = PulsePin;
        
        #how many steps per revolution
        self.NumSteps = NumSteps;

        #store the position and setpoint in radians, assume 0 because no homing method yet
        self.position = math.pi/2;
        self.setpoint = math.pi/2;

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

        #move the setpoint to the new position in absolute coords
        self.setpoint += Position - self.position;
        
        #convert to relative position error
        PositionError = self.setpoint - self.position;

        if(PositionError == 0):
            return;
        
        #correct the position error as best as the steps allow
        self.SetDirection(math.copysign(1,PositionError))
        numPulses = math.floor(self.NumSteps*(abs(PositionError)/(2*math.pi)))

        if(numPulses == 0):
            return;
        
        stepTime = 2*math.pi/abs(Velocity)/self.NumSteps;

        if(stepTime < 25e-6):
            stepTime = 25e-6;
            
        #print("NumPulses: ", numPulses)
        #print("StepTime: ", stepTime)
        #print("Goal angle: ", Position)
        #print("Position Error: ", PositionError)

        for i in range(1,int(numPulses)+1):
            self.Step();
            time.sleep(stepTime);

        #reflect the change in position only if the motor stepped
        self.position += numPulses*2*math.pi/self.NumSteps*math.copysign(1,PositionError);
        
        return
    
    
