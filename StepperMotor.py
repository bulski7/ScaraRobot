import RPi.GPIO as GPIO
import time, math

class StepperMotor:
    def __init__(self, DirectionPin, PulsePin, NumSteps):
        #define pinout
        self.DirectionPin = DirectionPin;
        self.PulsePin = PulsePin;

        #how many steps per revolution
        self.NumSteps = NumSteps;

        GPIO.setmode(GPIO.BCM);
        GPIO.setwarnings(False);

        #Setup Motor pins as outputs
        GPIO.setup(self.PulsePin, GPIO.OUT);
        GPIO.setup(self.DirectionPin, GPIO.OUT);

    def Step(self):
        GPIO.output(self.PulsePin, False);
        time.sleep(10e-6);
        GPIO.output(self.PulsePin,True);
        return

    def SetDirection(self, Direction):
        if(Direction > 0):
            GPIO.output(self.DirectionPin, False);
        else:
            GPIO.output(self.DirectionPin, True);

    def Move(self, Position, Velocity):
        self.SetDirection(math.copysign(1,Position))
        numPulses = self.NumSteps*(abs(Position)/(2*math.pi))
        stepTime = 2*math.pi/Velocity/self.NumSteps;

        for i in range(1,int(numPulses)):
            self.Step();
            time.sleep(stepTime);
        return    
    
