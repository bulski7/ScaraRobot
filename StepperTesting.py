import StepperMotor
from StepperMotor import StepperMotor
import math

a = StepperMotor(5,6,6400)
b = StepperMotor(13,19,6400)

a.Move(0.5,0.1);
b.Move(0.5,0.1);

a.Move(-0.5,0.1);
b.Move(-0.5,0.1);
