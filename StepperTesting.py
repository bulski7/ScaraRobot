import StepperMotor
from StepperMotor import StepperMotor
import math
import threading
import time

class MoveThread(threading.Thread):
    def __init__(self, threadID, motor, position, velocity):
        threading.Thread.__init__(self)
        self.motor    = motor;
        self.threadID = threadID
        self.position = position
        self.velocity = velocity
        
    def run(self):
        print("Starting " + self.threadID)
        self.motor.Move(self.position, self.velocity)
        print("Exiting " + self.threadID)
        

a = StepperMotor(5,6,6400)
b = StepperMotor(13,19,6400)

#create new thread objs
thread1 = MoveThread("1", a, math.pi/2+0.1, 0.2);
thread2 = MoveThread("2", b, math.pi/2-0.1, 0.2);

#start the threads
thread1.start()
thread2.start()

# wait until the threads terminate
thread1.join()
thread2.join()

#create new position to move to
thread1 = MoveThread("1", a, math.pi/2+0.2, 0.2);
thread2 = MoveThread("2", b, math.pi/2-0.2, 0.2);


#start the threads
thread1.start()
thread2.start()

# wait until the threads terminate
thread1.join()
thread2.join()

##
##
##threading.Threadstart_new_thread(a.Move,(math.pi/2+0.1,0.1));
##thread.start_new_thread(b.Move,(math.pi/2-0.1,0.1));
##
##time.sleep(0.5)
##
##threading.start_new_thread(a.Move,(math.pi/2+0.2,0.1));
##thread.start_new_thread(b.Move,(math.pi/2-0.2,0.1));
##
####a.Move(1.6,0.1);
####b.Move(1.6,0.1);
##threading.Thread.__init__()
##
