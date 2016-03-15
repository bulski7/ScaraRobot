import math;
def GetAngle(adj1, adj2, opposite):
    try:
        return math.acos((adj1**2 + adj2**2 - opposite**2)/(2*adj1*adj2));
    except ZeroDivisionError:
        return 0;
        
    
def GetAngleByPoints(x1,y1,x2,y2):
    return math.atan2(y2-y1,x2-x1);
    
def Distance(x1,y1,x2,y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2);
    
def sign(a):
    return (a > 0) - (a < 0)