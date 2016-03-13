import math;
def GetAngle(adj1, adj2, opposite):
    return math.acos((adj1**2 + adj2**2 - opposite**2)/(2*adj1*adj2));
    
def Distance(x1,y1,x2,y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2);