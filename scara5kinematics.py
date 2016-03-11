class 5Bar:
'5 Bar robot object

    def __init__(self, L, th)
        self.L = [1,1,1,1,1];
        self.th = [1,1,1,1,1];

    def SetCartesianPosition(self, EndEffectorX, EndEffectorY)
        xc = EndEffectorX;
        yc = EndEffectorY;

        A = xc;
        B = yc;
        C = (self.L[1]^2 - self.L[2]^2 + xc^2 + yc^2)/(2*self.L[1]));
        self.th[1] = 2*atan((-B+sqrt(A^2 + B^2 - C^2))/(-A-C));

        A = xc - self.L[0];
        B = yc;
        C = (self.L[4]^2 + self.L[0]^2 - self.L[3]^2 - 2*xc*self.L[0]) + xc^2 + yc^2)/(2*self.L[4]);
        self.th[4] = 2*atan((-B+sqrt(A^2 + B^2 - C^2))/(-A-C));
        

        
