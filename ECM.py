import numpy as np
import matplotlib.pyplot as plt


class ECM:
    def __init__(self,R0=0.01,R=0.02,C=2500,SOCInit=1.0, Vmax=4.2, Vmin=2.5, dt=1, Q=3.2):
        
        if R0 is None or R is None or C is None or Q is None:
            raise ValueError("R0, R, C, and Q must be provided (non-None).")
        if dt <= 0 or R <= 0 or C <= 0 or Q <= 0:
            raise ValueError("Require dt>0, R>0, C>0, Q>0.")
        
        self.Q          = Q
        self.R0         = R0
        self.R          = R
        self.C          = C
        self.soc        = SOCInit
        self.Vmin       = Vmin
        self.Vmax       = Vmax
        self.dt         = dt
        self.Vrc        = 0.0

    def OCV(self):
        return self.Vmin + self.soc*(self.Vmax-self.Vmin)
    
    def VRC(self, current):
        self.Vrc += self.dt * ((current/self.C)- (self.Vrc/(self.R * self.C)))
        return self.Vrc
    
    def SOC(self, current):
        self.soc -= (current * self.dt) / (3600 * self.Q)  # I>0 discharges
        self.soc = float(np.clip(self.soc, 0.0, 1.0))
        return self.soc
    
    def step(self,current):

        soc = self.SOC(current)
        V_terminal = self.OCV() - (current*self.R0) - self.VRC(current)
        

        return V_terminal, soc