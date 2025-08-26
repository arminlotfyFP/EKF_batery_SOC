import numpy as np
import matplotlib.pyplot as plt
import os
# print(os.getcwd())
os.chdir("/home/armin/Desktop/EKF_SOC/EKF_batery_SOC")

from ECM import ECM


class EKF_SOC_estimation:
    def __init__(self,R0=0.01,R=0.02,C=2500,SOCInit=1.0, Vmax=4.2, Vmin=2.5, dt=1.0, Q=3.2, Q_proc=np.diag([1e-8,1e-5]), R_meas = 4e-3, P0=np.diag([1e-3,1e-3])):
        self.R0             = float(R0)
        self.R1             = float(R)
        self.C1             = float(C)
        self.Q_Ah           = float(Q)
        self.SOC            = float(SOCInit)
        self.Vmax           = float(Vmax)
        self.Vmin           = float(Vmin)
        self.dt             = float(dt)
        self.x              = np.array([self.SOC,0.0])
        self.Q              = np.array(Q_proc)
        self.R              = float(R_meas)
        self.P              = np.array(P0)

        self.dOCV_dz = self.Vmax-self.Vmin

        model = ECM(R0=self.R0,R=self.R1,C=self.C1,SOCInit=self.SOC, Vmax=self.Vmax, Vmin=self.Vmin, dt=self.dt, Q=self.Q_Ah)

    def predict(self, I):
        z , vrc = self.x

        z_pred      = z-(I * self.dt)/(3600 * self.Q_Ah)
        z_pred      = float(np.clip(z_pred,0.0,1.0))
        vrc_pred    = vrc + self.dt * (I/self.C1 - vrc/(self.R1 * self.C1))

        self.x = np.array([z_pred, vrc_pred])

        # State Jacobian
        Fk = np.array([[1.0, 0.0],
            [0.0, 1.0 - self.dt/(self.R1*self.C1)]], dtype=float)
        # State Covariance
        self.P = Fk @ self.P @ Fk.T + self.Q
        # Force symetric
        self.P = 0.5 * (self.P + self.P.T)
        return self.x.copy(), self.P.copy()
    
    def update(self, v_meas, I):

        z_pred, vrc_pred = self.x

        V_pred = self.Vmin + z_pred * (self.Vmax-self.Vmin) - vrc_pred -self.R0*I

        # Measurement Jacobian
        Hk = np.array([self.dOCV_dz, -1.0], dtype=float)
        y = float(v_meas-V_pred)
        S = float(Hk @ self.P @ Hk.T + self.R)
        # Kalman gain
        K = (self.P @ Hk.T)/S

        # Updating estimation
        self.x = self.x + K.flatten() *y
        self.x[0] = self.x[0] = float(np.clip(self.x[0], 0.0, 1.0))
        self.P = (np.eye(2) - (K@ Hk)) @ self.P @ (np.eye(2) - (K@ Hk)).T + K * self.R * K.T
        # Force symetric
        self.P = 0.5 * (self.P + self.P.T)
        return self.x.copy(), self.P.copy(), V_pred,y,S,K
    
    def step(self, I, V_measure):
        self.predict(I)
        return self.update(V_measure, I)