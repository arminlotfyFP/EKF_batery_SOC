import os
print(os.getcwd())
os.chdir("/home/armin/Desktop/EKF_SOC/EKF_batery_SOC")

import numpy as np
import matplotlib.pyplot as plt

from ECM import ECM
from EKF import EKF_SOC_estimation


plant = ECM(R0=0.01, R=0.02, C=2500, SOCInit=0.95, Vmax=4.2, Vmin=2.5, dt=1.0, Q=3.2)

ekf = EKF_SOC_estimation(R0=plant.R0, R=plant.R, C=plant.C, Q_Ah=plant.Q,
                       Vmin=plant.Vmin, Vmax=plant.Vmax, dt=plant.dt,
                       Q_proc=np.diag([1e-8, 1e-5]), R_meas=4e-3,
                       x0=np.array([0.90, 0.0]), P0=np.diag([1e-3, 1e-3]))

N = 300
rng = np.random.default_rng(0)

I_hist, V_meas_hist = [], []
soc_true_hist, soc_est_hist = [], []
vrc_est_hist = []

for k in range(N):

    I = 2.0 + 3.0 * (np.sin(2*np.pi*k/60.0) + 0.2*rng.standard_normal())
    I = float(max(I, 0.0))   # discharge only for demo
    I_hist.append(I)

    V_true, soc_true = plant.step(I)
    soc_true_hist.append(soc_true)


    V_meas = V_true + rng.normal(0.0, np.sqrt(ekf.R))
    V_meas_hist.append(V_meas)


    (x_post, P_post, V_pred, resid, S, K) = ekf.step(I, V_meas)
    soc_est_hist.append(x_post[0])
    vrc_est_hist.append(x_post[1])


fig, ax = plt.subplots(3, 1, figsize=(9,7), sharex=True)
ax[0].plot(I_hist, label='Current (A)')
ax[1].plot(V_meas_hist, label='Measured V (V)')
ax[1].plot([plant.Vmin + z*(plant.Vmax-plant.Vmin) - vr - i*plant.R0
            for z, vr, i in zip(soc_est_hist, vrc_est_hist, I_hist)],
           label='EKF V_pred', alpha=0.7)
ax[2].plot(soc_true_hist, label='SOC True')
ax[2].plot(soc_est_hist, label='SOC EKF', alpha=0.8)

for a in ax: a.grid(True); a.legend()
ax[2].set_xlabel('Step')
plt.tight_layout(); plt.show()
