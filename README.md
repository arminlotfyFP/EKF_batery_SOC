# Battery Model with EKF (1-RC Thevenin)

## RC Branch Voltage $V_{rc}$

Two common methods to compute the RC branch voltage in a single RC branch:

1. **Discrete method with small $\Delta t$** (used in this repo).
2. **Analytical solution**:  

   $$V_{rc}(t) = \big(V_{rc}(0) - R I\big)\,e^{-t/\tau} + R I, \qquad \tau = R C$$

---

## EKF Overview

The Extended Kalman Filter (EKF) maintains a belief over the hidden state:

- **Mean:** $x$
- **Uncertainty (covariance):** $P$

At each time step:

1. **Predict** the next state and its uncertainty before seeing the new measurement.
2. **Update** that belief using the actual sensor measurement.

---

## Battery Model (1-RC Thevenin)

- **State vector** $(n=2)$:  

  $$x =
  \begin{bmatrix}
  z \\
  V_{rc}
  \end{bmatrix}$$

  - $z$: State of Charge (SOC), normalized $[0,1]$  
  - $V_{rc}$: RC polarization voltage (V)

- **Input:** $I_k(\Delta t)$ (A), **positive = discharge**  
- **Measurement:** Terminal voltage $V_k$ (V)

### Process Model (nonlinear, discrete-time)

<img width="483" height="92" alt="process-model" src="https://github.com/user-attachments/assets/593f8aa8-2aff-4851-911c-d852019736bb" />

### Measurement Model

<img width="259" height="53" alt="measurement-model" src="https://github.com/user-attachments/assets/a51e8672-4777-47a3-b354-69a57892d5e1" />

---

## Notation

- $R$: Ohmic resistance (Ω)  
- $C$: Capacitance (F)  
- $\tau = RC$: RC time constant (s)  
- $\Delta t$: Discretization step (s)  
- $I$: Current (A), **positive for discharge**  
- $V_{rc}$: RC branch voltage (V)  
- $V_k$: Measured terminal voltage at step $k$ (V)  

---

# Extended Kalman Filter (EKF) Equations

This section describes the EKF equations for the 1-RC Thevenin battery model, with matrix **shapes** provided for clarity.

---

## (A) Predict Step

**State prediction:**

$$
x_k^- = f(x_{k-1}, u_k) =
\begin{bmatrix}
z - \dfrac{\eta I \Delta t}{Q_c} \\
a v_{rc} + b I
\end{bmatrix}
$$

- $x_k^- \in \mathbb{R}^{2\times 1}$ : predicted state vector  
- $I$: current input  
- $Q_c$: battery capacity  
- $\eta$: Coulombic efficiency  
- $a, b$: RC branch parameters  

---

**State Jacobian:**

$$
F_k = \frac{\partial f}{\partial x} =
\begin{bmatrix}
1 & 0 \\
0 & a
\end{bmatrix}, \quad
F_k \in \mathbb{R}^{2\times 2}
$$

---

**Covariance prediction:**

$$
P_k^- = F_k P_{k-1} F_k^\top + Q
$$

- $P_k^- \in \mathbb{R}^{2\times 2}$: predicted state covariance  
- $Q \in \mathbb{R}^{2\times 2}$: process noise covariance  

---

## (B) Update Step

**Predicted measurement:**

$$
\hat{z}_k = h(x_k^-, u_k) = OCV(z^-) - v_{rc}^- - I R_0
$$

- $\hat{z}_k \in \mathbb{R}^{1\times 1}$: predicted terminal voltage  
- $R_0$: ohmic resistance  

---

**Measurement Jacobian:**

$$
H_k = \frac{\partial h}{\partial x} =
\begin{bmatrix}
\dfrac{d\,OCV}{dz} & -1
\end{bmatrix}, \quad
H_k \in \mathbb{R}^{1\times 2}
$$

---

**Innovation (residual):**

$$
y_k = z_k^{meas} - \hat{z}_k, \quad y_k \in \mathbb{R}^{1\times 1}
$$

---

**Innovation covariance:**

$$
S_k = H_k P_k^- H_k^\top + R, \quad S_k \in \mathbb{R}^{1\times 1}
$$

---

**Kalman gain:**

$$
K_k = P_k^- H_k^\top S_k^{-1}, \quad K_k \in \mathbb{R}^{2\times 1}
$$

---

**State update:**

$$
x_k^+ = x_k^- + K_k y_k, \quad x_k^+ \in \mathbb{R}^{2\times 1}
$$

---

**Covariance update (Joseph form):**

$$
P_k^+ = (I - K_k H_k) P_k^- (I - K_k H_k)^\top + K_k R K_k^\top, \quad P_k^+ \in \mathbb{R}^{2\times 2}
$$

---
