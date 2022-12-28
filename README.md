# Kalman_HW

Kalman Filter Project by using Real-Time Kinematic Precise Point Positioning (RTK-PPP) output data.

Requirements:
- Numpy
- Matplotlib

# Kalman Filter

```math
    X_{kp} = AX_{k-1} + B \mu  + \omega_k
```

```math
    P_{kp} = AP_{k-1}A^T + Q_k
```

```math
    KG = \frac{P_{kp}H}{HP_{kp}H^T + R}
```

```math
    Y_k = H X_{k} + z_k
```

```math
    X_k = X_{kp}+ KG(Y_k - HX_{kp})
```

```math
    P_k = (I - KG \cdot H) P_{kp}
```
# Model

```math
    A = \begin{bmatrix}
            1 & \Delta t & 0 & 0 & 0 & 0 \\
            0 & 1        & 0 & 0 & 0 & 0 \\
            0 & 0        & 1 & \Delta t  & 0 & 0 \\
            0 & 0        & 0 & 1 & 0 & 0 \\
            0 & 0        & 0 & 0 & 1 & \Delta t  \\
            0 & 0        & 0 & 0 & 0 & 1 
        \end{bmatrix} \\
    X = \begin{bmatrix}
            x \\
            \dot{x} \\
            y \\
            \dot{y} \\
            z \\
            \dot{z} 
        \end{bmatrix} \\
    AX = \begin{bmatrix}
            x + \dot{x} \Delta t\\
            \dot{x} \\
            y + \dot{y} \Delta t\\
            \dot{y} \\
            z + \dot{z} \Delta t\\
            \dot{z} 
        \end{bmatrix} \\
```
```math        
    B = \begin{bmatrix}
            \Delta t^2 & 0 & 0 & 0 & 0 & 0 \\
            0 & \Delta t & 0 & 0 & 0 & 0 \\
            0 & 0 & \Delta t^2 & 0 & 0 & 0 \\
            0 & 0 & 0 & \Delta t & 0 & 0 \\
            0 & 0 & 0 & 0 & \Delta t^2 & 0  \\
            0 & 0 & 0 & 0 & 0 & \Delta t 
        \end{bmatrix} \\
    \mu = \begin{bmatrix}
            \ddot{x} \\
            \ddot{x} \\
            \ddot{y} \\
            \ddot{y} \\
            \ddot{z} \\
            \ddot{z} 
        \end{bmatrix} \\   
```
```math        
    H = \begin{bmatrix}
            1 & 0 & 0 & 0 & 0 & 0 \\
            0 & 0 & 0 & 0 & 0 & 0 \\
            0 & 0 & 1 & 0 & 0 & 0 \\
            0 & 0 & 0 & 0 & 0 & 0 \\
            0 & 0 & 0 & 0 & 1 & 0  \\
            0 & 0 & 0 & 0 & 0 & 0 
        \end{bmatrix} \\
        
    R = \begin{bmatrix}
            \sigma_{x}^2 & 0 & 0 & 0 & 0 & 0 \\
            0 & \sigma_{\dot{x}}^2 & 0 & 0 & 0 & 0 \\
            0 & 0 & \sigma_{y}^2 & 0 & 0 & 0 \\
            0 & 0 & 0 & \sigma_{\dot{y}}^2 & 0 & 0 \\
            0 & 0 & 0 & 0 & \sigma_{z}^2 & 0  \\
            0 & 0 & 0 & 0 & 0 & \sigma_{\dot{z}}^2 
        \end{bmatrix} \\
    P = \begin{bmatrix}
            P_{x}^2 & 0 & 0 & 0 & 0 & 0 \\
            0 & P_{\dot{x}}^2 & 0 & 0 & 0 & 0 \\
            0 & 0 & P_{y}^2 & 0 & 0 & 0 \\
            0 & 0 & 0 & P_{\dot{y}}^2 & 0 & 0 \\
            0 & 0 & 0 & 0 & P_{z}^2 & 0  \\
            0 & 0 & 0 & 0 & 0 & P_{\dot{z}}^2 
        \end{bmatrix} \\
        
```
Mean of measurements covariance were used as process covariance values.
```math 
    P = \begin{bmatrix}
            \overline{\sigma_{x}}^2 & 0 & 0 & 0 & 0 & 0 \\
            0 & \overline{\sigma_{x}}^2 & 0 & 0 & 0 & 0 \\
            0 & 0 &\overline{\sigma_{y}}^2 & 0 & 0 & 0 \\
            0 & 0 & 0 & \overline{\sigma_{y}}^2 & 0 & 0 \\
            0 & 0 & 0 & 0 & \overline{\sigma_{z}}^2 & 0  \\
            0 & 0 & 0 & 0 & 0 & \overline{\sigma_{z}}^2 
        \end{bmatrix} \\

```
# References

- Laurichesse, D., Privat, A., "An Open-source PPP Client Implementation for the CNES PPP-WIZARD Demonstrator," Proceedings of the 28th International Technical Meeting of the Satellite Division of The Institute of Navigation (ION GNSS+ 2015), Tampa, Florida, September 2015, pp. 2780-2789.
