# Kalman_HW

Kalman Filter Project by using Real-Time Kinematic positioning data. Positioning data were obtained by using PPP-WIZARD software. GNSS data was acquired with a GNSS receiver attached to the pedestrian's backpack.

Requirements:
- Numpy
- Matplotlib

Project Steps:
- [Input data file](output_lowlevel_itu_kampus_RAIM) contains the (x,y,z) ECEF cartesian coordinates and their variance values.
- Velocity values were calculated from position data by using ```disp2vel``` function.
- Velocity variance values were calculated from position variance data by using ```Sdisp2vel``` function.
- Acceleration values were calculated from velocity data by using ```vel2acc``` function.
- Kalman filtering was done by using ```kalman``` function.
- The obtained (x,y,z) ECEF cartesian coordinates were converted to ellipsoidal coordinates (φ,λ,h).
- Ellipsoidal coordinates were converted to projected/UTM coordinates (Easting, Northing).
- 2D and 3D graphs were drawn.

# Kalman Filter

Kalman Filter is an iteartive mathematical process that uses a set of equations and consecutive data inputs to quickly estimate the true value, position, velocity etc. of the object being measured when the measured values contain unpredicted or random error, uncertainty or variation.

![](https://github.com/Bilalhappy/Kalman_HW/blob/master/pics/chart.png#center)

The kalman gain is used to determine how much of the measurements to use to update the new estimate.​

```math
    KG = \frac{E_{EST}}{E_{EST}+E_{MEA}},    0<= KG <= 1
```
```math
    EST_{t} = EST_{t-1} + KG(MEA - EST_{t-1})
```
```math
    E_{EST_{t}} = \frac{E_{MEA} \cdot E_{EST_{t-1}}}{E_{MEA} + E_{EST_{t-1}}} => E_{EST_{t}} = (1-KG)\cdot E_{EST_{t-1}}
```
![](https://github.com/Bilalhappy/Kalman_HW/blob/master/pics/KG.png)

If error of a measurement or an estimation is higher, it’s less weighted thanks to the Kalman Gain.​

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
            \frac{1}{2}\Delta t^2 & 0 & 0 & 0 & 0 & 0 \\
            0 & \Delta t & 0 & 0 & 0 & 0 \\
            0 & 0 & \frac{1}{2}\Delta t^2 & 0 & 0 & 0 \\
            0 & 0 & 0 & \Delta t & 0 & 0 \\
            0 & 0 & 0 & 0 & \frac{1}{2}\Delta t^2 & 0  \\
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

Means of measurements variance were used as process variance values.

```math 
    P = \begin{bmatrix}
            \overline{\sigma_{x}}^2 & 0 & 0 & 0 & 0 & 0 \\
            0 & \overline{\sigma_{\dot{x}}}^2 & 0 & 0 & 0 & 0 \\
            0 & 0 &\overline{\sigma_{y}}^2 & 0 & 0 & 0 \\
            0 & 0 & 0 & \overline{\sigma_{\dot{y}}}^2 & 0 & 0 \\
            0 & 0 & 0 & 0 & \overline{\sigma_{z}}^2 & 0  \\
            0 & 0 & 0 & 0 & 0 & \overline{\sigma_{\dot{z}}}^2 
        \end{bmatrix} \\

```

```math
    X_{kp} =  AX_{k-1} + B \mu =
        \begin{bmatrix}
            x_{k-1}+\dot{x}_{k-1}\Delta t+\ddot{x}_{k-1}\frac{1}{2}\Delta t^2 \\
            \dot{x}_{k-1} +\ddot{x}_{k-1}\Delta t \\
            y_{k-1}+\dot{y}_{k-1}\Delta t+\ddot{y}_{k-1}\frac{1}{2}\Delta t^2  \\
            \dot{y}_{k-1} +\ddot{y}_{k-1}\Delta t  \\
            z_{k-1}+\dot{z}_{k-1}\Delta t+\ddot{z}_{k-1}\frac{1}{2}\Delta t^2  \\
            \dot{z}_{k-1} +\ddot{z}_{k-1}\Delta t 
        \end{bmatrix} \\
```
# References

- Laurichesse, D., Privat, A., "An Open-source PPP Client Implementation for the CNES PPP-WIZARD Demonstrator," Proceedings of the 28th International Technical Meeting of the Satellite Division of The Institute of Navigation (ION GNSS+ 2015), Tampa, Florida, September 2015, pp. 2780-2789.

