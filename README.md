# Kalman_HW

Kalman Filter Project by using Real-Time Kinematic Precise Point Positioning (RTK-PPP) output data.

Requirements:
- Numpy
- Matplotlib


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
        
```
# References

- Laurichesse, D., Privat, A., "An Open-source PPP Client Implementation for the CNES PPP-WIZARD Demonstrator," Proceedings of the 28th International Technical Meeting of the Satellite Division of The Institute of Navigation (ION GNSS+ 2015), Tampa, Florida, September 2015, pp. 2780-2789.
