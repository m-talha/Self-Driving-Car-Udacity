# **PID Control**
---

## Overview
PID stands for Proportional-Integral-Derivative control. It is one of the most fundamental controllers for closed-loop control of a system. 

The controller takes as input the desired set point and controls the process value through outputs from the P, I and D components to bring the process value to the set point.

### Error Signal: Cross Track Error (CTE)
The error signal for this PID controller is the CTE. This is defined as the lateral displacement of the vehicle from the reference trajectory.

In this scenario, the set point is the reference trajectory at CTE = 0 and the process value is the current CTE. The output from the controller is the steering angle.

The PID controller takes the error (set point - process value) and determines how to control the output (steering angle) to bring the process value (current CTE) to the desired set point (CTE = 0).

## P Component

The proportional component of the controller aims to reduce the error in proportion to the error signal.

In this scenario, the error signal is the CTE and the controller sets the steering angle to be in proportion (but opposite) to correct the error:

```math
-tau_p * cte
```

where `tau_p` is the proportional factor.

### Overshoot

Proportional control alone is not enough. It leads to oscillations between overshooting and undershooting as the controller tries to converge on the desired target. This is known as **marginal stability**.

## D Component

To resolve this, a derivative component is added to the controller which relates the steering angle to the gradient of the error signal.

As the magnitude of the CTE decreases, so does the gradient. The derivative controller produces a signal in proportion to counteract this rapid change:

```math
d_error = (cte - prev_cte) / dt
-tau_d * d_error
```

where `tau_d` is the derivative factor.

This has the effect of adding damping to the system leading to a steady state.

The derivative controller must be coupled with either a P, I or PI controller to guide the system to a steady state since it only knows about the rate of change of error.

## I Component

The integral controller helps to counteract a bias in the system such as an offset in the steering angle due to the misalignment of the car wheels.

To counteract this effect requires a sustained period of correction using the integral of all observed errors:

```math
i_error += cte
-tau_i * i_error
```

where `tau_i` is the integral factor.

## PID Controller

The above can be combined to give the following controller for the CTE:

```math
steer = (-tau_p * cte) - (tau_d * d_error) - (tau_i * i_error)
```

For the PID controller to be effective, the parameters `tau_p`, `tau_i` and `tau_d` need to be tuned. This can be done manually or using the Twiddle algorithm.

### Parameter Tuning

For this project, initial values for the parameters were determined through trial and error. 

This was achieved as follows:

```
- Initialise parameters Kd, Ki, Kp to 0

- Increase the proportional term (Kp) until the vehicle begins to oscillate

- Increase the derivative term (Kd) to dampen the oscillations

- For this scenario, there isn't a strict bias so the integral term (Ki) was set to a very low value to help reduce any steady state error.
```

This resulted in the following initial values for the parameters:

```
Kp = 0.1, Ki = 0.0001, Kd = 3.0
```

These values were further tuned using the Twiddle algorithm. This is an optimsation algorithm that tries to find the optimal parameters for a function that returns an error. 

The algorithm runs a continuous cycle of parameter tweaking (positive and negative) and trial runs to evaluate the error. If the error is reduced, the new values are kept, otherwise, they are tweaked again by a smaller magnitude. This is repeated until the sum of the magnitudes for the tweaks is below a user specified threshold.

The code for the algorithm is implemented in [twiddle.hpp](./src/twiddle.hpp)/[.cpp](./src/twiddle.cpp) files and used in [main.cpp](./src/main.cpp).

After running the algorithm for 50 iterations of the whole track, the result of the tuning was a bit anti-climatic:

```
Kp = 0.11 (0.1), Ki = 0.0001, Kd = 2.87 (3.0) 
```

## Result
---
The result of the tuning process can be seen in the accompanying [video](./output_vids/pid_control_comp.mp4).