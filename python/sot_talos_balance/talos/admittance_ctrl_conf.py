from math import sqrt
import numpy as np

kp_force = 6*(0.0,)
ki_force = 6*(0.0,)
kp_vel   = 32*(0.0,)
ki_vel   = 32*(0.0,)
force_integral_saturation = (0.0, 0.0, 160.0, 20.0, 20.0, 0.0)
force_integral_deadzone   = (5.0, 5.0, 5.0, 0.5, 0.5, 0.5)
