# Classic Cartesian impedance settings
CartesianImpedanceControlMode cartesian_impedance

# Amplitude in [N, N, N, Nm, Nm, Nm] of sinusoidal output pattern. 
CartesianQuantity amplitude

# Frequency in Hertz of sinusoidal motion.
CartesianQuantity frequency

# Phase in Degrees of the control mode. # NOT CLEAR FROM KUKA DOCS
CartesianQuantity phase

# Offset in [N N N Nm Nm Nm] of Amplitude. 
CartesianQuantity bias

# Feed forward force limit in [N, N, N, Nm, Nm, Nm]. 
CartesianQuantity force_limit

# Feed forward position limit in [mm, mm, mm, rad, rad, rad].
CartesianQuantity position_limit

# Rise time of the control mode [s] 
float64 rise_time

# Hold time of the control mode [s].
float64 hold_time

# Fall time of the control mode [s] 
float64 fall_time