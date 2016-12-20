

# Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad]. 
Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0. 
CartesianQuantity cartesian_stiffness

# Dimensionless stiffness and damping values for the cartesian impedance control, for all degrees of freedom : [x, y, z, a, b, c].
# Precondition: 0.1 <= x, y, z, a, b, c <= 1.0. 
CartesianQuantity cartesian_damping

# The stiffness value for null space [Nm/rad]. 
# Precondition: 0.0 <= value. 
float64 nullspace_stiffness

# The damping parameter for null space [Nm*s/rad]. 
# Precondition: value >= 0.3 and value <= 1.0. - A good damping value is 0.7. 
float64 nullspace_damping

# Sets the maximum cartesian deviation accepted. If the deviation is exceeded a stop condition occurs. 
# [x, y, z] in [mm]. Precondition: value >= 0.0.
# [a, b, c] in [rad]. Precondition: value >=0.0
CartesianQuantity max_path_deviation

# The maximum control force parameter.
# [x, y, z] in [N]
# [a, b, c] in [Nm]
# Returns null if value not set.
CartesianQuantity max_control_force

# Indicates whether a stop condition is fired if the maximum control force is exceeded. 
bool max_control_force_stop

# Cartesian velocity parameter 
# [x, y, z] in [mm/s]
# [a, b, c] in [rad/s]
CartesianQuantity max_cartesian_velocity