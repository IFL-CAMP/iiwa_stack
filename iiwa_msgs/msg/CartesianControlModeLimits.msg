# Sets the maximum cartesian deviation accepted. If the deviation is exceeded a stop condition occurs. 
# [x, y, z] in [mm]. Precondition: value > 0.0.
# [a, b, c] in [rad]. Precondition: value > 0.0.
CartesianQuantity max_path_deviation

# The maximum control force parameter.
# [x, y, z] in [N]. Precondition: value > 0.0.
# [a, b, c] in [Nm]. Precondition: value > 0.0.
CartesianQuantity max_control_force

# Indicates whether a stop condition is fired if the maximum control force is exceeded. 
bool max_control_force_stop

# Maximum Cartesian velocity parameter 
# [x, y, z] in [mm/s]. Precondition: value > 0.0.
# [a, b, c] in [rad/s]. Precondition: value > 0.0.
CartesianQuantity max_cartesian_velocity 
