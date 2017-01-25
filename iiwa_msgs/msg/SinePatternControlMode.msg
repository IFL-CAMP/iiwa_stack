# The degree of freedom for performing the sine oscillation.
int32 cartesian_dof

# The value of the frequency for the sine oscillation [Hz] 
float64 frequency

# The value of the amplitude. # KUKA DOC says "CartDOF X,Y,Z [N] / A,B,C [Nm]" which doesn't make sense.
float64 amplitude

# The value of the stiffness. # KUKA DOC says "CartDOF X,Y,Z [N/m] / A,B,C [Nm/rad]" which doesn't make sense.
float64 stiffness