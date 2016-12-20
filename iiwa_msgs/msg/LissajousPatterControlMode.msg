# The plane in the Cartesian space for performing the lissajous-pattern. e.g. (XY, XZ, YZ)
CartPlane cartesian_plane

# The value of the frequency for the sine oscillation [Hz] 
float64 frequency

# The value of the amplitude. # KUKA DOC says "CartDOF X,Y,Z [N] / A,B,C [Nm]" which doesn't make sense.
float64 amplitude

# The value of the stiffness. # KUKA DOC says "CartDOF X,Y,Z [N/m] / A,B,C [Nm/rad]" which doesn't make sense.
float64 stiffness