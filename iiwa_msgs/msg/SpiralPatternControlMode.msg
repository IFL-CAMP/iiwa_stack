# The plane in the Cartesian space for performing the spiral-pattern. e.g. (XY, XZ, YZ)
int32 cartesian_plane

# The value of the frequency for the two sine oscillations [Hz] 
float64 frequency

# The value of the amplitude. # KUKA DOC says "CartDOF X,Y,Z [N] / A,B,C [Nm]" which doesn't make sense.
float64 amplitude

# The value of the stiffness for the degree of freedoms of the given plane [N/m]
float64 stiffness

# The time duration for expanding the spiral pattern [s].
float64 total_time