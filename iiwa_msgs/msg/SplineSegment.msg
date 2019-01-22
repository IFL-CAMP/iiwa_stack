# This message describes a segment of a spline path

int32 SPL = 0
int32 LIN =  1
int32 CIRC = 2

# The type of the spline segment
int32 type

# The endpoint of the current segment
CartesianPose point

# Auxiliary point. Only used for circular segments
CartesianPose point_aux