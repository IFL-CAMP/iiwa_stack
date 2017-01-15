int32 JOINT_IMPEDANCE = 0
int32 CARTESIAN_IMPEDANCE = 1
int32 CONSTANT_FORCE = 2

int32 X = 1
int32 Y = 2
int32 Z = 3

int32 mode

int32 constant_force_direction
float32 constant_force
float32 constant_force_stiffness

float32 nullspace_stiffness
float32 nullspace_damping

JointStiffness joint_stiffness
JointDamping joint_damping

CartesianStiffness cartesian_stiffness
CartesianDamping cartesian_damping
