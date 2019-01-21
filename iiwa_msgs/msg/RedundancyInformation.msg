# E1 parameter
# Specifies Angle of axis 3
float64 e1

# Status parameter
#   Bit 0: 1 - The working is working above its head
#          0 - The robot is working in the ground area
#   Bit 1: 1 - Angle A4 < 0°
#          0 - Angle A4 >= 0°
#   Bit 2: 1 - Angle A6 <= 0
#          0 - Angle A6 > 0
int32 status

# Turn parameter
# according to Sunrise handbook this is not used for iiwa
int32 turn