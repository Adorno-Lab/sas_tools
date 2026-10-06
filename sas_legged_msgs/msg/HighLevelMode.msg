# High-level mode of a legged robot. Used on <prefix>/set/high_level_mode.
#   IDLE:     the robot stays on and balancing; every target is ignored (zero velocity, the limbs hold still).
#   STANDING: the base orientation, the base height, and the commandable limbs can be commanded; the
#             robot cannot walk.
#   WALKING:  the twist and the base height are accepted; the base orientation is not. Only the limbs
#             the driver marks as commandable in WALKING accept targets.
uint8 IDLE=0
uint8 STANDING=1
uint8 WALKING=2

uint8 mode
