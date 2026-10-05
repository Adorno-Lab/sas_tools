# High-level mode of a legged robot. Used on <prefix>/set/high_level_mode.
#   IDLE:     the robot stays on and balancing; every target is ignored (zero velocity, arms hold still).
#   STANDING: the arms and the base orientation can be commanded; the robot cannot walk.
#   WALKING:  the twist is accepted; the base orientation is not. Arms only if supports_manipulation_while_walking.
uint8 IDLE=0
uint8 STANDING=1
uint8 WALKING=2

uint8 mode
