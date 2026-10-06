# Current state of a legged robot driver. Published on <prefix>/get/status.
std_msgs/Header header

# One of the HighLevelMode constants.
uint8 mode

# What the driver accepts in the current mode.
bool accepts_twist
bool accepts_base_orientation
bool accepts_base_height

# One entry per limb of get/info (same order): true if <prefix>/<name>/set/target_joint_positions
# takes effect in the current mode. All false in IDLE.
bool[] commandable_limbs
