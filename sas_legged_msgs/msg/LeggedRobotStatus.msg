# Current state of a legged robot driver. Published on <prefix>/get/status.
std_msgs/Header header

# One of the HighLevelMode constants.
uint8 mode

# One entry per joint of get/joint_states (same order). Entries that are false are ignored
# in set/target_joint_positions. The mask can change with the mode.
bool[] commandable_joints

# What the driver accepts in the current mode.
bool accepts_twist
bool accepts_base_orientation
bool accepts_base_height
bool accepts_manipulator_commands
