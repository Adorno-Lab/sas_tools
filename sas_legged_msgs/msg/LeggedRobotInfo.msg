# Static description of a legged robot driver. Published once on <prefix>/get/info (transient_local).

# Names of every joint of get/joint_states, in the same order. A client can compare them with
# its kinematic model to check that it is connected to the robot it expects.
string[] joint_names

# HighLevelMode constants the robot supports.
uint8[] supported_modes

bool supports_twist
bool supports_base_height
bool supports_base_orientation
bool supports_manipulation_while_walking

# Limits of set/target_base_orientation, in radians, as ZYX roll-pitch-yaw angles relative
# to the frame F_f (see set/target_base_orientation). Each angle is clamped to [min, max];
# the limits do not need to be symmetric. Zero when supports_base_orientation is false.
float64 min_base_roll
float64 max_base_roll
float64 min_base_pitch
float64 max_base_pitch
float64 min_base_yaw
float64 max_base_yaw
