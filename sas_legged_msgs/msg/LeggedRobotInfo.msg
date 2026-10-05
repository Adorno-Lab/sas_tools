# Static description of a legged robot driver. Published once on <prefix>/get/info (transient_local).
string robot_model

# Names of every joint of get/joint_states, in the same order.
string[] joint_names

# HighLevelMode constants the robot supports.
uint8[] supported_modes

bool supports_twist
bool supports_base_height
bool supports_base_orientation
bool supports_manipulation_while_walking
