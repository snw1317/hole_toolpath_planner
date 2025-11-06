# hole_toolpath_planner/Hole.msg
# Pose at entry rim center; +Z points into the part.
# kind identifies which pipeline produced the detection.

uint8 SURFACE_CIRCLE=0
uint8 CYLINDER=1

std_msgs/Header header
geometry_msgs/Pose pose
float32 diameter
float32 length
geometry_msgs/Vector3 axis
int32 id
uint8 kind
