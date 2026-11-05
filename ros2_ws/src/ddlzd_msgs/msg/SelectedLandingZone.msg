uint8 NONE=0
uint8 LOCKED=1
uint8 COASTING=2

uint8 UNKNOWN=0
uint8 SAFEST=1
uint8 SAFE=2
uint8 RISKY=3

std_msgs/Header header
uint64 generation
uint64 id
uint8 selection_state
bool selection_valid
bool gnss_valid
string reason
string sensor_model

geometry_msgs/Pose local_pose
geometry_msgs/Vector3 vehicle_relative_enu

float64 latitude
float64 longitude
float64 gnss_horizontal_uncertainty_m

float32 radius
float32 risk_score
uint8 category
uint32 consecutive_observations
