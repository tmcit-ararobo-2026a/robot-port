uint8 START_ZONE=0
uint8 FIXED_BUCKET1=1
uint8 FIXED_BUCKET2=2
uint8 FIXED_BUCKET3=3
uint8 DESK1=4
uint8 DESK2=5
uint8 FLAG=6
uint8 MOVING_BUCKET=7
uint8 STANDBY=0
uint8 READY=1
uint8 MOVING=2
uint8 TRACKING=3
uint8 GOAL=4
uint8 FAIL=5
# Non-latched full snapshot from a single producer. Metadata is never sent on wire.
builtin_interfaces/Time stamp
# Changes on producer restart. First snapshot must be zero-speed Standby or Fail.
uint64 producer_session
uint64 generation
# Strictly increasing within one session, even for periodic publication.
uint64 update_sequence
uint8 target_id
uint8 navigation_status
float32 velocity_x_m_s
float32 velocity_y_m_s
float32 velocity_yaw_rad_s
float32 belt_launcher_speed_m_s
float32 bucket_arm_height_m
bool bucket_arm_width_extract
bool bucket_arm_hold
# Approved signed degree values, [-20,20]. No invalid-angle sentinel is defined.
int8[6] angles_deg
