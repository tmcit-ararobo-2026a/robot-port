builtin_interfaces/Time stamp
# Changes on bridge restart; advancing reports preserve each input age.
uint64 bridge_session
uint64 update_sequence
bool test_mode
bool loopback_only
bool command_fresh
bool feedback_fresh
bool operation_fresh
bool transmission_enabled
bool last_send_success
bool watchdog_active
# Monotonic elapsed times; -1 means never received/sent.
float64 command_age_ms
float64 feedback_age_ms
float64 operation_age_ms
float64 last_successful_send_age_ms
uint64 rejected_command_packets
uint64 rejected_feedback_packets
uint64 rejected_operations
uint64 receive_errors
uint64 send_errors
# UDP send success does not imply board acceptance or any mechanism stopping.
string reason
