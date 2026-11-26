# Per-thruster force command for the V4 ROV (8 x T500).
#
# Thruster order: force[i] is thruster_0(i+1) in
#   rov_description/config/thrusters_v4.yaml
#   [0..3] = thruster_01..04 (horizontal, +/-45 deg in the x-y plane)
#   [4..7] = thruster_05..08 (vertical, +z)
# Sign: positive = force on the vehicle along that thruster's `direction`
#   unit vector (base_link); negative = reverse thrust.

# header.stamp: time the command was computed.
# header.frame_id: base_link (frame of the thruster directions).
std_msgs/Header header

# N. Limits come from parameters (f_ctrl = 140 N, provisional),
# not from this message.
float64[8] force
