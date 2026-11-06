# Per-finger contact interpretation of the raw tactile arrays.
# Published by tactile_encoder_node @ 100 Hz on /g1/tactile/contact_state.
# Finger order: 5 left-hand fingers (little..thumb) then 5 right-hand fingers.

std_msgs/Header header
bool[] finger_contact     # 10 booleans - which fingers are in contact
float32[] normal_force    # 10 values, estimated normal force (N)
float32[] shear_force     # 10 values, estimated shear/slip force (N)
bool grasp_stable         # True if all contacted fingers show stable pressure
