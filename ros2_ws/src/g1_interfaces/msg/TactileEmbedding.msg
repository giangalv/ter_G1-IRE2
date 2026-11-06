# SPARSH tactile embedding for FiLM conditioning of the VLA backbone.
# Published by tactile_encoder_node @ 100 Hz on /g1/tactile/embedding.

std_msgs/Header header
float32[] embedding          # 256-dim (128 per hand, left then right, concatenated)
bool contact_active          # gate signal: True only when any phalanx > contact threshold
float32 mean_pressure        # scalar summary for diagnostics (N)
float32[] per_finger_force   # 10 values (5 left fingers then 5 right), Newtons
