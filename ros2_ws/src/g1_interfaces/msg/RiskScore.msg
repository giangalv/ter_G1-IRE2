# Aggregate risk estimate published by risk_estimator_node @ 10 Hz.
# Drives fast <-> precise dual-path dispatch in vla_inference_node.

std_msgs/Header header
float32 collision_proximity   # sigmoid of min(depth, lidar) distance in arm workspace
float32 grasp_uncertainty     # 1 - top1 SigLIP similarity from DynaMem query
float32 action_entropy        # mirrored from ActionChunk.action_entropy
float32 tactile_anomaly       # spike in tactile pressure vs expected (slip / surprise contact)
bool trigger_precise          # dispatch decision: request the precise (DDIM) path
