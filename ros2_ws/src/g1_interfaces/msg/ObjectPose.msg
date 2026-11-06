# Grounded 3D object pose with open-vocabulary embedding for DynaMem lookup.

geometry_msgs/PoseStamped pose
string label
float32 confidence
float64 last_seen             # epoch seconds of last observation
float64[] siglip_embedding    # 768-dim SigLIP feature (empty if not computed)
