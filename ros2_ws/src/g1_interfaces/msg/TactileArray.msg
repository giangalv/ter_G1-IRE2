# Raw tactile frame from one Inspire RH56DFQ hand (published @ 100 Hz by
# inspire_hand_bridge_node). Per-phalanx pressure: 5 fingers x 3 phalanges.

std_msgs/Header header
string hand_side          # "left" | "right"
float32[] pressure        # 15 values (finger-major: f1p1 f1p2 f1p3 f2p1 ...), Newtons
float32[] contact_normal  # 45 values: 15 x 3 unit normal vectors, row-major
