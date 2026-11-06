# One episodic memory record (ReMEmbR tier). Written by remembr_writer_node
# every ~3 s, retrieved by remembr_reader_node via QueryMemory.

string caption                 # VILA-generated scene caption
geometry_msgs/Point position   # robot base position in `world` at capture time
float64 timestamp              # epoch seconds
float64[] embedding            # 1024-dim mxbai-embed-large-v1 text embedding
string task_name               # active ExecuteTask instruction (may be empty)
bool tactile_event             # True if a stable-grasp tactile pattern was active
                               # (enables "last successful grasp" recovery queries)
