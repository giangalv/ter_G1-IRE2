# Hierarchical subtask token from subtask_planner_node (@ 0.4 Hz).
# Vocabulary: navigate_to, approach, grasp, grasp_bimanual, grasp_precision,
#             handover_object, place, done  (U6 adds grasp_precision / handover_object)

string text          # e.g. "grasp_precision red mug"
float32 completion   # [0..1] estimated completion of this subtask
float64 issued_at    # epoch seconds
