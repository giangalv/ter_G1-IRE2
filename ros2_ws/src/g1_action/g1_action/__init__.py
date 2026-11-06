"""g1_action: safety-critical realtime execution package for the G1 U6 VLA stack.

Nodes (regular rclpy Nodes, realtime CPU):

* ``action_executor_node``    -- 50 Hz receding-horizon ActionChunk execution,
  dual-path blending, five-stage safety pipeline, hand force control and
  navigation subtask velocity control.
* ``risk_estimator_node``     -- 10 Hz aggregate risk scoring driving the
  fast <-> precise dual-path dispatch.
* ``inspire_hand_bridge_node`` -- Inspire hand bridge (RH56E2 Modbus TCP or
  RH56DFQ serial) with hot-disconnect recovery and mock mode.

Core algorithms live in rclpy-free modules (``executor_core``, ``safety``,
``blending``, ``risk_core``, ``inspire_protocol``) so the unit tests run with
numpy only.
"""
