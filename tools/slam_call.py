"""Send ONE unitree_api Request to /api/slam_operate and print the response.

Usage: python3 slam_call.py API_ID 'JSON-PARAMETER' [--dry]
Aborts without sending if nobody is subscribed to the request topic.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

api_id = int(sys.argv[1])
parameter = sys.argv[2]
dry = "--dry" in sys.argv

rclpy.init()
node = Node("slam_call")
pub = node.create_publisher(Request, "/api/slam_operate/request", 10)
responses = []
node.create_subscription(Response, "/api/slam_operate/response", responses.append, 10)

t0 = time.time()
while time.time() - t0 < 6.0 and pub.get_subscription_count() == 0:
    rclpy.spin_once(node, timeout_sec=0.1)
for _ in range(10):  # let the response publisher get discovered too
    rclpy.spin_once(node, timeout_sec=0.1)
print(f"request-topic subscribers: {pub.get_subscription_count()}")
if pub.get_subscription_count() == 0:
    print("ABORT: nobody listens on /api/slam_operate/request, nothing sent")
    sys.exit(2)
if dry:
    print("dry run: nothing sent")
    sys.exit(0)

req = Request()
req.header.identity.id = int(time.time() * 1000)
req.header.identity.api_id = api_id
req.header.lease.id = 0
req.header.policy.priority = 0
req.header.policy.noreply = False
req.parameter = parameter
print(f"sending api_id={api_id} id={req.header.identity.id} parameter={parameter}")
pub.publish(req)

t0 = time.time()
while time.time() - t0 < 12.0:
    rclpy.spin_once(node, timeout_sec=0.1)
    mine = [r for r in responses if r.header.identity.id == req.header.identity.id]
    if mine:
        r = mine[0]
        print(f"RESPONSE api_id={r.header.identity.api_id} status.code={r.header.status.code}")
        print(f"data={r.data}")
        break
else:
    print(f"no matching response in 12 s (other responses seen: {len(responses)})")
node.destroy_node()
rclpy.shutdown()
