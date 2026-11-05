"""List or switch the robot's services (the "Service Status" page of the phone app) over DDS.

Usage: python3 g1_service.py list
       python3 g1_service.py on  NAME      e.g. lidar_driver
       python3 g1_service.py off NAME
       add --dry to only check that the robot_state service listens (nothing is sent)

It sends ONE unitree_api request to /api/robot_state/request, the same call the app makes
(unitree_ros2 ros2_robot_state_client.h: api_id 1003 = ServiceList without parameter,
api_id 1001 = ServiceSwitch with {"name": NAME, "switch": 0|1}) and prints the response.
Needs the DDS environment of docs/01 and the unitree_api messages. `off` refuses the services
that keep the robot safe or reachable unless --force is given.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

API_SWITCH, API_LIST = 1001, 1003
KEEP_ON = {"robot_state", "emergency_stop", "battery_guard", "state_estimator", "ai_sport",
           "motion_switcher", "basic_service", "robot_type_service", "net_switcher"}

args = [a for a in sys.argv[1:] if not a.startswith("--")]
dry, force = "--dry" in sys.argv, "--force" in sys.argv
if not args or args[0] not in ("list", "on", "off") or (args[0] != "list" and len(args) != 2):
    sys.exit(__doc__)
cmd = args[0]
if cmd == "off" and args[1] in KEEP_ON and not force:
    sys.exit(f"refusing to switch off {args[1]!r} (safety/connectivity service); --force overrides")

rclpy.init()
node = Node("g1_service")
pub = node.create_publisher(Request, "/api/robot_state/request", 10)
responses = []
node.create_subscription(Response, "/api/robot_state/response", responses.append, 10)

t0 = time.time()
while time.time() - t0 < 6.0 and pub.get_subscription_count() == 0:
    rclpy.spin_once(node, timeout_sec=0.1)
for _ in range(10):  # let the response publisher be discovered too
    rclpy.spin_once(node, timeout_sec=0.1)
print(f"request-topic subscribers: {pub.get_subscription_count()}")
if pub.get_subscription_count() == 0:
    sys.exit("ABORT: nobody listens on /api/robot_state/request, nothing sent")
if dry:
    sys.exit("dry run: nothing sent")

req = Request()
req.header.identity.id = int(time.time() * 1000)
req.header.identity.api_id = API_LIST if cmd == "list" else API_SWITCH
req.header.lease.id = 0
req.header.policy.priority = 0
req.header.policy.noreply = False
req.parameter = "" if cmd == "list" else json.dumps({"name": args[1], "switch": 1 if cmd == "on" else 0})
print(f"sending api_id={req.header.identity.api_id} id={req.header.identity.id} parameter={req.parameter!r}")
pub.publish(req)

t0 = time.time()
mine = []
while time.time() - t0 < 12.0 and not mine:
    rclpy.spin_once(node, timeout_sec=0.1)
    mine = [r for r in responses if r.header.identity.id == req.header.identity.id]
if not mine:
    print(f"no matching response in 12 s (other responses seen: {len(responses)})")
else:
    r = mine[0]
    print(f"RESPONSE api_id={r.header.identity.api_id} status.code={r.header.status.code}")
    try:
        data = json.loads(r.data)
    except ValueError:
        data = None
    if cmd == "list" and isinstance(data, list):
        for s in sorted(data, key=lambda s: str(s.get("name"))):
            print(f"  {str(s.get('name')):<24} status={s.get('status')}  protect={s.get('protect')}")
    else:
        print(f"data={r.data}")
node.destroy_node()
rclpy.shutdown()
