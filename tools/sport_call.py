"""Talk to the G1's locomotion service (/api/sport) with ONE request per call.

Usage: python3 tools/sport_call.py get                  FSM id (7001), FSM mode (7002), stand height (7005); read-only
       python3 tools/sport_call.py watch SECONDS        print every FSM id change (polls 7001 at 10 Hz); read-only
       G1_ALLOW_MOTION=1 python3 tools/sport_call.py fsm 1|4|501    SetFsmId (7101); only these three ids
       G1_ALLOW_MOTION=1 python3 tools/sport_call.py damp           = fsm 1
       G1_ALLOW_MOTION=1 python3 tools/sport_call.py stop           SetVelocity 0 0 0
       G1_ALLOW_MOTION=1 python3 tools/sport_call.py vel VX VY W DURATION    SetVelocity (7105), capped:
                                                         |VX| <= 0.2 m/s, |VY| <= 0.1 m/s, |W| <= 0.3 rad/s, 0 < DURATION <= 1.0 s

Everything except `get` and `watch` can move the robot and is refused unless G1_ALLOW_MOTION=1 is set.
Needs the DDS environment of docs/01 (domain 0, CycloneDDS on the robot NIC) and the `unitree_api` messages.
Aborts without sending if nobody listens on /api/sport/request. Prints the round-trip time of every call.
Ids are from the official unitree_ros2 sources (g1_loco_client.hpp). 501 is the regular mode of the G1 with the
3-DoF waist (the 29-motor one); 500, which the SDK calls Start, is the 1-DoF-waist variant, see docs/08.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request, Response

CAP_VX, CAP_VY, CAP_W, CAP_T = 0.2, 0.1, 0.3, 1.0   # m/s, m/s, rad/s, s
FSM_IDS = ("1", "4", "501")
READ_ONLY = {7001, 7002, 7005}


class Client(Node):
    def __init__(self):
        super().__init__("sport_call")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Request, "/api/sport/request", qos)
        self.create_subscription(Response, "/api/sport/response", self._on_response, qos)
        self.answers = {}

    def _on_response(self, msg):
        self.answers[msg.header.identity.id] = (time.monotonic(), msg)

    def ready(self):
        return self.pub.get_subscription_count() >= 1 and self.count_publishers("/api/sport/response") >= 1

    def wait_ready(self, timeout=5.0):
        # both directions must be matched, or the first answer is lost
        t0 = time.monotonic()
        while not self.ready() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 0.5:   # let the endpoint matching settle
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.ready()

    def call(self, api_id, parameter="", timeout=5.0):
        """One request; returns (status code, data, round trip in ms), or (None, None, None) without an answer."""
        if api_id not in READ_ONLY and os.environ.get("G1_ALLOW_MOTION") != "1":
            sys.exit(f"api {api_id} can move the robot: refused (set G1_ALLOW_MOTION=1)")
        request_id = time.monotonic_ns()
        msg = Request()
        msg.header.identity.id = request_id
        msg.header.identity.api_id = api_id
        msg.parameter = parameter
        t_send = time.monotonic()
        self.pub.publish(msg)
        while request_id not in self.answers and time.monotonic() - t_send < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
        if request_id not in self.answers:
            return None, None, None
        t_answer, response = self.answers.pop(request_id)
        return response.header.status.code, response.data, (t_answer - t_send) * 1000


def build_calls(cmd, args):
    """[(api_id, parameter, label)] for a command; exits on bad arguments before ROS is touched."""
    if cmd == "get":
        return [(7001, "", "fsm_id"), (7002, "", "fsm_mode"), (7005, "", "stand_height")]
    if cmd == "damp":
        return [(7101, json.dumps({"data": 1}), "SetFsmId 1 (damping)")]
    if cmd == "fsm":
        if len(args) != 1 or args[0] not in FSM_IDS:
            sys.exit("usage: fsm " + "|".join(FSM_IDS))
        return [(7101, json.dumps({"data": int(args[0])}), f"SetFsmId {args[0]}")]
    if cmd == "stop":
        return [(7105, json.dumps({"velocity": [0.0, 0.0, 0.0], "duration": 1.0}), "SetVelocity 0")]
    if cmd == "vel":
        if len(args) != 4:
            sys.exit("usage: vel VX VY W DURATION")
        vx, vy, w, duration = (float(a) for a in args)
        if abs(vx) > CAP_VX or abs(vy) > CAP_VY or abs(w) > CAP_W or not 0 < duration <= CAP_T:
            sys.exit(f"over the caps: |vx|<={CAP_VX} |vy|<={CAP_VY} |w|<={CAP_W} 0<duration<={CAP_T}")
        return [(7105, json.dumps({"velocity": [vx, vy, w], "duration": duration}), f"SetVelocity {vx} {vy} {w} for {duration} s")]
    sys.exit(__doc__)


def watch(client, seconds):
    t_end, last = time.monotonic() + seconds, None
    while time.monotonic() < t_end:
        _, data, ms = client.call(7001)
        if data != last:
            print(time.strftime("%H:%M:%S"), f"fsm_id {data} (round-trip {ms:.0f} ms)" if ms is not None else "no answer", flush=True)
            last = data
        time.sleep(0.1)


def main():
    cmd, args = (sys.argv[1], sys.argv[2:]) if len(sys.argv) > 1 else ("", [])
    if cmd == "watch":
        if len(args) != 1:
            sys.exit("usage: watch SECONDS")
        seconds, calls = float(args[0]), []
    else:
        seconds = 0.0
        if cmd not in ("get", "damp", "fsm", "stop", "vel"):
            sys.exit(__doc__)
        if cmd != "get" and os.environ.get("G1_ALLOW_MOTION") != "1":
            sys.exit(f"'{cmd}' can move the robot: refused (set G1_ALLOW_MOTION=1)")
        calls = build_calls(cmd, args)
    rclpy.init()
    client = Client()
    if not client.wait_ready():
        sys.exit("ABORT: nobody answers on /api/sport (robot off, cable not up, or wrong DDS environment), nothing sent")
    if cmd == "watch":
        watch(client, seconds)
    for api_id, parameter, label in calls:
        code, data, ms = client.call(api_id, parameter)
        if code is None:
            print(f"{label}: NO ANSWER in 5 s")
        else:
            print(f"{label}: code={code} data={data!r} round-trip {ms:.0f} ms")
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
