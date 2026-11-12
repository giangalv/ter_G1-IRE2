"""Passive event stream for a session with the Unitree remote. Publishes NOTHING.

Usage: python3 tools/remote_events.py [SECONDS] [--quiet]      (default 1200 s)

One line per event:
  KEYS ARRIVE A+B    the buttons that reach the robot, decoded from /lf/lowstate.wireless_remote (docs/08);
                     "keys released" when they go back to zero
  FSM old -> new     every change of the FSM id, read from the answers that somebody else's ~2 Hz poll of
                     7001 gets on /api/sport/response (nothing is sent, so the resolution is ~0.5 s)
  STICKS lx= ly= ... a stick beyond 0.5 (at most once per 3 s)
  LEGS |tau|= max|dq|=   once per second, only when a leg joint moves faster than 0.6 rad/s or the mean |tau_est|
                     of the 12 leg motors jumps by more than 1.5
--quiet keeps only the first two kinds (keys and FSM).
Needs the DDS environment of docs/01 and the `unitree_api` / `unitree_hg` messages.
"""
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowState

BUTTONS = ["R1", "L1", "START", "SELECT", "R2", "L2", "F1", "F2", "A", "B", "X", "Y", "UP", "RIGHT", "DOWN", "LEFT"]
args = [a for a in sys.argv[1:] if not a.startswith("--")]
duration = float(args[0]) if args else 1200.0
quiet = "--quiet" in sys.argv

rclpy.init()
node = Node("remote_events")
t0 = time.time()
pending = {}          # request id -> api_id
fsm = [None]
keys_last = [0]
stick_t = [0.0]
state_t = [0.0]
tau_last = [None]


def stamp():
    return time.strftime("%H:%M:%S")


def on_request(msg):
    pending[msg.header.identity.id] = msg.header.identity.api_id


def on_response(msg):
    if pending.pop(msg.header.identity.id, None) == 7001 and msg.data.strip() != fsm[0]:
        print(f"{stamp()} FSM {fsm[0]} -> {msg.data.strip()}", flush=True)
        fsm[0] = msg.data.strip()


def on_lowstate(msg):
    now = time.time()
    raw = bytes(msg.wireless_remote)
    keys = struct.unpack_from("<H", raw, 2)[0]
    lx, rx, ry, _l2, ly = struct.unpack_from("<5f", raw, 4)
    if keys != keys_last[0]:
        names = "+".join(BUTTONS[i] for i in range(16) if keys >> i & 1)
        print(f"{stamp()} KEYS ARRIVE {names}" if keys else f"{stamp()} keys released", flush=True)
        keys_last[0] = keys
    if quiet:
        return
    if max(abs(lx), abs(rx), abs(ry), abs(ly)) > 0.5 and now - stick_t[0] > 3.0:
        print(f"{stamp()} STICKS lx={lx:+.1f} ly={ly:+.1f} rx={rx:+.1f} ry={ry:+.1f}", flush=True)
        stick_t[0] = now
    if now - state_t[0] >= 1.0:
        state_t[0] = now
        motors = msg.motor_state
        tau = sum(abs(motors[i].tau_est) for i in range(12)) / 12
        dq = max(abs(motors[i].dq) for i in range(29))
        if dq > 0.6 or (tau_last[0] is not None and abs(tau - tau_last[0]) > 1.5):
            print(f"{stamp()} LEGS |tau|={tau:.1f} max|dq|={dq:.2f} "
                  f"rpy=({msg.imu_state.rpy[0]:+.2f},{msg.imu_state.rpy[1]:+.2f})", flush=True)
        tau_last[0] = tau


reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)
best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
node.create_subscription(Request, "/api/sport/request", on_request, reliable)
node.create_subscription(Response, "/api/sport/response", on_response, reliable)
node.create_subscription(LowState, "/lf/lowstate", on_lowstate, best_effort)
print(f"{stamp()} watching for {duration:.0f} s, publishing nothing", flush=True)
while time.time() - t0 < duration:
    rclpy.spin_once(node, timeout_sec=0.1)
print(f"{stamp()} done", flush=True)
rclpy.shutdown()
