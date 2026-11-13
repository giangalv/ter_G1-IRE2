#!/usr/bin/env python3
"""Keyboard teleop of the G1 through the locomotion service (/api/sport, SetVelocity 7105).

Run it in a real terminal (it reads single keys, so it cannot run from a pipe):

    source tools/g1_env.sh cable            # or wifi, see docs/08
    G1_ALLOW_MOTION=1 python3 tools/keyboard_teleop.py [--dry] [--vx 0.2] [--vy 0.1] [--w 0.3] [--hold 0.2] [--first-hold 0.6]

Keys:
    r        ARM (it starts disarmed: nothing is sent until you press r)
    w / s    forward / backward          q / e   turn left / right
    a / d    step left / right
    1 2 3    speed 25 / 50 / 100 % of the caps (starts at 50 %)
    Enter    STOP: zero velocity and disarm (space does the same)
    x        HOLD x FOR 3 SECONDS = DAMP (SetFsmId 1): the robot goes limp at once (and falls if it is not held).
             A tap or a stray x does nothing; the status line counts the seconds; releasing x cancels.
    Ctrl-C   zero velocity and quit (the robot stays standing in regular mode)

Dead-man: a motion key acts only while the terminal keeps repeating it. After the FIRST press of a key the command lasts
--first-hold seconds (default 0.6: it bridges the terminal's key-repeat delay of about 0.5 s, and the gait needs a
command of about a second to start, measured in docs/08); once the repeats arrive each one keeps it alive only --hold
seconds (default 0.2), so on release the robot stops within 0.2 s. A single tap is therefore a 0.6 s command. Every
request carries its own 0.3 s duration, so if this program, the cable or the Wi-Fi dies the robot stops by itself
within 0.3 s.

Nothing is sent, and you cannot arm, unless ALL of these hold: FSM id = 501 (regular mode, answered by the robot, not
older than 2 s), /lf/lowstate arrives (not older than 0.5 s), |roll| and |pitch| < 0.35 rad. A guard that trips disarms
(you must press r again), and so do three unanswered velocity requests and any answer with a code other than 0.
The remote stays the person's backup (L2+B, ~5 s in a walking state). Caps: defaults 0.2 m/s forward, 0.1 m/s
sideways, 0.3 rad/s; the hard ceiling is 0.4 / 0.2 / 0.6. --dry reads and shows everything and sends nothing but the
FSM poll (7001).
"""
import argparse
import json
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowState

API_FSM_GET, API_FSM_SET, API_VEL = 7001, 7101, 7105
MOVE_FSM = 501                 # regular mode of the 29-motor G1 (docs/08)
DEFAULT_CAPS = (0.2, 0.1, 0.3)  # m/s forward, m/s sideways, rad/s
HARD_CAPS = (0.4, 0.2, 0.6)
SEND_PERIOD = 0.1              # s between velocity requests while a key is held
DAMP_HOLD = 3.0                # s the x key must be held for DAMP
X_GAP, X_FAST_GAP, X_MIN_FAST = 0.7, 0.2, 20   # largest gap in a hold (the first one is the repeat delay), repeat-phase gap, repeats needed
REPEAT_GAP = 0.15             # s: the same key again within this gap counts as the terminal's key repeat
CMD_DURATION = 0.3             # s, the robot's own timeout of every velocity request
FSM_STALE_S, LOW_STALE_S, TILT_LIMIT = 2.0, 0.5, 0.35
SPEED = {"1": 0.25, "2": 0.5, "3": 1.0}
MOVES = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0), "q": (0, 0, 1), "e": (0, 0, -1)}


class Teleop(Node):
    def __init__(self, dry):
        super().__init__("keyboard_teleop")
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)
        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Request, "/api/sport/request", reliable)
        self.create_subscription(Response, "/api/sport/response", self._on_response, reliable)
        self.create_subscription(LowState, "/lf/lowstate", self._on_low, best_effort)
        self.dry = dry
        self.lock = threading.Lock()
        self.pending = {}            # request id -> (time sent, api id)
        self._last_id = 0
        self.fsm, self.fsm_t = None, 0.0
        self.low_t, self.rpy = None, (0.0, 0.0)
        self.sent = self.acked = self.missed = 0
        self.last_ms = None
        self.errors = []
        self.vel_sent = False

    def _on_low(self, msg):
        self.rpy = (msg.imu_state.rpy[0], msg.imu_state.rpy[1])
        self.low_t = time.monotonic()

    def _on_response(self, msg):
        now = time.monotonic()
        with self.lock:
            sent = self.pending.pop(msg.header.identity.id, None)
        if sent is None:                       # somebody else's answer
            return
        t_sent, api = sent
        self.last_ms = (now - t_sent) * 1000
        code = msg.header.status.code
        if api == API_FSM_GET:
            try:
                self.fsm, self.fsm_t = int(json.loads(msg.data)["data"]), now
            except (ValueError, KeyError, TypeError):
                self.errors.append(f"unreadable FSM answer {msg.data!r}")
            return
        if code != 0:
            self.errors.append(f"api {api}: code {code} {msg.data!r}")
        elif api == API_VEL:
            self.acked += 1
            self.missed = 0

    def send(self, api_id, parameter=""):
        if self.dry and api_id != API_FSM_GET:
            return
        with self.lock:
            self._last_id = max(self._last_id + 1, time.monotonic_ns())
            rid = self._last_id
            self.pending[rid] = (time.monotonic(), api_id)
        msg = Request()
        msg.header.identity.id = rid
        msg.header.identity.api_id = api_id
        msg.parameter = parameter
        self.pub.publish(msg)
        if api_id == API_VEL:
            self.sent += 1
            self.vel_sent = True

    def poll_fsm(self):
        self.send(API_FSM_GET)

    def velocity(self, vx, vy, w, duration=CMD_DURATION):
        self.send(API_VEL, json.dumps({"velocity": [round(vx, 3), round(vy, 3), round(w, 3)], "duration": duration}))

    def damp(self):
        self.send(API_FSM_SET, json.dumps({"data": 1}))

    def expire(self, now, timeout=1.0):
        """Forget requests that were never answered; returns how many velocity requests were lost."""
        with self.lock:
            old = [(rid, api) for rid, (t, api) in self.pending.items() if now - t > timeout]
            for rid, _ in old:
                del self.pending[rid]
        lost = sum(1 for _, api in old if api == API_VEL)
        self.missed += lost
        return lost

    def blocked(self, now):
        """Why no velocity may be sent right now, or None."""
        if self.fsm is None or now - self.fsm_t > FSM_STALE_S:
            return "FSM id unknown (no answer from /api/sport)"
        if self.fsm != MOVE_FSM:
            return f"FSM {self.fsm}: velocity only in {MOVE_FSM}"
        if self.low_t is None or now - self.low_t > LOW_STALE_S:
            return "no /lf/lowstate (link lost?)"
        if max(abs(self.rpy[0]), abs(self.rpy[1])) > TILT_LIMIT:
            return f"TILT roll {self.rpy[0]:+.2f} pitch {self.rpy[1]:+.2f} rad"
        return None


def say(text):
    sys.stdout.write("\r\x1b[K" + text + "\n")
    sys.stdout.flush()


def draw(node, armed, want, level, now, damp_progress=""):
    fsm = str(node.fsm) if node.fsm is not None and now - node.fsm_t <= FSM_STALE_S else "?"
    low = f"{(now - node.low_t) * 1000:.0f} ms" if node.low_t is not None else "none"
    ack = f"{node.last_ms:.0f} ms" if node.last_ms is not None else "-"
    mode = "DRY" if node.dry else ("ARMED" if armed else "disarmed")
    if damp_progress:
        mode += f" DAMP {damp_progress}"
    sys.stdout.write(f"\r\x1b[K[{mode}] FSM {fsm} | v {want[0]:+.2f} {want[1]:+.2f} {want[2]:+.2f} | speed {int(SPEED[level] * 100)}%"
                     f" | sent {node.sent} ack {node.acked} ({ack}) | rpy {node.rpy[0]:+.2f} {node.rpy[1]:+.2f} | low {low}")
    sys.stdout.flush()


def loop(node, caps, hold, first_hold, fd):
    level, armed, moving = "2", False, False
    want = (0.0, 0.0, 0.0)
    hold_until = last_send = last_poll = last_draw = last_damp = 0.0
    damp_tries = 0
    x_start, x_last, x_fast, x_fired = None, 0.0, 0, False
    last_key, last_key_t = None, 0.0

    def stop_if_walking():
        if node.vel_sent and node.fsm == MOVE_FSM:
            node.velocity(0, 0, 0, 1.0)

    def disarm(reason):
        nonlocal armed, moving, want
        if armed:
            say(f"DISARMED: {reason}")
        armed = moving = False
        want = (0.0, 0.0, 0.0)

    while True:
        now = time.monotonic()
        if select.select([fd], [], [], 0.02)[0]:
            for ch in os.read(fd, 64).decode(errors="ignore"):
                now = time.monotonic()
                if ch in " \r\n":                  # space or Enter: STOP (a held key repeats: act only while armed)
                    if armed:
                        stop_if_walking()
                        disarm("Enter" if ch != " " else "space")
                elif ch == "x":
                    # DAMP = x HELD for DAMP_HOLD s: the terminal's repeat must be continuous (gaps <= X_GAP, and a
                    # fast repeat phase of at least X_MIN_FAST events), so taps and a stray x do nothing
                    if x_start is None or now - x_last > X_GAP:
                        x_start, x_fast, x_fired = now, 0, False
                        say(f"x pressed: hold it {DAMP_HOLD:.0f} s to DAMP (the robot goes limp), release to cancel; Enter stops")
                    elif now - x_last <= X_FAST_GAP:
                        x_fast += 1
                    x_last = now
                    if not x_fired and now - x_start >= DAMP_HOLD and x_fast >= X_MIN_FAST:
                        x_fired = True                 # one request per hold; release x to start over
                        damp_tries, last_damp = 5, 0.0
                        disarm("x held = DAMP")
                        say("DAMP requested" + (" (dry: nothing is sent)" if node.dry else ""))
                elif ch == "r":
                    why = node.blocked(now)
                    if why:
                        say(f"cannot arm: {why}")
                    elif not armed:
                        armed = True
                        say("ARMED: hold w/s/a/d/q/e, Enter or space = stop, hold x 3 s = damp")
                elif ch in SPEED:
                    level = ch
                elif ch in MOVES and armed:
                    k = SPEED[level]
                    want = tuple(m * c * k for m, c in zip(MOVES[ch], caps))
                    repeating = ch == last_key and now - last_key_t < REPEAT_GAP
                    hold_until = now + (hold if repeating else first_hold)
                    last_key, last_key_t = ch, now
        now = time.monotonic()
        if now - last_poll >= (0.2 if damp_tries else 0.5):
            node.poll_fsm()
            last_poll = now
        if damp_tries and now - last_damp >= 0.3:
            if node.fsm == 1 and now - node.fsm_t < 1.0:
                damp_tries = 0
                say("FSM 1 (damping) confirmed")
            else:
                node.damp()
                damp_tries -= 1
                last_damp = now
        while node.errors:
            disarm("the robot answered: " + node.errors.pop(0))
        if node.expire(now) and node.missed >= 3:
            disarm(f"{node.missed} velocity requests without an answer")
        why = node.blocked(now)
        if armed and why:
            stop_if_walking()
            disarm(why)
        if armed and moving and now >= hold_until:
            stop_if_walking()
            moving, want = False, (0.0, 0.0, 0.0)
        elif armed and now < hold_until and now - last_send >= SEND_PERIOD:
            node.velocity(*want)
            moving, last_send = True, now
        if now - last_draw >= 0.2:
            draw(node, armed, want, level, now,
                 f"x {now - x_start:.1f}/{DAMP_HOLD:.0f} s" if x_start is not None and not x_fired and now - x_last <= X_GAP else "")
            last_draw = now


def main():
    ap = argparse.ArgumentParser(description="Keyboard teleop of the G1 (/api/sport). Keys in the module docstring.")
    ap.add_argument("--vx", type=float, default=DEFAULT_CAPS[0], help="forward/back cap, m/s (hard max 0.4)")
    ap.add_argument("--vy", type=float, default=DEFAULT_CAPS[1], help="sideways cap, m/s (hard max 0.2)")
    ap.add_argument("--w", type=float, default=DEFAULT_CAPS[2], help="turning cap, rad/s (hard max 0.6)")
    ap.add_argument("--hold", type=float, default=0.2, help="seconds each key REPEAT keeps the command alive (0.05-0.8)")
    ap.add_argument("--first-hold", type=float, default=0.6, help="seconds the FIRST press of a key keeps it alive (0.05-0.8)")
    ap.add_argument("--dry", action="store_true", help="show everything, send only the FSM poll (7001)")
    args = ap.parse_args()
    caps = (args.vx, args.vy, args.w)
    if any(not 0 < c <= h for c, h in zip(caps, HARD_CAPS)) or not 0.05 <= args.hold <= 0.8 or not 0.05 <= args.first_hold <= 0.8:
        sys.exit(f"caps must be in (0, {HARD_CAPS}] and --hold / --first-hold in 0.05..0.8")
    if not args.dry and os.environ.get("G1_ALLOW_MOTION") != "1":
        sys.exit("this can move the robot: refused (set G1_ALLOW_MOTION=1, or use --dry)")
    if not sys.stdin.isatty():
        sys.exit("needs a terminal: it reads single keys")

    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)   # Ctrl-C must not shut ROS down before the final stop
    node = Teleop(args.dry)

    def spin():
        try:
            rclpy.spin(node)
        except Exception:        # the context was shut down at exit
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    t0 = time.monotonic()
    while not (node.pub.get_subscription_count() >= 1 and node.count_publishers("/api/sport/response") >= 1):
        if time.monotonic() - t0 > 5.0:
            sys.exit("ABORT: nobody answers on /api/sport (robot off, cable/Wi-Fi not up, or wrong DDS environment), nothing sent")
        time.sleep(0.1)
    time.sleep(0.5)                                   # let the endpoint matching settle

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(0))
    print(__doc__.split("Keys:")[1].split("Dead-man")[0].rstrip() + "\n")
    try:
        tty.setcbreak(fd)
        loop(node, caps, args.hold, args.first_hold, fd)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        if node.vel_sent and not node.dry and node.fsm in (None, MOVE_FSM):
            for _ in range(3):
                node.velocity(0, 0, 0, 1.0)
                time.sleep(0.05)
        print(f"\nexit: sent {node.sent} velocity requests, {node.acked} answered, FSM {node.fsm}")
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == "__main__":
    main()
