#!/usr/bin/env python3
"""Keyboard teleop of the G1 through the locomotion service (/api/sport: SetVelocity 7105, SetFsmId 7101).

Run it in a real terminal (it reads single keys, so it cannot run from a pipe):

    source tools/g1_env.sh cable            # or wifi, see docs/08
    G1_ALLOW_MOTION=1 python3 tools/keyboard_teleop.py [--dry] [--vx 0.2] [--vy 0.1] [--w 0.3] [--hold 0.2] [--first-hold 0.6]

Keys:
    r        ARM (it starts disarmed: nothing is sent until you press r)
    w / s    forward / backward          q / e   turn left / right
    a / d    step left / right
    1 2 3    speed 25 / 50 / 100 % of the caps (starts at 50 %)
    Enter    STOP: zero velocity and disarm (space does the same)
    z p l g  HOLD THE KEY FOR 3 SECONDS = change the robot's state, as the remote's L2+Y / L2+B / L2+Up / R1+Y:
               z  ZERO TORQUE      (SetFsmId 0)    only from damping: all motors off, the robot falls if it is not held
               p  DAMPING          (SetFsmId 1)    from any state: the robot goes limp at once (and falls if it is not held)
               l  LOCKED STANDING  (SetFsmId 4)    only from damping: the legs take the weight and hold the robot up
               g  REGULAR MODE     (SetFsmId 501)  only from locked standing: walking-ready, then r arms the teleop
             so the way up is p, l, g and the way down is p (then z for zero torque). A tap, a stray key, another key in
             between or releasing early does nothing; the status line counts the seconds. A key that does not fit the
             robot's present state says so when pressed and does not count.
    Ctrl-C   zero velocity and quit (the robot stays in the state it is in)

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
import shutil
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
FSM_HOLD = 3.0                 # s a state key (z p l g) must be held
HOLD_GAP, HOLD_FAST_GAP, HOLD_MIN_FAST = 0.7, 0.2, 20   # largest gap in a hold (the first one is the repeat delay), repeat-phase gap, repeats needed
FSM_WAIT_S = 6.0               # s to wait for the robot to report the new FSM id before saying it did not change
# key -> (FSM id to set, what it is called, FSM ids it may start from (None: any, it is the emergency), effect).
# The "from" sets are the remote's own order (docs/08): zero torque <- damping -> locked stand -> regular mode.
FSM_KEYS = {
    "z": (0, "ZERO TORQUE", {1}, "all motors off, the robot falls if it is not held"),
    "p": (1, "DAMPING", None, "the robot goes limp at once, and falls if it is not held"),
    "l": (4, "LOCKED STANDING", {1}, "the legs take the weight and hold the robot up"),
    "g": (501, "REGULAR MODE", {4}, "walking-ready: the legs move for a moment, then r arms the teleop"),
}
STATE_NAMES = {0: "zero torque", 1: "damping", 4: "locked standing", 501: "regular mode"}
KEY_OF_STATE = {v[0]: k for k, v in FSM_KEYS.items()}
REPEAT_GAP = 0.15             # s: the same key again within this gap counts as the terminal's key repeat
CMD_DURATION = 0.3             # s, the robot's own timeout of every velocity request
FSM_STALE_S, LOW_STALE_S, TILT_LIMIT = 2.0, 0.5, 0.35
REMOTE_STALE_S = 2.0           # s: the Unitree remote counts as ON if its payload in /lf/lowstate was non-zero this recently
REMOTE_WAIT_S = 4.0            # s the start waits to see the remote before it refuses to run
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
        self.remote_t = None         # when /lf/lowstate.wireless_remote last had a non-zero byte (the remote is on and linked)
        self.sent = self.acked = self.missed = 0
        self.last_ms = None
        self.errors = []
        self.vel_sent = False

    def _on_low(self, msg):
        self.rpy = (msg.imu_state.rpy[0], msg.imu_state.rpy[1])
        self.low_t = time.monotonic()
        if any(msg.wireless_remote):     # all zero = no frame of the remote reaches the robot (remote off or not linked)
            self.remote_t = self.low_t

    def remote_alive(self, now):
        return self.remote_t is not None and now - self.remote_t <= REMOTE_STALE_S

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

    def set_fsm(self, target):
        self.send(API_FSM_SET, json.dumps({"data": target}))

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
        if not self.dry and not self.remote_alive(now):
            return "the Unitree remote is not detected: switch it on (it is the backup, L2+B damps the robot)"
        if max(abs(self.rpy[0]), abs(self.rpy[1])) > TILT_LIMIT:
            return f"TILT roll {self.rpy[0]:+.2f} pitch {self.rpy[1]:+.2f} rad"
        return None


class Hold:
    """Tells a key that is HELD from one that was tapped or struck by mistake, from the characters the terminal sends.

    A hold is continuous when no gap between two characters exceeds HOLD_GAP (the first gap is the terminal's key-repeat
    delay, about 0.5 s) and it includes at least HOLD_MIN_FAST fast repeats; it counts from its first character."""

    def __init__(self, seconds=FSM_HOLD):
        self.seconds = seconds
        self.reset()

    def reset(self):
        self.start, self.last, self.fast, self.fired = None, 0.0, 0, False

    def press(self, now):
        """One character of the key arrived. Returns "start" (a new hold begins), "fire" (held long enough, once per
        hold: release the key to start over) or None."""
        event = None
        if self.start is None or now - self.last > HOLD_GAP:
            self.start, self.fast, self.fired = now, 0, False
            event = "start"
        elif now - self.last <= HOLD_FAST_GAP:
            self.fast += 1
        self.last = now
        if not self.fired and now - self.start >= self.seconds and self.fast >= HOLD_MIN_FAST:
            self.fired = True
            event = "fire"
        return event

    def progress(self, now):
        """Seconds held so far while a hold is in progress and has not fired, else None."""
        if self.start is not None and not self.fired and now - self.last <= HOLD_GAP:
            return now - self.start
        return None


def fsm_refusal(node, key, now):
    """Why the state key cannot be used now, or None. Damp is always allowed (it is the emergency); the others only
    from the state the remote uses, and only while the robot's FSM id is known."""
    target, _name, sources, _effect = FSM_KEYS[key]
    if sources is None:
        return None
    if node.fsm is None or now - node.fsm_t > FSM_STALE_S:
        return "FSM id unknown (no answer from /api/sport)"
    if node.fsm == target:
        return f"already in FSM {target} ({STATE_NAMES[target]})"
    if node.fsm not in sources:
        need = min(sources)
        return (f"{FSM_KEYS[key][1]} starts from FSM {need} ({STATE_NAMES[need]}): hold {KEY_OF_STATE[need]} first; "
                f"the robot is in FSM {node.fsm}" + (f" ({STATE_NAMES[node.fsm]})" if node.fsm in STATE_NAMES else ""))
    return None


def say(text):
    sys.stdout.write("\r\x1b[K" + text + "\n")
    sys.stdout.flush()


def draw(node, armed, want, level, now, hold_label=""):
    fsm = str(node.fsm) if node.fsm is not None and now - node.fsm_t <= FSM_STALE_S else "?"
    low = f"{(now - node.low_t) * 1000:.0f} ms" if node.low_t is not None else "none"
    ack = f"{node.last_ms:.0f} ms" if node.last_ms is not None else "-"
    mode = "DRY" if node.dry else ("ARMED  Enter/Space=STOP" if armed else "disarmed")   # first on the line: never cut by the width
    if hold_label:
        mode += f" {hold_label}"
    text = (f"[{mode}] FSM {fsm} | v {want[0]:+.2f} {want[1]:+.2f} {want[2]:+.2f} | speed {int(SPEED[level] * 100)}%"
            f" | sent {node.sent} ack {node.acked} ({ack}) | rpy {node.rpy[0]:+.2f} {node.rpy[1]:+.2f} | low {low}")
    # one terminal row only: a longer line wraps, and \r then returns to the last wrapped row, so every redraw
    # would scroll a new line instead of updating in place
    cols = shutil.get_terminal_size((120, 24)).columns
    sys.stdout.write("\r\x1b[K" + text[:max(cols - 1, 20)])
    sys.stdout.flush()


def loop(node, caps, hold, first_hold, fd):
    level, armed, moving = "2", False, False
    want = (0.0, 0.0, 0.0)
    hold_until = last_send = last_poll = last_draw = 0.0
    holds = {k: Hold() for k in FSM_KEYS}
    goal = None                  # an FSM change being carried out: target id, requests left, last request, start
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
                for k, h in holds.items():         # any other key cancels a hold in progress
                    if k != ch:
                        h.reset()
                if ch in " \r\n":                  # space or Enter: STOP (a held key repeats: act only while armed)
                    if armed:
                        stop_if_walking()
                        disarm("Enter" if ch != " " else "space")
                elif ch in FSM_KEYS:
                    # a state key must be HELD for FSM_HOLD s: the terminal's repeat has to be continuous, so taps and a
                    # stray key do nothing (see Hold). The key must also fit the robot's present state (fsm_refusal).
                    target, name, _, effect = FSM_KEYS[ch]
                    event = holds[ch].press(now)
                    if event:
                        refusal = fsm_refusal(node, ch, now)
                        if refusal:
                            holds[ch].fired = True         # nothing to count: release the key, fix the state, press again
                            say(f"{ch}: {refusal}")
                        elif event == "start":
                            say(f"{ch} pressed: hold it {FSM_HOLD:.0f} s for {name} ({effect}); release to cancel; Enter or space stops")
                        else:
                            disarm(f"{ch} held = {name}")
                            say(f"{name} requested" + (" (dry: nothing is sent)" if node.dry else ""))
                            if not node.dry:
                                goal = {"target": target, "tries": 5 if target == 1 else 1, "last": 0.0, "t0": now}
                elif ch == "r":
                    why = node.blocked(now)
                    if why:
                        say(f"cannot arm: {why}")
                    elif not armed:
                        armed = True
                        say("ARMED: hold w/s/a/d/q/e;  Enter or Space = STOP;  hold p 3 s = damping")
                elif ch in SPEED:
                    level = ch
                elif ch in MOVES and armed:
                    k = SPEED[level]
                    want = tuple(m * c * k for m, c in zip(MOVES[ch], caps))
                    repeating = ch == last_key and now - last_key_t < REPEAT_GAP
                    hold_until = now + (hold if repeating else first_hold)
                    last_key, last_key_t = ch, now
        now = time.monotonic()
        if now - last_poll >= (0.2 if goal else 0.5):
            node.poll_fsm()
            last_poll = now
        if goal:
            target = goal["target"]
            if node.fsm == target and now - node.fsm_t < 1.0:
                say(f"FSM {target} ({STATE_NAMES[target]}) confirmed")
                goal = None
            elif goal["tries"] and now - goal["last"] >= 0.3:     # damping is repeated (5 x), the other states are sent once
                node.set_fsm(target)
                goal["tries"] -= 1
                goal["last"] = now
            elif not goal["tries"] and now - goal["t0"] > FSM_WAIT_S:
                say(f"FSM did not become {target} ({STATE_NAMES[target]}) within {FSM_WAIT_S:.0f} s (it is {node.fsm}): nothing more is sent")
                goal = None
        while node.errors:                         # an error answer is shown in every state, and disarms if armed
            err = "the robot answered: " + node.errors.pop(0)
            if armed:
                disarm(err)
            else:
                say(err)
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
            label = ""
            for k, h in holds.items():
                p = h.progress(now)
                if p is not None:
                    label = f"{FSM_KEYS[k][1]} {k} {p:.1f}/{FSM_HOLD:.0f} s"
            draw(node, armed, want, level, now, label)
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

    def abort(message):
        """Leave before the keyboard is touched: stop ROS first, or the spin thread aborts the process at exit."""
        rclpy.shutdown()
        spinner.join(timeout=1.0)
        sys.exit(message)

    t0 = time.monotonic()
    while not (node.pub.get_subscription_count() >= 1 and node.count_publishers("/api/sport/response") >= 1):
        if time.monotonic() - t0 > 5.0:
            abort("ABORT: nobody answers on /api/sport (robot off, cable/Wi-Fi not up, or wrong DDS environment), nothing sent")
        time.sleep(0.1)
    time.sleep(0.5)                                   # let the endpoint matching settle

    # SAFETY: the Unitree remote is the person's backup (L2+B damps the robot), so it must be ON before this runs.
    # A remote that is off, or on but not linked to the robot, leaves /lf/lowstate.wireless_remote all zero.
    t0 = time.monotonic()
    while time.monotonic() - t0 < REMOTE_WAIT_S and not (node.low_t is not None and node.remote_alive(time.monotonic())):
        time.sleep(0.1)
    if node.low_t is None:
        if not args.dry:
            abort("ABORT: no /lf/lowstate arrives, so the Unitree remote cannot be checked: nothing sent")
    elif not node.remote_alive(time.monotonic()):
        warning = ("SAFETY: the Unitree remote is not detected (its payload in /lf/lowstate is all zero).\n"
                   "  For safety, SWITCH ON THE UNITREE REMOTE first: it is the backup (L2+B damps the robot).")
        if not args.dry:
            abort(warning + "\n  Nothing was sent; run this again when the remote is on.")
        print("warning (--dry, so it goes on): " + warning)

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
