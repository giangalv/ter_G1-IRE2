"""Move the thumb-rotation actuator of ONE Inspire RH56E2 hand in steps and back, to check that it works and which way it goes.
Without --go it only reads and prints the plan.

usage: python3 thumb_test.py HOST [destra|sinistra] [--go] [--steps=850,700] [--poll=0.4] [--stall=2.5] [--bend=N] [--channel=rot|bend]
       (HOST 192.168.123.211 = right, .210 = left)
--channel which thumb channel the steps move: rot (rotation, default) or bend
--steps  the set values that channel goes through, then back to the start value (default 850,700)
--poll   seconds between two readings (default 0.4); --stall  seconds without movement before a step counts as stalled (2.5)
--bend   first move the thumb bend to N (1000 = thumb stretched, far from the palm) and put it back at the end
Near an end stop use e.g. --steps=700,500,300,150,0 --poll=0.1 --stall=0.5 --bend=1000: a stall or a rise of the rotation
force of more than 250 over its idle value counts as contact, the set value is moved 40 counts back off the stop and the test
goes straight to the way back.

Writes only SPEED_SET (300, back to 1000 at the end) and ANGLE_SET. The other five channels are written back at their current
set value, so only the thumb rotation moves. Steps: start -> 850 -> 700 -> start. After every ANGLE_SET it reads the six angles
and forces every 0.4 s. It stops that step when the angle is within 30 of the target (the actuator rests ~15 below its set value),
when it has not moved for 2.5 s, or after 6 s; it does not go on to the next step after a stall. It aborts and holds the
current position when ERROR is not zero or |FORCE_ACT| goes over 700 on any channel.
"""
import socket, struct, subprocess, sys, time

GO = "--go" in sys.argv
args = [a for a in sys.argv[1:] if not a.startswith("--")]

def opt(name, default):
    for a in sys.argv[1:]:
        if a.startswith(name + "="): return a.split("=", 1)[1]
    return default
HOST = args[0]
SIDE = args[1] if len(args) > 1 else "destra"   # only used in the spoken warning
SPEED_SLOW, SPEED_FACTORY = 300, 1000
FORCE_ABORT = 700          # |FORCE_ACT| above this -> hold position and stop (idle ~130)
STEPS = [int(x) for x in opt("--steps", "850,700").split(",")]
POLL, STALL = float(opt("--poll", "0.4")), float(opt("--stall", "2.5"))
BEND = int(opt("--bend", "-1"))     # -1 = leave the thumb bend alone
CONTACT = 250                      # rise of |FORCE_ACT| of the rotation over its idle value that counts as contact
BACKOFF = 40
A_SET, F_SET, S_SET, A_ACT, F_ACT, CUR, ERR, STA, TMP = 1486, 1498, 1522, 1546, 1582, 1594, 1606, 1612, 1618
ROT, BENDCH = 5, 4

sock = socket.create_connection((HOST, 6000), timeout=2.0); sock.settimeout(1.5)
tid = 0

def recv_exact(n):
    b = b""
    while len(b) < n:
        c = sock.recv(n - len(b))
        if not c: raise ConnectionError("closed by the hand")
        b += c
    return b

def txn(pdu):
    global tid
    tid += 1
    sock.sendall(struct.pack(">HHHB", tid, 0, 1 + len(pdu), 1) + pdu)
    h = recv_exact(7); body = recv_exact(struct.unpack(">H", h[4:6])[0] - 1)
    if body[0] & 0x80: raise RuntimeError(f"modbus exception {body[1]} for fn {pdu[0]:#x}")
    return body

def rd(addr, n): return list(struct.unpack(">%dH" % n, txn(struct.pack(">BHH", 3, addr, n))[2:2 + 2 * n]))
def s16(v): return v - 65536 if v > 32767 else v
def wr(addr, vals):
    body = txn(struct.pack(">BHHB", 0x10, addr, len(vals), 2 * len(vals)) + struct.pack(">%dH" % len(vals), *vals))
    if struct.unpack(">HH", body[1:5]) != (addr, len(vals)): raise RuntimeError(f"unexpected write echo {body.hex()}")
def say(t): subprocess.run(["spd-say", "-w", "-l", "it", t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def snapshot(tag):
    st = dict(ANGLE_SET=[s16(v) for v in rd(A_SET, 6)], FORCE_SET=rd(F_SET, 6), SPEED_SET=rd(S_SET, 6),
              ANGLE_ACT=[s16(v) for v in rd(A_ACT, 6)], FORCE_ACT=[s16(v) for v in rd(F_ACT, 6)],
              ERROR=rd(ERR, 3), STATUS=rd(STA, 3), TEMP=rd(TMP, 3))
    print(f"[{tag}]", st, flush=True); return st

st = snapshot("before")

def hold(i):
    """value to write back for a channel that must not move: the last set value if it is sane (the actuator rests ~15 below it,
    so writing the measured value back would make it creep), else the measured one (the register reads garbage after a power-up)"""
    s, a = st["ANGLE_SET"][i], st["ANGLE_ACT"][i]
    return s if 0 <= s <= 1000 and abs(s - a) <= 40 else max(0, a)

if any(st["ERROR"]): sys.exit("ERROR register not zero - not writing anything")
if max(abs(v) for v in st["FORCE_ACT"]) > FORCE_ABORT: sys.exit("fingers already loaded - not writing anything")

base = [hold(i) for i in range(6)]
orig = list(base)
CH = BENDCH if opt("--channel", "rot") == "bend" else ROT
NAME = {ROT: "thumb rotation", BENDCH: "thumb bend"}
idle_force = list(st["FORCE_ACT"])
prelude = BEND >= 0 and CH == ROT
print(f"plan: SPEED_SET -> {SPEED_SLOW}; " + (f"thumb bend {orig[BENDCH]} -> {BEND} first; " if prelude else "")
      + f"{NAME[CH]} {orig[CH]} -> {STEPS} -> {orig[CH]}; " + (f"thumb bend back to {orig[BENDCH]}; " if prelude else "")
      + f"read every {POLL} s, stalled after {STALL} s, contact = force of the moving channel {CONTACT} over its idle value "
      f"(then {BACKOFF} counts back off); abort at |FORCE_ACT| > {FORCE_ABORT} or ERROR; SPEED_SET back to {SPEED_FACTORY}", flush=True)
if not GO:
    print("dry run: nothing written (add --go to run the test)", flush=True); sock.close(); sys.exit(0)

say(f"Muovo il pollice della mano {SIDE} tra cinque secondi. Tieni le mani lontane dalla mano.")
time.sleep(5)

wr(S_SET, [SPEED_SLOW] * 6)
assert rd(S_SET, 6) == [SPEED_SLOW] * 6, "SPEED_SET readback differs"
print("SPEED_SET -> 300 confirmed", flush=True)

def back_off(ch, ang, dirn, why):
    """stop pushing against whatever blocks the channel: set value BACKOFF counts behind the measured angle"""
    b = min(1000, max(0, ang[ch] - dirn * BACKOFF))
    cmd = list(base); cmd[ch] = b
    wr(A_SET, cmd); base[ch] = b
    print(f"{why}: set value moved back to {b} (measured {ang[ch]})", flush=True)

def move_to(ch, target):
    """write the new set value of one channel and watch until reached / contact / stalled / timeout / abort; returns (verdict, measured)"""
    ang0 = [s16(v) for v in rd(A_ACT, 6)]
    dirn = 1 if target >= ang0[ch] else -1
    cmd = list(base); cmd[ch] = target
    wr(A_SET, cmd); base[ch] = target
    print(f"ANGLE_SET written {cmd} readback {[s16(v) for v in rd(A_SET, 6)]}", flush=True)
    t0 = time.time(); last_move = t0; prev = None
    while time.time() - t0 < 8:
        time.sleep(POLL)
        ang = [s16(v) for v in rd(A_ACT, 6)]; frc = [s16(v) for v in rd(F_ACT, 6)]; err = rd(ERR, 3)
        print(f"{time.time()-t0:4.1f}s ANGLE_ACT {ang} FORCE_ACT {frc} ERROR {err}", flush=True)
        if any(err) or max(abs(v) for v in frc) > FORCE_ABORT:
            wr(A_SET, [max(0, a) for a in ang])
            print("ABORT: error/force limit -> holding current position", flush=True)
            return "aborted", ang[ch]
        if abs(frc[ch] - idle_force[ch]) > CONTACT:
            back_off(ch, ang, dirn, "CONTACT")
            return "contact", ang[ch]
        if prev is None or abs(ang[ch] - prev) > 2: last_move = time.time()
        prev = ang[ch]
        if abs(ang[ch] - target) <= 30: return "reached", ang[ch]
        if time.time() - last_move > STALL:
            back_off(ch, ang, dirn, "STALLED")
            return "stalled", ang[ch]
    return "timeout", prev

results = []
verdict = "reached"
if prelude:
    print(f"--- thumb bend -> {BEND}", flush=True)
    verdict, a = move_to(BENDCH, BEND); results.append(("thumb bend", BEND, verdict, a))
if verdict == "reached":
    for target in STEPS:
        print(f"--- step: {NAME[CH]} -> {target}", flush=True)
        verdict, a = move_to(CH, target); results.append((NAME[CH], target, verdict, a))
        if verdict != "reached": break
if verdict != "aborted":
    print(f"--- back: {NAME[CH]} -> {orig[CH]}", flush=True)
    verdict, a = move_to(CH, orig[CH]); results.append((NAME[CH], orig[CH], verdict, a))
    if prelude and verdict != "aborted":
        print(f"--- back: thumb bend -> {orig[BENDCH]}", flush=True)
        verdict, a = move_to(BENDCH, orig[BENDCH]); results.append(("thumb bend", orig[BENDCH], verdict, a))

wr(S_SET, [SPEED_FACTORY] * 6)
snapshot("after")
print("RESULTS (channel, target, verdict, measured):", flush=True)
for r in results: print("  ", r, flush=True)
sock.close()
say(f"Test del pollice della mano {SIDE} finito.")
