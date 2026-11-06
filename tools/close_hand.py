"""Close one Inspire RH56E2 hand to a relaxed fist, slowly. Without --go it only reads and prints the plan.

usage: python3 close_hand.py HOST [destra|sinistra] [--go]      (HOST 192.168.123.211 = right, .210 = left)

Same write path as open_hand.py (the only writes verified on these hands): SPEED_SET 300, then ANGLE_SET, then SPEED_SET back to 1000.
The target is not a full squeeze: fingers 100, thumb bend 200 (0 = fully closed), thumb rotation unchanged, so the fingertips do not
press on the palm. It stops and holds the current position when ERROR is not zero or |FORCE_ACT| goes over 700.
FORCE_SET (factory 2000) is not touched. Open again with open_hand.py.
"""
import socket, struct, subprocess, sys, time

GO = "--go" in sys.argv
args = [a for a in sys.argv[1:] if a != "--go"]
HOST = args[0]
SIDE = args[1] if len(args) > 1 else "destra"   # only used in the spoken warning
SPEED_SLOW, SPEED_FACTORY = 300, 1000
FORCE_ABORT = 700          # |FORCE_ACT| above this -> hold position and stop (idle ~130)
A_SET, F_SET, S_SET, A_ACT, F_ACT, CUR, ERR, STA, TMP = 1486, 1498, 1522, 1546, 1582, 1594, 1606, 1612, 1618
FINGERS, THUMB_BEND = 100, 200

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
if any(st["ERROR"]): sys.exit("ERROR register not zero - not writing anything")
if max(abs(v) for v in st["FORCE_ACT"]) > FORCE_ABORT: sys.exit("fingers already loaded - not writing anything")

thumb_rot = max(0, st["ANGLE_ACT"][5])
target = [FINGERS, FINGERS, FINGERS, FINGERS, THUMB_BEND, thumb_rot]
print(f"plan: SPEED_SET -> {SPEED_SLOW}, ANGLE_SET -> {target} (little, ring, middle, index, thumb bend, thumb rotation), "
      f"abort at |FORCE_ACT| > {FORCE_ABORT}, SPEED_SET back to {SPEED_FACTORY}", flush=True)
if not GO:
    print("dry run: nothing written (add --go to close the hand)", flush=True); sock.close(); sys.exit(0)

say(f"Chiudo la mano {SIDE} tra cinque secondi. Tieni le mani lontane dalle dita.")
time.sleep(5)

wr(S_SET, [SPEED_SLOW] * 6)
assert rd(S_SET, 6) == [SPEED_SLOW] * 6, "SPEED_SET readback differs"
print("SPEED_SET -> 300 confirmed", flush=True)

wr(A_SET, target)
print("ANGLE_SET written", target, "readback", [s16(v) for v in rd(A_SET, 6)], flush=True)

t0 = time.time(); last_move = t0; prev = None; aborted = False
while time.time() - t0 < 12:
    time.sleep(0.4)
    ang = [s16(v) for v in rd(A_ACT, 6)]; frc = [s16(v) for v in rd(F_ACT, 6)]; err = rd(ERR, 3)
    print(f"{time.time()-t0:4.1f}s ANGLE_ACT {ang} FORCE_ACT {frc} ERROR {err}", flush=True)
    if any(err) or max(abs(v) for v in frc) > FORCE_ABORT:
        wr(A_SET, [max(0, a) for a in ang]); aborted = True
        print("ABORT: error/force limit -> holding current position", flush=True); break
    if prev is None or max(abs(a - b) for a, b in zip(ang, prev)) > 3: last_move = time.time()
    prev = ang
    if all(a <= FINGERS + 30 for a in ang[:4]) or time.time() - last_move > 2.5: break

wr(S_SET, [SPEED_FACTORY] * 6)
final = snapshot("after")
print("RESULT:", "aborted" if aborted else "done", flush=True)
sock.close()
if not aborted: say(f"Mano {SIDE} chiusa. Dimmi pronto quando vuoi iniziare.")
