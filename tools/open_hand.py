"""Open one Inspire RH56E2 hand slowly (first write to the hardware, user-approved 2026-11-04).

usage: python3 open_hand.py HOST [destra|sinistra] [--keep-thumb]
--keep-thumb opens only the four fingers; the thumb bend stays where it is (the thumb rotation is always kept)."""
import socket, struct, subprocess, sys, time

KEEP_THUMB = "--keep-thumb" in sys.argv
_args = [a for a in sys.argv[1:] if a != "--keep-thumb"]
HOST = _args[0]
SIDE = _args[1] if len(_args) > 1 else "destra"   # only used in the spoken warning
SPEED_SLOW, SPEED_FACTORY = 300, 1000
FORCE_ABORT = 700          # |FORCE_ACT| above this -> hold position and stop (idle ~130)
A_SET, F_SET, S_SET, A_ACT, F_ACT, CUR, ERR, STA, TMP = 1486, 1498, 1522, 1546, 1582, 1594, 1606, 1612, 1618

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

say(f"Apro la mano {SIDE} tra cinque secondi. Tieni le mani lontane dalle dita.")
time.sleep(5)

wr(S_SET, [SPEED_SLOW] * 6)
assert rd(S_SET, 6) == [SPEED_SLOW] * 6, "SPEED_SET readback differs"
print("SPEED_SET -> 300 confirmed", flush=True)

thumb_rot = hold(5)
thumb_bend = hold(4) if KEEP_THUMB else 1000
target = [1000, 1000, 1000, 1000, thumb_bend, thumb_rot]
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
    if all(a >= 950 for a in ang[:4 if KEEP_THUMB else 5]) or time.time() - last_move > 2.5: break

wr(S_SET, [SPEED_FACTORY] * 6)
final = snapshot("after")
print("RESULT:", "aborted" if aborted else "done", flush=True)
sock.close()
if not aborted: say(f"Mano {SIDE} aperta. Dimmi pronto quando vuoi iniziare.")
