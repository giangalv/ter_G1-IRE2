"""Which palm camera belongs to which hand? Cover-the-lens test with audio cues.

Streams 16x12 grayscale thumbnails of the two palm cameras (/dev/video6 = A,
/dev/video8 = B) from the Jetson over SSH (read-only: it only opens the two
capture nodes) while the laptop beeps a schedule: cover the lens of the camera
of ONE hand at the double beep, uncover it at the low beep. The camera whose
thumbnail changes most while covered is that hand's camera.

Usage:  G1_PW=<jetson password> python3 cam_cover_test.py LEAD CYCLES [--dry]
        (LEAD = seconds before the baseline starts, e.g. 15; CYCLES e.g. 4)

Needs `play` (sox) for the audio cues and tools/askpass.sh (echoes $G1_PW to
ssh); the password is read from the environment, never stored. Brightness is
a poor metric (covering barely changes it), hence the thumbnail difference.
Raw samples are written to cover_test_raw.npz in the current directory.
"""
import os
import subprocess
import sys
import threading
import time

import numpy as np

LEAD = float(sys.argv[1])
CYCLES = int(sys.argv[2])
DRY = "--dry" in sys.argv
BASE, COVER, REST = 6.0, 3.0, 3.0
SP = os.path.dirname(os.path.abspath(__file__))
total = 6.0 if DRY else LEAD + BASE + CYCLES * (COVER + REST) + 3.0

REMOTE = r'''
import cv2, sys, time
caps = {}
for name, idx in (("A", 6), ("B", 8)):
    c = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 320); c.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    caps[name] = c
end = time.time() + %f
while time.time() < end:
    out = []
    for name, c in caps.items():
        ok, f = c.read()
        if ok:
            g = cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (16, 12), interpolation=cv2.INTER_AREA)
            out.append("%%s=%%s" %% (name, ",".join(str(int(v)) for v in g.flatten())))
        else:
            out.append("%%s=" %% name)
    sys.stdout.write(" ".join(out) + "\n"); sys.stdout.flush()
'''

if "G1_PW" not in os.environ:
    sys.exit("set G1_PW to the Jetson password for user unitree")
env = dict(os.environ, SSH_ASKPASS=f"{SP}/askpass.sh",
           SSH_ASKPASS_REQUIRE="force", DISPLAY=":0")
cmd = ["ssh", "-o", "StrictHostKeyChecking=accept-new", "-o", "PubkeyAuthentication=no",
       "-o", "PreferredAuthentications=password,keyboard-interactive",
       "-o", "ConnectTimeout=8", "-o", "NumberOfPasswordPrompts=1",
       "unitree@192.168.123.164", "python3 -c '" + (REMOTE % (total + 4.0)).replace("'", "'\"'\"'") + "'"]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env)
samples = []  # (t_local, A, B)


def reader():
    for line in proc.stdout:
        try:
            kv = dict(p.split("=") for p in line.split())
            a = np.array([int(v) for v in kv["A"].split(",")], dtype=float)
            b = np.array([int(v) for v in kv["B"].split(",")], dtype=float)
            if a.size == 192 and b.size == 192:
                samples.append((time.time(), a, b))
        except (ValueError, KeyError):
            pass


threading.Thread(target=reader, daemon=True).start()


def beep(freq, dur=0.15):
    subprocess.run(["play", "-q", "-n", "synth", str(dur), "sine", str(freq), "vol", "0.6"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def multi(freq, n, dur=0.15, gap=0.1):
    for _ in range(n):
        beep(freq, dur)
        time.sleep(gap)


events = []
if not DRY:
    sched = [(0.0, "start", lambda: multi(440, 2))]
    for t in (LEAD - 3, LEAD - 2, LEAD - 1):
        if t > 1:
            sched.append((t, "tick", lambda: beep(660, 0.1)))
    sched.append((LEAD, "baseline", lambda: beep(1000, 0.6)))
    t = LEAD + BASE
    for c in range(CYCLES):
        sched.append((t, f"cover_{c}", lambda: multi(1200, 2, 0.12, 0.08)))
        sched.append((t + COVER, f"uncover_{c}", lambda: beep(500, 0.4)))
        t += COVER + REST
    sched.append((t + 0.5, "end", lambda: multi(440, 4, 0.15, 0.12)))
    t0 = time.time()
    for t_rel, name, action in sched:
        d = t0 + t_rel - time.time()
        if d > 0:
            time.sleep(d)
        events.append((name, time.time()))
        action()
else:
    time.sleep(total)

proc.terminate()
T = np.array([x[0] for x in samples])
A = np.array([x[1] for x in samples])
B = np.array([x[2] for x in samples])
np.savez("cover_test_raw.npz", t=T, A=A, B=B)
print(f"samples: {len(T)}  rate ~{len(T) / total:.1f} Hz (both cameras per sample)")
if len(T) == 0:
    sys.exit("no data from the cameras")
if DRY:
    sys.exit(0)

ev = dict(events)
b0, b1 = ev["baseline"] + 1.0, ev["cover_0"] - 0.3
base = (T >= b0) & (T < b1)
refA, refB = np.median(A[base], axis=0), np.median(B[base], axis=0)
dA = np.abs(A - refA).mean(axis=1)
dB = np.abs(B - refB).mean(axis=1)
noiseA, noiseB = dA[base].mean(), dB[base].mean()
print("camera A = /dev/video6 (USB 1-2.2.1), camera B = /dev/video8 (USB 1-2.2.2)")
print(f"baseline change vs reference (noise): A={noiseA:.1f}  B={noiseB:.1f}   [0..255 scale]")
gains = {"A": [], "B": []}
for c in range(CYCLES):
    c0, u0 = ev[f"cover_{c}"], ev[f"uncover_{c}"]
    cov = (T >= c0 + 0.8) & (T < u0)
    if cov.sum() < 2:
        continue
    ga, gb = dA[cov].mean(), dB[cov].mean()
    gains["A"].append(ga)
    gains["B"].append(gb)
    print(f"cycle {c}: change while covered  A={ga:.1f}  B={gb:.1f}")
ma, mb = float(np.mean(gains["A"])), float(np.mean(gains["B"]))
print(f"MEAN change while covered  A={ma:.1f} (x{ma / max(noiseA, 0.5):.1f} noise)  B={mb:.1f} (x{mb / max(noiseB, 0.5):.1f} noise)")
ra, rb = ma / max(noiseA, 0.5), mb / max(noiseB, 0.5)
if max(ra, rb) < 2.0:
    print("=> inconclusive: neither camera changed clearly")
elif max(ra, rb) > 2.0 * min(ra, rb) + 1.0:
    print("=> covered camera:", "A (video6, USB 1-2.2.1)" if ra > rb else "B (video8, USB 1-2.2.2)")
else:
    print("=> inconclusive: both changed")
