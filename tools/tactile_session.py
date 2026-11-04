"""Guided read-only tactile session (Modbus function 0x03 only).

Usage: python3 -I tactile_session.py HOST SECONDS_PER_STEP LABEL [LABEL ...]

Takes 2 s of baseline (hand untouched), then for every LABEL: high beep = start
touching, SECONDS_PER_STEP of recording, low beep = release. Per step it prints the
regions with the largest rise over the baseline, with the cell (row,col) where the
peak was, so the physical meaning of each region can be read off.
"""
import importlib.util
import json
import os
import socket
import statistics
import subprocess
import sys
import time

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tactile_probe.py")
spec = importlib.util.spec_from_file_location("tactile_probe", TOOLS)
tp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tp)

THRESH = 20  # counts above baseline that count as "touched"


def beep(freq, dur=0.25):
    subprocess.Popen(["play", "-q", "-n", "synth", str(dur), "sine", str(freq)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def say(text):
    """Speak `text` in Italian and wait until it is finished (fallback: silent)."""
    try:
        subprocess.run(["spd-say", "-w", "-l", "it", text], timeout=15,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def record(sock, tid, seconds):
    """Per region: list of per-cell maxima over the window, and the last tid."""
    peak = {}
    end = time.time() + seconds
    while time.time() < end:
        data, tid = tp.read_all(sock, tid)
        for name, cells in data.items():
            old = peak.get(name)
            peak[name] = list(cells) if old is None else [max(a, b) for a, b in zip(old, cells)]
    return peak, tid


def main():
    host, secs, labels = sys.argv[1], float(sys.argv[2]), sys.argv[3:]
    sock = socket.create_connection((host, 6000), timeout=2.0)
    sock.settimeout(1.0)
    tid = 0
    shape = {n: (r, c) for n, _, r, c in tp.REGIONS}

    time.sleep(0.5)
    say("Non toccare la mano. Misuro due secondi.")
    print("baseline 2 s: do NOT touch the hand", flush=True)
    samples = {}
    end = time.time() + 2.0
    while time.time() < end:
        data, tid = tp.read_all(sock, tid)
        for n, cells in data.items():
            samples.setdefault(n, []).append(cells)
    base = {n: [statistics.median(col) for col in zip(*rows)] for n, rows in samples.items()}

    saved = {}
    for label in labels:
        time.sleep(1.0)
        print(f"\n>>> {label}", flush=True)
        say(label)
        beep(880)
        peak, tid = record(sock, tid, secs)
        beep(330)
        rows = []
        for n, cells in peak.items():
            delta = [max(0, v - b) for v, b in zip(cells, base[n])]
            m = max(delta)
            if m >= THRESH:
                i = delta.index(m)
                cols = shape[n][1]
                rows.append((m, n, sum(d >= THRESH for d in delta), (i // cols, i % cols)))
        rows.sort(reverse=True)
        saved[label] = {n: [max(0, v - b) for v, b in zip(cells, base[n])]
                        for n, cells in peak.items() if max(cells) - max(base[n]) >= THRESH}
        if not rows:
            print("    nothing above threshold")
        for m, n, cnt, rc in rows[:5]:
            print(f"    {n:13s} peak +{m:4.0f}  cells>={THRESH}: {cnt:3d}  peak at (row,col)={rc}")
    sock.close()
    out = os.environ.get("TACTILE_OUT")
    if out:
        with open(out, "w") as fh:
            json.dump(saved, fh)


if __name__ == "__main__":
    main()
