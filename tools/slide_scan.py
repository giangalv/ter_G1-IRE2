"""Read-only slide scan: HOST then pairs SECONDS LABEL ... ; prints, per 0.5 s bucket, which cells are active."""
import importlib.util, json, os, socket, statistics, subprocess, sys, time
spec = importlib.util.spec_from_file_location("tp", os.path.join(os.path.dirname(os.path.abspath(__file__)), "tactile_probe.py"))
tp = importlib.util.module_from_spec(spec); spec.loader.exec_module(tp)
THRESH, BUCKET = 40, 0.5
def say(t): subprocess.run(["spd-say", "-w", "-l", "it", t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
def beep(f, d=0.25): subprocess.Popen(["play", "-q", "-n", "synth", str(d), "sine", str(f)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
shape = {n: (r, c) for n, _, r, c in tp.REGIONS}
host, args = sys.argv[1], sys.argv[2:]
sock = socket.create_connection((host, 6000), timeout=2.0); sock.settimeout(1.0); tid = 0
say("Non toccare la mano. Misuro due secondi.")
rows_ = {}; end = time.time() + 2.0
while time.time() < end:
    d, tid = tp.read_all(sock, tid)
    for n, c in d.items(): rows_.setdefault(n, []).append(c)
base = {n: [statistics.median(col) for col in zip(*r)] for n, r in rows_.items()}
out = {}
for i in range(0, len(args), 2):
    secs, label = float(args[i]), args[i + 1]
    time.sleep(1.0); print(f"\n>>> {label} ({secs:.0f} s)", flush=True); say(label); beep(880)
    t0 = time.time(); buckets = {}
    while time.time() - t0 < secs:
        d, tid = tp.read_all(sock, tid); b = int((time.time() - t0) / BUCKET)
        for n, c in d.items():
            dl = [max(0, v - x) for v, x in zip(c, base[n])]
            old = buckets.setdefault(b, {}).get(n)
            buckets[b][n] = dl if old is None else [max(a, z) for a, z in zip(old, dl)]
    beep(330); line = []
    for b in sorted(buckets):
        act = {n: v for n, v in buckets[b].items() if max(v) >= THRESH}
        if not act: line.append((b * BUCKET, "-")); continue
        n = max(act, key=lambda k: max(act[k])); v = act[n]; cols = shape[n][1]
        on = [j for j, x in enumerate(v) if x >= THRESH]
        rs = sorted({j // cols for j in on}); cs = sorted({j % cols for j in on})
        line.append((b * BUCKET, f"{n} rows {rs[0]}-{rs[-1]} cols {cs[0]}-{cs[-1]} peak+{max(v):.0f}"))
    for t, s in line: print(f"  t={t:4.1f}s  {s}")
    out[label] = line
sock.close(); say("Scorrimento finito.")
json.dump(out, open(os.environ.get("SLIDE_OUT", "/dev/null"), "w"))
