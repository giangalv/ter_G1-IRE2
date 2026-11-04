"""Read-only wide capture of both hands (function 0x03 only), saved to .npz.

Usage: python3 -I modbus_capture.py SECONDS OUT_PREFIX

Modbus addresses on the RH56E2 are BYTE offsets: a read of N registers at an
even address A returns the N little-endian 16-bit words at bytes A, A+2, ...
The capture sweeps bytes 0..5999 (reads at >= 6000 drop the connection) in
chunks of 100 words and stores them so that ``x[:, k]`` is the word at byte
address ``2*k``. Saves OUT_PREFIX_210.npz / OUT_PREFIX_211.npz with ``t``
(sample times) and ``x`` (samples x 3000 words). ~9 Hz per hand.
"""
import socket
import struct
import sys
import threading
import time

import numpy as np

DUR = float(sys.argv[1])
PREFIX = sys.argv[2]
HOSTS = {"210": "192.168.123.210", "211": "192.168.123.211"}
LAST = 6000          # byte addresses 0..5999 (6000+ drops the connection)
CHUNK = 100
result = {}


def recv_exact(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            raise ConnectionError("closed")
        buf += c
    return buf


def worker(tag, host):
    tid = 0
    s = socket.create_connection((host, 6000), timeout=2.0)
    s.settimeout(1.0)
    times, rows = [], []
    t_end = time.time() + DUR
    while time.time() < t_end:
        row = np.zeros(LAST // 2, dtype=np.uint16)
        t0 = time.time()
        ok = True
        for a in range(0, LAST, 2 * CHUNK):
            n = min(CHUNK, (LAST - a) // 2)
            tid = (tid + 1) & 0xFFFF
            try:
                s.sendall(struct.pack(">HHHBBHH", tid, 0, 6, 1, 0x03, a, n))
                hdr = recv_exact(s, 7)
                ln = struct.unpack(">H", hdr[4:6])[0]
                body = recv_exact(s, ln - 1)
            except (socket.timeout, ConnectionError, OSError):
                ok = False
                try:
                    s.close()
                except OSError:
                    pass
                time.sleep(0.2)
                s = socket.create_connection((host, 6000), timeout=2.0)
                s.settimeout(1.0)
                break
            if body[0] & 0x80:
                ok = False
                break
            row[a // 2:a // 2 + n] = struct.unpack(">%dH" % n, body[2:2 + 2 * n])
        if ok:
            times.append((t0 + time.time()) / 2)
            rows.append(row)
    s.close()
    result[tag] = (np.array(times), np.array(rows))


threads = [threading.Thread(target=worker, args=(k, v)) for k, v in HOSTS.items()]
for t in threads:
    t.start()
for t in threads:
    t.join()
for tag, (tt, rr) in result.items():
    np.savez(f"{PREFIX}_{tag}.npz", t=tt, x=rr)
    print(tag, "samples:", len(tt), "rate ~%.1f Hz" % (len(tt) / DUR))
