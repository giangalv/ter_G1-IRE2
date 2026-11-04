"""Read-only live view of the RH56E2-T1 tactile arrays (Modbus function 0x03 only).

Usage: python3 -I tools/tactile_probe.py HOST [SECONDS] [--map]

  HOST     192.168.123.211 = right hand, 192.168.123.210 = left hand
  SECONDS  how long to watch (default 10); one line per second
  --map    also draw the most active region as a 0-9 heat map at the end

Prints, per second, every region that has signal: sum of its cells, maximum
cell, and how many cells are above 20 counts (0..4095 scale). Nothing is
written to the hand. Standard library only.

Layout (byte addresses, 16-bit cells, 17 regions, 1062 cells): the one of
Inspire's RH56DFTP manual, confirmed on the RH56E2-2R-T1 with a pull test of
the right little finger (only little_nail and little_pad moved). Which
physical part of a finger each region covers is NOT verified.
"""
import socket
import struct
import sys
import time

REGIONS = [  # (name, byte address, rows, cols)
    ("little_tip", 3000, 3, 3), ("little_nail", 3018, 12, 8), ("little_pad", 3210, 10, 8),
    ("ring_tip", 3370, 3, 3), ("ring_nail", 3388, 12, 8), ("ring_pad", 3580, 10, 8),
    ("middle_tip", 3740, 3, 3), ("middle_nail", 3758, 12, 8), ("middle_pad", 3950, 10, 8),
    ("index_tip", 4110, 3, 3), ("index_nail", 4128, 12, 8), ("index_pad", 4320, 10, 8),
    ("thumb_tip", 4480, 3, 3), ("thumb_nail", 4498, 12, 8),
    ("thumb_middle", 4690, 3, 3), ("thumb_pad", 4708, 12, 8),
    ("palm", 4900, 8, 14),
]
UNIT = 1
MAX_WORDS = 125


def recv_exact(sock, size):
    buf = b""
    while len(buf) < size:
        chunk = sock.recv(size - len(buf))
        if not chunk:
            raise ConnectionError("closed by the hand")
        buf += chunk
    return buf


def read_words(sock, tid, address, count):
    """Function 0x03 read of `count` registers at an EVEN byte address."""
    sock.sendall(struct.pack(">HHHBBHH", tid & 0xFFFF, 0, 6, UNIT, 0x03, address, count))
    header = recv_exact(sock, 7)
    length = struct.unpack(">H", header[4:6])[0]
    body = recv_exact(sock, length - 1)
    if body[0] & 0x80:
        raise RuntimeError(f"modbus exception {body[1]} at {address}")
    return list(struct.unpack(">%dH" % count, body[2:2 + 2 * count]))


def read_all(sock, tid):
    """All regions, merging address-contiguous ones into reads of <= 125 words."""
    out, batch = {}, []

    def flush():
        nonlocal tid
        if not batch:
            return
        start = batch[0][1]
        words = sum(r[2] * r[3] for r in batch)
        tid += 1
        values = read_words(sock, tid, start, words)
        cursor = 0
        for name, _, rows, cols in batch:
            out[name] = values[cursor:cursor + rows * cols]
            cursor += rows * cols
        batch.clear()

    for region in REGIONS:
        if batch:
            end = batch[0][1] + 2 * sum(r[2] * r[3] for r in batch)
            size = sum(r[2] * r[3] for r in batch) + region[2] * region[3]
            if region[1] != end or size > MAX_WORDS:
                flush()
        batch.append(region)
    flush()
    return out, tid


def heat_map(values, rows, cols, peak):
    lines = []
    for r in range(rows):
        row = values[r * cols:(r + 1) * cols]
        lines.append(" ".join(str(min(9, int(9 * v / peak))) if v else "." for v in row))
    return "\n".join(lines)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    host = args[0]
    seconds = float(args[1]) if len(args) > 1 else 10.0
    draw = "--map" in sys.argv
    sock = socket.create_connection((host, 6000), timeout=2.0)
    sock.settimeout(1.0)
    tid = 0
    peak = {name: 0 for name, *_ in REGIONS}
    last = {}
    end = time.time() + seconds
    next_print = 0.0
    while time.time() < end:
        data, tid = read_all(sock, tid)
        last = data
        for name, cells in data.items():
            peak[name] = max(peak[name], max(cells))
        if time.time() >= next_print:
            next_print = time.time() + 1.0
            active = [f"{n}: sum={sum(c)} max={max(c)} cells>20={sum(v > 20 for v in c)}"
                      for n, c in data.items() if max(c) > 0]
            print(time.strftime("%H:%M:%S"), " | ".join(active) if active else "(all cells 0)")
    sock.close()
    print("\npeak cell per region over the run:",
          {n: p for n, p in peak.items() if p > 0} or "none above 0")
    if draw and any(peak.values()):
        name = max(peak, key=peak.get)
        _, _, rows, cols = next(r for r in REGIONS if r[0] == name)
        print(f"\nlast frame of {name} ({rows}x{cols}), 0-9 = fraction of {peak[name]}:")
        print(heat_map(last[name], rows, cols, peak[name]))


if __name__ == "__main__":
    main()
