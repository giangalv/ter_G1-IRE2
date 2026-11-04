"""Read-only Modbus TCP probe (function 0x03 only) for the Inspire hands.

Usage: python3 -I modbus_probe.py HOST [PORT] [UNIT]
Never writes: only builds read-holding-registers requests.

Left hand = 192.168.123.210, right hand = 192.168.123.211, port 6000, unit 1.
The addresses below are BYTE offsets (all even): a read of N registers at
address A returns the words at bytes A, A+2, ... Odd addresses return
byte-shifted garbage.
"""
import socket
import struct
import sys

HOST = sys.argv[1]
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 6000
UNIT = int(sys.argv[3]) if len(sys.argv) > 3 else 1

# (name, byte address, count) -- Inspire register map; verified on the RH56E2
# for HAND_ID, ANGLE_ACT and FORCE_ACT (the others answered with plausible values).
BLOCKS = [
    ("HAND_ID", 1000, 1),
    ("REDU_RATIO", 1002, 1),
    ("ANGLE_SET", 1486, 6),
    ("FORCE_SET", 1498, 6),
    ("SPEED_SET", 1522, 6),
    ("ANGLE_ACT", 1546, 6),
    ("FORCE_ACT", 1582, 6),
    ("CURRENT", 1594, 6),
    ("ERROR", 1606, 3),
    ("STATUS", 1612, 3),
    ("TEMP", 1618, 3),
]

_tid = 0


def read_regs(sock, addr, count):
    global _tid
    _tid = (_tid + 1) & 0xFFFF
    req = struct.pack(">HHHBBHH", _tid, 0, 6, UNIT, 0x03, addr, count)
    sock.sendall(req)
    hdr = b""
    while len(hdr) < 7:
        chunk = sock.recv(7 - len(hdr))
        if not chunk:
            raise ConnectionError("closed while reading MBAP header")
        hdr += chunk
    _, _, length, _ = struct.unpack(">HHHB", hdr)
    body = b""
    while len(body) < length - 1:
        chunk = sock.recv(length - 1 - len(body))
        if not chunk:
            raise ConnectionError("closed while reading PDU")
        body += chunk
    func = body[0]
    if func & 0x80:
        return ("EXC", body[1])
    nbytes = body[1]
    vals = struct.unpack(">%dH" % (nbytes // 2), body[2:2 + nbytes])
    return ("OK", list(vals))


def main():
    print(f"== {HOST}:{PORT} unit={UNIT} ==")
    with socket.create_connection((HOST, PORT), timeout=2.0) as s:
        s.settimeout(1.5)
        for name, addr, count in BLOCKS:
            try:
                kind, val = read_regs(s, addr, count)
            except (socket.timeout, ConnectionError, OSError) as exc:
                print(f"{name:11s} @{addr:5d} x{count}: ERROR {exc!r}")
                break
            if kind == "EXC":
                print(f"{name:11s} @{addr:5d} x{count}: modbus exception {val}")
            else:
                signed = [v - 65536 if v > 32767 else v for v in val]
                print(f"{name:11s} @{addr:5d} x{count}: raw={val} signed={signed}")


if __name__ == "__main__":
    main()
