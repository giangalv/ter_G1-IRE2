#!/usr/bin/env python3
"""Relay DDS discovery multicast between two network interfaces (runs ON the Jetson, user "unitree", no root).

Why: the robot's DDS (PC1, CycloneDDS) only learns about a new participant from its *multicast* SPDP packet
(239.255.0.1, port 7400 + 250 * domain). A laptop on the robot's Wi-Fi access point (wlan0) sends that packet on
the Wi-Fi only, and the Jetson does not route multicast, so PC1 never hears it (docs/07-wifi.md, test 2).
This relays those packets wlan0 <-> eth1. Everything after the discovery (data, replies) is unicast and goes
through the Jetson's normal IP forwarding.

    python3 wifi_dds_relay.py 10.42.0.1 192.168.123.164            # domain 0
    python3 wifi_dds_relay.py 10.42.0.1 192.168.123.164 --domain 0 --stats 30

The two arguments are the addresses of the two interfaces. Only the discovery port is relayed (the data
port, 7401, is not: with AllowMulticast=spdp on the laptop all data is unicast). Standard library only (Python 3.8).
"""
import argparse
import select
import socket
import struct
import sys
import time

IP_MULTICAST_ALL = 49  # linux/in.h; not in the socket module


def make_sockets(group, port, ifaddr):
    """(receive socket joined on ifaddr only, send socket out of ifaddr without local loop-back)."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    # without this a socket gets the group's packets from every interface on which anyone joined it
    rx.setsockopt(socket.IPPROTO_IP, IP_MULTICAST_ALL, 0)
    rx.bind(("", port))
    rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(group) + socket.inet_aton(ifaddr))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(ifaddr))
    tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
    return rx, tx


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("addr_a", help="address of the first interface, e.g. 10.42.0.1 (wlan0)")
    ap.add_argument("addr_b", help="address of the second interface, e.g. 192.168.123.164 (eth1)")
    ap.add_argument("--domain", type=int, default=0, help="DDS domain id (default 0)")
    ap.add_argument("--group", default="239.255.0.1", help="RTPS discovery group (default 239.255.0.1)")
    ap.add_argument("--stats", type=float, default=0.0, help="print counters every N seconds (0 = only at the end)")
    args = ap.parse_args()

    port = 7400 + 250 * args.domain
    rx_a, tx_a = make_sockets(args.group, port, args.addr_a)
    rx_b, tx_b = make_sockets(args.group, port, args.addr_b)
    # a packet read on A goes out of B and the other way round
    route = {rx_a: (tx_b, "A->B", args.addr_b), rx_b: (tx_a, "B->A", args.addr_a)}
    count = {"A->B": 0, "B->A": 0}
    print(f"relaying {args.group}:{port} between {args.addr_a} and {args.addr_b} (domain {args.domain})", flush=True)
    last = time.time()
    try:
        while True:
            ready, _, _ = select.select([rx_a, rx_b], [], [], 1.0)
            for rx in ready:
                data, _addr = rx.recvfrom(65535)
                tx, name, _out = route[rx]
                # no loop: tx has IP_MULTICAST_LOOP=0 and rx only gets the group on its own interface
                tx.sendto(data, (args.group, port))
                count[name] += 1
            if args.stats and time.time() - last >= args.stats:
                last = time.time()
                print(f"{time.strftime('%T')} relayed A->B {count['A->B']}, B->A {count['B->A']}", flush=True)
    except KeyboardInterrupt:
        pass
    print(f"stopped; relayed A->B {count['A->B']}, B->A {count['B->A']}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
