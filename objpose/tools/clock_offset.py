#!/usr/bin/env python3
"""clock_offset.py — measure another host's wall clock minus this host's (UDP ping-pong, no root, no NTP).

    python3 objpose/tools/clock_offset.py --host 192.168.2.1 --ssh laptop     # starts the echo on the host over ssh
    python3 objpose/tools/clock_offset.py --host 192.168.2.1 --no-ssh         # echo already running there:
        # remote:  python3 clock_offset.py --echo

Prints the median offset in seconds on stdout (last line, `offset_s=+0.0207`) — pass it to
`hub.py --sam-tau-s` in live mode (SLAM host clock = this PC's clock + offset). Symmetric-path assumption:
on a direct cable the RTT is < 1 ms, so the error is far below the 20 ms rig tolerance.
"""
from __future__ import annotations

import argparse
import socket
import statistics
import subprocess
import sys
import time

PORT = 17999


def echo(seconds: float):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", PORT))
    s.settimeout(1.0)
    t_end = time.time() + seconds
    while time.time() < t_end:
        try:
            d, a = s.recvfrom(64)
        except socket.timeout:
            continue
        s.sendto(d + b"|%.9f" % time.time(), a)


def measure(host: str, n: int):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)
    offs, rtts = [], []
    for i in range(n):
        t0 = time.time()
        s.sendto(b"%d" % i, (host, PORT))
        try:
            d, _ = s.recvfrom(128)
        except socket.timeout:
            continue
        t1 = time.time()
        tr = float(d.split(b"|")[1])
        rtt = t1 - t0
        offs.append(tr - (t0 + rtt / 2))
        rtts.append(rtt)
        time.sleep(0.05)
    return offs, rtts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--echo", action="store_true", help="run the echo responder (on the other host)")
    ap.add_argument("--echo-seconds", type=float, default=20.0)
    ap.add_argument("--host", default="192.168.2.1")
    ap.add_argument("--ssh", default="laptop", help="ssh destination used to start the echo remotely")
    ap.add_argument("--no-ssh", action="store_true")
    ap.add_argument("--n", type=int, default=50)
    a = ap.parse_args()
    if a.echo:
        echo(a.echo_seconds)
        return 0
    proc = None
    if not a.no_ssh:
        code = open(__file__).read()
        proc = subprocess.Popen(["ssh", a.ssh, "python3", "-", "--echo", "--echo-seconds", "15"],
                                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        proc.stdin.write(code.encode())
        proc.stdin.close()
        time.sleep(1.5)
    offs, rtts = measure(a.host, a.n)
    if proc is not None:
        proc.wait(timeout=30)
    if not offs:
        print("no replies", file=sys.stderr)
        return 1
    med = statistics.median(offs)
    print(f"samples {len(offs)}  rtt median {statistics.median(rtts) * 1e3:.2f} ms  "
          f"remote-local offset median {med * 1e3:+.2f} ms (min {min(offs) * 1e3:+.2f}, max {max(offs) * 1e3:+.2f})",
          file=sys.stderr)
    print(f"offset_s={med:+.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
