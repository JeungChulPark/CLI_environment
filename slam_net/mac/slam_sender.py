#!/usr/bin/env python3
"""slam_sender.py — 맥(.113)에서 ORB-SLAM3 결과를 .100 의 수신기로 흘려보낸다.

WSL2 가 걸어 둔 역터널 덕분에 맥 입장에서는 **localhost:5765 로 보내면 된다**.
상대가 NAT 뒤에 있다는 것을 몰라도 된다.

세 가지 입력을 받는다.
  --follow <file>   ORB-SLAM3 가 쓰는 CameraTrajectory_live.txt 를 tail 한다 (권장)
  --stdin           파이프로 받은 TUM 줄을 그대로 보낸다
  --demo            가짜 궤적을 만들어 보낸다 (연결 점검용 · SLAM 없이도 된다)

표준 라이브러리만 쓴다. 맥에 파이썬3 만 있으면 된다.

TUM 한 줄:  stamp tx ty tz qx qy qz qw    (미터)
보내는 줄 :  {"type":"pose","frame":n,"stamp":...,"t_mm":[...],"q":[...]}
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time


class Link:
    """끊기면 다시 붙는다. SLAM 은 계속 돌아야 하므로 전송 실패로 죽지 않는다."""

    def __init__(self, host, port, log=print):
        self.addr = (host, port)
        self.log = log
        self.sock = None
        self.dropped = 0
        self.sent = 0

    def _connect(self):
        try:
            s = socket.create_connection(self.addr, timeout=5)
            s.settimeout(5)
            self.sock = s
            self.log(f"[send] 연결 {self.addr[0]}:{self.addr[1]}")
            return True
        except OSError as e:
            self.sock = None
            self.log(f"[send] 연결 실패 {type(e).__name__}: {e}")
            return False

    def send(self, obj):
        line = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        for _ in range(2):
            if self.sock is None and not self._connect():
                self.dropped += 1
                return False
            try:
                self.sock.sendall(line)
                self.sent += 1
                return True
            except OSError:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None          # 한 번만 재접속을 시도한다
        self.dropped += 1
        return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def parse_tum(line):
    p = line.split()
    if len(p) < 8:
        return None
    try:
        v = [float(x) for x in p[:8]]
    except ValueError:
        return None
    return dict(stamp=v[0], t_mm=[v[1] * 1000.0, v[2] * 1000.0, v[3] * 1000.0],
                q=[v[4], v[5], v[6], v[7]])


def follow(path, from_start=False):
    """tail -f. 파일이 잘리거나 새로 만들어져도 따라간다(재실행 대비)."""
    while not os.path.exists(path):
        time.sleep(0.5)
    f = open(path, "r")
    if not from_start:
        f.seek(0, os.SEEK_END)
    ino = os.fstat(f.fileno()).st_ino
    while True:
        line = f.readline()
        if line:
            yield line
            continue
        time.sleep(0.05)
        try:
            if os.stat(path).st_ino != ino or os.stat(path).st_size < f.tell():
                f.close(); f = open(path, "r"); ino = os.fstat(f.fileno()).st_ino
        except FileNotFoundError:
            pass


def demo(n, fps):
    """8자 궤적. 연결 점검용 — SLAM 없이 전송 경로만 확인할 때 쓴다."""
    t0 = time.time()
    for i in range(n):
        u = i / max(1, fps)
        x, z = 0.8 * math.sin(u), 0.8 * math.sin(u) * math.cos(u)
        a = 0.5 * u
        yield dict(stamp=round(t0 + u, 6),
                   t_mm=[x * 1000, 0.0, (z + 1.5) * 1000],
                   q=[0.0, math.sin(a / 2), 0.0, math.cos(a / 2)])
        time.sleep(1.0 / fps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1",
                    help="역터널 덕분에 기본값은 localhost 다")
    ap.add_argument("--port", type=int, default=5765)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--follow", metavar="FILE")
    src.add_argument("--stdin", action="store_true")
    src.add_argument("--demo", type=int, metavar="N")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--from-start", action="store_true")
    ap.add_argument("--session", default=None)
    args = ap.parse_args()

    link = Link(args.host, args.port)
    session = args.session or time.strftime("%Y%m%d-%H%M%S")
    link.send(dict(type="hello", source="mac-orbslam3", session=session,
                   fps=args.fps, host=socket.gethostname()))

    n = 0
    t_start = time.time()
    try:
        if args.demo:
            it = demo(args.demo, args.fps)
        elif args.stdin:
            it = (parse_tum(l) for l in sys.stdin)
        else:
            it = (parse_tum(l) for l in follow(args.follow, args.from_start))
        for rec in it:
            if rec is None:
                continue
            rec.update(type="pose", frame=n)
            link.send(rec)
            n += 1
            if n % 300 == 0:
                el = time.time() - t_start
                link.send(dict(type="stat", frame=n,
                               fps=round(n / el, 2) if el else 0,
                               sent=link.sent, dropped=link.dropped))
                print(f"[send] {n} 프레임  {n/el:.1f} fps  "
                      f"보냄 {link.sent} 실패 {link.dropped}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        link.send(dict(type="bye", frames=n))
        link.close()
        print(f"[send] 종료 — {n} 프레임, 전송 {link.sent}, 실패 {link.dropped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
