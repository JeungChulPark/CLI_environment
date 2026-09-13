#!/usr/bin/env python3
"""slam_receiver.py — 맥(.113)이 보내는 ORB-SLAM3 포즈 스트림을 받아 저장한다.

이 PC(192.168.219.100)에서 돌린다. 표준 라이브러리만 쓴다 — 어느 파이썬에서도
바로 뜨게 하려는 것이다(conda 환경을 활성화하지 않아도 된다).

받는 것: `\n` 으로 끝나는 JSON 한 줄이 한 메시지.
남기는 것 (세션 폴더 하나에):
    raw.jsonl                받은 줄 그대로. 원본이 곧 증거다.
    CameraTrajectory.txt     TUM 형식 `stamp tx ty tz qx qy qz qw` (미터)
    session.json             세션 요약 (프레임 수, 지연 통계, 끊김 횟수)

설계 원칙
--------
* **끊겨도 죽지 않는다.** 맥이 재접속하면 같은 세션 폴더에 이어 쓴다.
* **받은 것을 먼저 남긴다.** 파싱에 실패해도 raw.jsonl 에는 들어간다. 나중에
  형식이 바뀌어도 데이터를 잃지 않는다.
* **flush 를 미루지 않는다.** 중간에 프로세스가 죽어도 그때까지가 남는다.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import socketserver
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

STATE_NAME = {0: "NO_IMAGES_YET", 1: "NOT_INITIALIZED", 2: "OK",
              3: "RECENTLY_LOST", 4: "LOST", 5: "OK_KLT"}


class Session:
    """한 세션의 파일과 통계. 여러 접속이 같은 세션에 이어 쓸 수 있다."""

    def __init__(self, root: Path, name: str, log=print):
        self.dir = root / name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.log = log
        self.name = name
        self._lock = threading.Lock()
        self._raw = open(self.dir / "raw.jsonl", "a", buffering=1)
        traj = self.dir / "CameraTrajectory.txt"
        new = not traj.exists() or traj.stat().st_size == 0
        self._traj = open(traj, "a", buffering=1)
        if new:
            self._traj.write("# stamp tx ty tz qx qy qz qw   (meters, TUM)\n")
        self.started = time.time()
        self.frames = 0
        self.connects = 0
        self.bad_lines = 0
        self.by_state = {}
        self.track_ms = []
        self.last_frame = None

    def on_connect(self, peer):
        with self._lock:
            self.connects += 1
        self.log(f"[recv] 접속 {peer}  (세션 {self.name}, {self.connects}번째)")

    def feed(self, line: str):
        with self._lock:
            self._raw.write(line if line.endswith("\n") else line + "\n")
        try:
            m = json.loads(line)
        except Exception:
            with self._lock:
                self.bad_lines += 1
            return
        t = m.get("type")
        if t == "pose":
            self._pose(m)
        elif t == "hello":
            self.log(f"[recv] hello  source={m.get('source')} "
                     f"session={m.get('session')} fps={m.get('fps')}")
        elif t == "bye":
            self.log(f"[recv] bye  frames={m.get('frames')}")

    def _pose(self, m):
        with self._lock:
            self.frames += 1
            self.last_frame = m.get("frame")
            st = m.get("state")
            if st is not None:
                k = STATE_NAME.get(st, str(st))
                self.by_state[k] = self.by_state.get(k, 0) + 1
            ms = m.get("track_ms")
            if isinstance(ms, (int, float)):
                self.track_ms.append(float(ms))
            tm = m.get("t_mm")
            q = m.get("q")
            stamp = m.get("stamp")
            if (isinstance(tm, list) and len(tm) == 3
                    and isinstance(q, list) and len(q) == 4 and stamp is not None):
                self._traj.write(
                    f"{float(stamp):.6f} "
                    f"{tm[0]/1000.0:.9f} {tm[1]/1000.0:.9f} {tm[2]/1000.0:.9f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")

    def summary(self):
        import statistics as st
        with self._lock:
            a = sorted(self.track_ms)
            d = dict(session=self.name, frames=self.frames,
                     connects=self.connects, bad_lines=self.bad_lines,
                     elapsed_s=round(time.time() - self.started, 1),
                     last_frame=self.last_frame, by_state=dict(self.by_state))
            if a:
                d["track_ms"] = dict(
                    mean=round(sum(a) / len(a), 2),
                    p50=round(a[len(a) // 2], 2),
                    p95=round(a[min(len(a) - 1, int(len(a) * 0.95))], 2),
                    max=round(a[-1], 2),
                    over_33_33=sum(1 for v in a if v > 33.33))
            return d

    def close(self):
        d = self.summary()
        (self.dir / "session.json").write_text(json.dumps(d, indent=1))
        with self._lock:
            self._raw.close(); self._traj.close()
        return d


class Handler(socketserver.StreamRequestHandler):
    timeout = 120

    def handle(self):
        sess: Session = self.server.session
        sess.on_connect(self.client_address)
        buf = b""
        try:
            while True:
                chunk = self.request.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if line.strip():
                        sess.feed(line.decode("utf-8", "replace"))
        except (socket.timeout, ConnectionResetError, OSError) as e:
            sess.log(f"[recv] 연결 종료 {self.client_address}: {type(e).__name__}")
        finally:
            sess.log(f"[recv] 끊김 {self.client_address}  누적 {sess.frames} 프레임")


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5765)
    ap.add_argument("--out", default=str(Path.home() / "slam_results"))
    ap.add_argument("--session", default=None, help="기본: 시각으로 자동 생성")
    ap.add_argument("--pidfile", default=None)
    args = ap.parse_args()

    name = args.session or datetime.now().strftime("%Y%m%d-%H%M%S")
    sess = Session(Path(args.out), name)
    srv = Server((args.host, args.port), Handler)
    srv.session = sess

    if args.pidfile:
        Path(args.pidfile).write_text(str(os.getpid()))

    print(f"[recv] 대기 {args.host}:{args.port}")
    print(f"[recv] 저장 {sess.dir}")
    sys.stdout.flush()

    stop = threading.Event()

    def bye(_s, _f):
        stop.set()
        threading.Thread(target=srv.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)

    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        while not stop.is_set():
            stop.wait(5)
    finally:
        srv.server_close()
        d = sess.close()
        print("\n[recv] 세션 요약")
        print(json.dumps(d, indent=1, ensure_ascii=False))
        if args.pidfile:
            try:
                os.unlink(args.pidfile)
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
