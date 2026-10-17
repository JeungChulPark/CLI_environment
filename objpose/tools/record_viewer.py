#!/usr/bin/env python3
"""record_viewer.py — record the hub's viewer page to an mp4 without touching the desktop.

The viewer is a normal web page, so a headless Chrome renders it offscreen and the DevTools
protocol streams the rendered frames back (`Page.startScreencast`). Nothing appears on the
user's X display, and the recording keeps real time because every frame carries its arrival
timestamp (ffmpeg concat with per-frame durations).

    python3 objpose/tools/record_viewer.py --url http://127.0.0.1:8767 --out demo.mp4 --seconds 120

Stops early when the page is gone or --seconds elapses. WebGL runs on SwiftShader in headless,
which is enough for this map pane; if it still fails the page shows its own notice and the video
keeps the video panel and the tables.
"""
from __future__ import annotations

import argparse
import base64
import json
import shutil
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

import websocket  # websocket-client

CHROME = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")


def wait_page(port: int, timeout_s: float = 30.0):
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/json/list", timeout=2) as r:
                for t in json.load(r):
                    if t.get("type") == "page" and t.get("webSocketDebuggerUrl"):
                        return t["webSocketDebuggerUrl"]
        except Exception:
            pass
        time.sleep(0.3)
    raise RuntimeError("headless Chrome did not expose a page target")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--width", type=int, default=1680)
    ap.add_argument("--height", type=int, default=1050)
    ap.add_argument("--quality", type=int, default=80)
    ap.add_argument("--port", type=int, default=9333)
    ap.add_argument("--settle-s", type=float, default=3.0, help="wait after load before recording")
    ap.add_argument("--keep-frames", action="store_true")
    a = ap.parse_args()
    if CHROME is None:
        raise SystemExit("no chrome/chromium found")

    tmp = Path(tempfile.mkdtemp(prefix="viewer_rec_"))
    frames = tmp / "frames"
    frames.mkdir()
    profile = tmp / "profile"
    cmd = [CHROME, "--headless=new", f"--remote-debugging-port={a.port}", f"--user-data-dir={profile}",
           "--no-first-run", "--no-default-browser-check", "--remote-allow-origins=*", "--disable-extensions", "--mute-audio",
           "--enable-unsafe-swiftshader", "--hide-scrollbars",
           f"--window-size={a.width},{a.height}", a.url]
    chrome = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        ws_url = wait_page(a.port)
        ws = websocket.create_connection(ws_url, timeout=30)
        mid = [0]

        def send(method, params=None):
            mid[0] += 1
            ws.send(json.dumps({"id": mid[0], "method": method, "params": params or {}}))
            return mid[0]

        send("Page.enable")
        send("Emulation.setDeviceMetricsOverride",
             {"width": a.width, "height": a.height, "deviceScaleFactor": 1, "mobile": False})
        time.sleep(a.settle_s)
        send("Page.startScreencast", {"format": "jpeg", "quality": a.quality,
                                      "maxWidth": a.width, "maxHeight": a.height, "everyNthFrame": 1})
        t0 = time.monotonic()
        stamps, n = [], 0
        ws.settimeout(5.0)
        while time.monotonic() - t0 < a.seconds:
            try:
                msg = json.loads(ws.recv())
            except websocket.WebSocketTimeoutException:
                continue
            if msg.get("method") != "Page.screencastFrame":
                continue
            p = msg["params"]
            (frames / f"{n:06d}.jpg").write_bytes(base64.b64decode(p["data"]))
            stamps.append(time.monotonic())
            n += 1
            send("Page.screencastFrameAck", {"sessionId": p["sessionId"]})
        send("Page.stopScreencast")
        ws.close()
    finally:
        chrome.terminate()
        try:
            chrome.wait(timeout=10)
        except subprocess.TimeoutExpired:
            chrome.kill()

    if n < 2:
        raise SystemExit(f"captured only {n} frames — is the page reachable at {a.url}?")
    # per-frame durations keep the video in real time even though screencast fps varies
    lines = []
    for i in range(n):
        d = (stamps[i + 1] - stamps[i]) if i + 1 < n else 0.1
        lines.append(f"file '{frames / f'{i:06d}.jpg'}'\nduration {max(d, 0.01):.3f}")
    lines.append(f"file '{frames / f'{n-1:06d}.jpg'}'")
    lst = tmp / "list.txt"
    lst.write_text("\n".join(lines))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst),
                    "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-pix_fmt", "yuv420p",
                    "-c:v", "libx264", "-crf", "22", "-movflags", "+faststart", str(a.out)], check=True)
    dur = stamps[-1] - stamps[0]
    print(f"[rec] {n} frames, {dur:.1f} s wall ({n/max(dur,1e-3):.1f} fps) -> {a.out}")
    if a.keep_frames:
        print(f"[rec] frames kept in {frames}")
    else:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
