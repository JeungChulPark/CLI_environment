#!/usr/bin/env python3
"""replay_run.py — play a finished live run again in the same web viewer (pc/web/index.html), no robot or camera needed.

    python objpose/pc/replay_run.py objpose/output/<run dir> [--port 8767] [--speed 1.0] [--loop] [--start 290]
    -> http://<this PC>:8767      (the page gets a bar: pause, seek, speed)

Navigation pane ("주행 지도"), best first:
  * <run>/nav.jsonl.gz — what the hub recorded from the Nav2 bridge (hub.py --nav-bridge with --record-events-hz):
    robot, scan, plan, footprint, goal state 5 times a second, the map and the local costmap whenever they changed.
  * otherwise, for fixed-map runs: the saved map (--map, default nav/real/maps/globalmap_261002.yaml) with the robot
    from the recorded pose and the goals found in <run>/nav_laptop/*.log. No scan, costmap or plan in that case.

Two sources, best first:
  * <run>/events.jsonl.gz  — the viewer events the hub recorded (hub.py --record-events-hz): identical picture,
    camera image with overlays included.
  * otherwise the hub's logs: slam_poses.jsonl (robot pose), display_objects.jsonl (objects in the camera frame at
    every displayed frame), X_base_sam.json / the lane extrinsic, summary.json. The 3D view (robot, camera, objects,
    trajectory) is rebuilt; the camera image was not stored, so the video pane shows a placeholder.
"""
import argparse, base64, bisect, gzip, json, math, queue, re, struct, sys, threading, time, zlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def m16(T):
    return None if T is None else [round(float(v), 6) for v in np.asarray(T).reshape(-1)]


def from_logs(run: Path):
    """[(t_ns, payload)] rebuilt from the hub logs"""
    try:
        from object_extents import load_object_extents
        ext = load_object_extents()
    except Exception:                                    # noqa: BLE001
        ext = {}
    summary = json.loads((run / "summary.json").read_text()) if (run / "summary.json").exists() else {}
    lane = (summary.get("lanes") or [{"name": "slam", "label": "SLAM", "color": "#f2a03d", "backend": "lidar"}])[0]
    P = [json.loads(l) for l in open(run / "slam_poses.jsonl")]
    P = [p for p in P if p.get("T_wc")]
    pt = [p["t_ns"] for p in P]
    pT = [np.array(p["T_wc"]).reshape(4, 4) for p in P]
    xf = run / "X_base_sam.json"
    X = np.array(json.loads(xf.read_text())["T_slam_sam"]) if xf.exists() else np.eye(4)
    frames = {}
    for l in open(run / "display_objects.jsonl"):
        d = json.loads(l)
        frames[d["t_ns"]] = d                            # the same frame is logged again while it stays on screen
    img = np.full((480, 640, 3), 28, np.uint8)
    cv2.putText(img, "camera image was not recorded", (70, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (170, 170, 170), 2)
    cv2.putText(img, "3D view rebuilt from the logs", (110, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 1)
    jpg = base64.b64encode(cv2.imencode(".jpg", img)[1].tobytes()).decode()
    t0 = summary.get("t0_ns") or (pt[0] if pt else 0)
    stats0 = {k: v for k, v in summary.items() if k not in ("objects", "observations_final", "memory_landmarks")}
    out, last_traj = [], -1e18
    for t_ns in sorted(frames):
        d = frames[t_ns]
        k = bisect.bisect_right(pt, t_ns) - 1
        if k + 1 < len(pt) and abs(pt[k + 1] - t_ns) < abs(pt[max(k, 0)] - t_ns):
            k += 1
        T_wslam = pT[k] if 0 <= k < len(pt) and abs(pt[k] - t_ns) < 500_000_000 else None
        T_ws = None if T_wslam is None else T_wslam @ X
        objs = []
        for o in d["objects"]:
            if T_ws is None or not o.get("T_cam_obj"):
                continue
            T_w_obj = T_ws @ np.array(o["T_cam_obj"]).reshape(4, 4)
            e = ext.get(o["name"])
            objs.append({"name": o["name"], "source": o.get("source"), "age_s": o.get("age_s"), "score": 0.0, "est_count": 0,
                         "anchor_kf": None, "near_loop": False, "status": None, "instance": None, "multi": False,
                         "confidence": None, "T_w_obj": m16(T_w_obj), "T_w_obj_lane": m16(T_w_obj), "T_cam_obj": o["T_cam_obj"],
                         "size_mm": None if e is None else e.get("size_mm"),
                         "center_mm": None if e is None else (np.add(e["min_mm"], e["max_mm"]) / 2).tolist(), "history": []})
        lf = {"name": lane.get("name", "slam"), "label": lane.get("label", "SLAM"), "color": lane.get("color", "#f2a03d"),
              "backend": lane.get("backend"), "jpeg": jpg, "slam_status": d.get("slam"), "T_w_sam": m16(T_ws),
              "T_w_slam": m16(T_wslam), "slam_frame": "base_link" if xf.exists() else None, "objects": objs, "divergence_cm": None}
        if t_ns - last_traj > 1_000_000_000:
            last_traj = t_ns
            lf["trajectory"] = [T[:3, 3].round(4).tolist() for T in pT[:max(k, 0) + 1:3]]
        st = dict(stats0, status="replay (logs)", replay_t_rel_s=round((t_ns - t0) / 1e9, 2), lanes=[dict(lane)],
                  reference_lane=lane.get("name", "slam"), world_zup=bool(summary.get("world_zup", True)))
        out.append((t_ns, {"type": "frame", "t_ns": t_ns, "t_rel_s": round((t_ns - t0) / 1e9, 3), "frame_idx": d.get("frame_idx"),
                           "lanes": [lf], "jpeg": jpg, "slam_status": lf["slam_status"], "T_w_sam": lf["T_w_sam"],
                           "T_w_slam": lf["T_w_slam"], "objects": objs, "stats": st}))
    init = {"type": "init", "K": [[384.5, 0, 320.8], [0, 383.9, 243.4], [0, 0, 1]], "width": 640, "height": 480,
            "X_slam_sam": m16(X), "trajectory": [],
            "lanes": [{"name": lane.get("name", "slam"), "label": lane.get("label", "SLAM"), "color": lane.get("color", "#f2a03d"),
                       "backend": lane.get("backend"), "X_slam_sam": m16(X), "host": "replay", "link": ""}],
            "reference_lane": lane.get("name", "slam"), "world_zup": bool(summary.get("world_zup", True))}
    return init, out


RE_REL = re.compile(r'"t_rel_s":(-?[0-9.]+)')


def from_events(path: Path):
    """(init, frames(start_rel_s) iterator factory, scan()). The file is read while it plays (a long run is hundreds of
    MB) and a file cut off by a killed hub (no gzip end marker) plays up to its last complete line."""
    def raw():
        try:
            with gzip.open(path, "rt") as f:
                for l in f:
                    yield l
        except EOFError:
            return

    init = None
    for l in raw():
        if l.startswith('{"type":"init"'):
            init = json.loads(l)
            break

    def frames(start=0.0):
        for l in raw():
            if not l.startswith('{"type":"frame"'):
                continue
            m = RE_REL.search(l, 0, 200)
            if m and float(m.group(1)) < start:            # seeking: skip without parsing the image
                continue
            try:
                p = json.loads(l)
            except ValueError:
                return
            p.setdefault("stats", {})["status"] = "replay (recorded)"
            yield p["t_ns"], p

    def scan():
        last = 0.0
        for l in raw():
            m = RE_REL.search(l, 0, 200)
            if m:
                last = float(m.group(1))
        return last
    return init, frames, scan


def png_rgba(a):
    h, w = a.shape[:2]
    raw = np.concatenate([np.zeros((h, 1), np.uint8), a.reshape(h, w * 4)], 1).tobytes()
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b""))


class RecordedNav:
    """<run>/nav.jsonl.gz: lines {"t_ns", "state", ["meta", "map_png"], ["costmap_png"]} in time order"""
    def __init__(self, path):
        self.path = path
        self.reset()

    def reset(self):
        self.state = self.meta = self.map_png = self.cm_png = None
        self._it = self._lines()
        self._next = next(self._it, None)

    def _lines(self):
        try:
            with gzip.open(self.path, "rt") as f:
                for l in f:
                    try:
                        yield json.loads(l)
                    except ValueError:
                        return
        except EOFError:
            return

    def advance(self, t_ns, frame=None):
        while self._next is not None and self._next["t_ns"] <= t_ns:
            d = self._next
            self.state = d["state"]
            if "meta" in d:
                self.meta, self.map_png = d["meta"], base64.b64decode(d["map_png"])
            if "costmap_png" in d:
                self.cm_png = base64.b64decode(d["costmap_png"])
            self._next = next(self._it, None)


class MapOnlyNav:
    """no nav recording: the saved map + the robot from the recorded pose + goals from the laptop's Nav logs"""
    def __init__(self, map_yaml: Path, run: Path):
        import yaml
        y = yaml.safe_load(map_yaml.read_text())
        g = cv2.imread(str(map_yaml.parent / y["image"]), cv2.IMREAD_UNCHANGED)
        occ = (255 - g.astype(np.float32)) / 255.0
        if y.get("negate"):
            occ = 1 - occ
        g = np.flipud(g)
        occ = np.flipud(occ)                                 # row 0 = lowest y, as the bridge sends it
        a = np.zeros(g.shape + (4,), np.uint8)
        a[occ < y["free_thresh"]] = (38, 51, 64, 255)
        a[occ > y["occupied_thresh"]] = (214, 224, 232, 255)
        h, w = g.shape
        res, (ox, oy) = y["resolution"], y["origin"][:2]
        js, is_ = np.nonzero(occ > y["occupied_thresh"])
        self.meta = {"w": w, "h": h, "res": res, "ox": ox, "oy": oy, "ver": 1,
                     "known": [float(ox + is_.min() * res - 1), float(oy + js.min() * res - 1),
                               float(ox + (is_.max() + 1) * res + 1), float(oy + (js.max() + 1) * res + 1)]}
        self.map_png, self.cm_png, self.state = png_rgba(a), None, None
        self.events = []                                     # (t_ns, "goal", [x, y, yaw]) / (t_ns, "done"|"failed")
        for f in sorted((run / "nav_laptop").glob("*.log")) if (run / "nav_laptop").is_dir() else []:
            for l in f.read_text(errors="replace").splitlines():
                m = re.search(r"\[(\d+\.\d+)\].*goal from the viewer: \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+) deg\)", l)
                if m:
                    self.events.append((int(float(m.group(1)) * 1e9), "goal", [float(m.group(2)), float(m.group(3)), math.radians(float(m.group(4)))]))
                m = re.search(r"\[(\d+\.\d+)\].*bt_navigator\]: (Goal succeeded|Goal failed|Goal canceled|Goal was canceled)", l)
                if m:
                    self.events.append((int(float(m.group(1)) * 1e9), "succeeded" if "succeeded" in m.group(2) else "aborted", None))
        self.events.sort(key=lambda e: e[0])

    def reset(self):
        self.state = None

    def advance(self, t_ns, frame=None):
        T = frame and frame["lanes"][0].get("T_w_slam")
        if not T:
            return
        x, y, yaw = T[3], T[7], math.atan2(T[4], T[0])
        c, s = math.cos(yaw), math.sin(yaw)
        fp = [[round(x + c * px - s * py, 3), round(y + s * px + c * py, 3)] for px, py in ((0.52, 0.38), (0.52, -0.38), (-0.52, -0.38), (-0.52, 0.38))]
        status, goal = "idle", None
        for t, kind, g in self.events:
            if t > t_ns:
                break
            status, goal = ("executing", g) if kind == "goal" else (kind, goal)
        self.state = {"robot": [round(x, 3), round(y, 3), round(yaw, 4)], "pose_age_s": 0.0, "map_ver": 1, "loc_event": None,
                      "nav": {"status": status, "distance_remaining": None, "goal": goal, "jog": None},
                      "plan": [], "scan": [], "footprint": fp, "costmap": None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--port", type=int, default=8767)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=0.0, help="start at this time of the run [s]")
    ap.add_argument("--loop", action="store_true", help="start again from the beginning when the run ends")
    ap.add_argument("--map", default=str(HERE.parent / "nav" / "real" / "maps" / "globalmap_261002.yaml"),
                    help="saved map for the navigation pane when the run has no nav.jsonl.gz (fixed-map runs only)")
    a = ap.parse_args()
    run = Path(a.run)
    ev = run / "events.jsonl.gz"
    if ev.exists():
        init, frames, scan = from_events(ev)
    else:
        init, flist = from_logs(run)
        frames = lambda start=0.0: ((t, p) for t, p in flist if p["t_rel_s"] >= start)
        scan = lambda: flist[-1][1]["t_rel_s"] if flist else 0.0
    if init is None:
        sys.exit("nothing to replay in " + str(run))
    nav = None
    if (run / "nav.jsonl.gz").exists():
        nav = RecordedNav(run / "nav.jsonl.gz")
        # the navigation record can outlive the camera record (the hub's picture thread died at 12:09 on 2026-10-14 while
        # the robot drove on until 12:13): after the last picture keep its image and carry the timeline on to the end of
        # nav.jsonl.gz, so the navigation pane still plays
        nav_end = None
        for d in RecordedNav(run / "nav.jsonl.gz")._lines():
            nav_end = d["t_ns"]
        if nav_end is not None:
            img_frames, img_scan = frames, scan

            def frames(start=0.0, img_frames=img_frames, nav_end=nav_end):
                last = None
                for t_ns, p in img_frames(start):
                    last = (t_ns, p); yield t_ns, p
                if last is None:                                  # seek past the last picture: find it once
                    for last in img_frames(0.0):
                        pass
                if last is None or nav_end <= last[0]:
                    return
                t_ns, p = last; rel = float(p.get("t_rel_s", 0.0)); q = dict(p)
                q.setdefault("stats", {}); q["stats"] = dict(q["stats"], status="replay (camera record ended: navigation only)")
                while t_ns + 200_000_000 <= nav_end:
                    t_ns += 200_000_000; rel += 0.2
                    if rel < start:
                        continue
                    q = dict(q, t_ns=t_ns, t_rel_s=round(rel, 3)); yield t_ns, q

            def scan(img_scan=img_scan, img_frames=img_frames, nav_end=nav_end):
                first = next(img_frames(0.0), None)              # t0 of the run = first frame time - its t_rel
                if first is None:
                    return img_scan()
                t0 = first[0] - float(first[1].get("t_rel_s", 0.0)) * 1e9
                return max(img_scan(), (nav_end - t0) / 1e9)
    elif "loc-nav" in run.name and Path(a.map).exists():
        nav = MapOnlyNav(Path(a.map), run)
    print(f"[replay] {run.name}: {'recorded events (image included)' if ev.exists() else 'logs (no camera image)'}; navigation pane: "
          f"{'recorded' if isinstance(nav, RecordedNav) else 'saved map + robot only' if nav else 'none'}", flush=True)
    clients, lock, stop = [], threading.Lock(), threading.Event()
    ctl = {"paused": False, "seek": None, "speed": float(a.speed), "t_rel": 0.0, "duration": None, "rebase": False}
    threading.Thread(target=lambda: ctl.__setitem__("duration", round(scan(), 1)), daemon=True).start()
    page = HERE / "web" / "index.html"

    def play():
        start = float(a.start)
        while not stop.is_set():
            if nav:
                nav.reset()
            wall0 = base = None
            for t_ns, p in frames(start):
                rel = float(p.get("t_rel_s", 0.0))
                if base is None:
                    base, wall0 = rel, time.monotonic()
                while not stop.is_set() and ctl["seek"] is None:
                    if ctl["rebase"]:                          # speed changed: keep the position, change the pace
                        ctl["rebase"] = False
                        base, wall0 = ctl["t_rel"], time.monotonic()
                    if ctl["paused"]:
                        time.sleep(0.05)
                        wall0 += 0.05
                        continue
                    wait = wall0 + (rel - base) / ctl["speed"] - time.monotonic()
                    if wait <= 0:
                        break
                    time.sleep(min(0.02, wait))
                if stop.is_set() or ctl["seek"] is not None:
                    break
                if nav:
                    nav.advance(t_ns, p)
                ctl["t_rel"] = rel
                data = ("data: " + json.dumps(p, separators=(",", ":")) + "\n\n").encode()
                with lock:
                    for q in clients:
                        if q.qsize() < 5:
                            q.put(data)
            else:                                            # reached the end
                if not a.loop:
                    while ctl["seek"] is None and not stop.is_set():
                        time.sleep(0.1)
            start, ctl["seek"] = (ctl["seek"] if ctl["seek"] is not None else 0.0), None
    threading.Thread(target=play, daemon=True).start()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, body, ctype="application/json", code=HTTPStatus.OK):
            if not isinstance(body, (bytes, bytearray)):
                body = json.dumps(body, separators=(",", ":")).encode()
            self.send_response(code); self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store"); self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = self.path.split("?")[0]
            try:
                d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0) or 0)) or b"{}")
            except ValueError:
                d = {}
            if path == "/replay/ctl":
                if "pause" in d:
                    ctl["paused"] = bool(d["pause"])
                if "speed" in d:
                    ctl["speed"], ctl["rebase"] = min(16.0, max(0.1, float(d["speed"]))), True
                if "seek" in d:
                    ctl["seek"] = max(0.0, float(d["seek"]))
                self.reply({"ok": True})
            elif path.startswith("/nav/"):
                self.reply({"ok": False, "error": "재생 중에는 로봇을 움직일 수 없습니다"})
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/replay/state":
                self.reply({k: ctl[k] for k in ("paused", "speed", "t_rel", "duration")}); return
            if path.startswith("/nav/"):
                if nav is None or nav.state is None:
                    self.send_error(HTTPStatus.NOT_FOUND)
                elif path == "/nav/state":
                    self.reply(dict(nav.state, replay=True))
                elif path == "/nav/meta":
                    self.reply(nav.meta or {"ver": 0})
                elif path == "/nav/map.png" and nav.map_png:
                    self.reply(nav.map_png, "image/png")
                elif path == "/nav/costmap.png" and nav.cm_png:
                    self.reply(nav.cm_png, "image/png")
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
                return
            if self.path.startswith("/vendor/"):
                root = (HERE / "web" / "vendor").resolve()
                f = (HERE / "web" / self.path.lstrip("/").split("?")[0]).resolve()
                if root not in f.parents or not f.is_file():
                    self.send_error(HTTPStatus.NOT_FOUND); return
                self.reply(f.read_bytes(), "text/javascript; charset=utf-8")
            elif path in ("/", "/index.html"):
                self.reply(page.read_bytes(), "text/html; charset=utf-8")
            elif path == "/events":
                self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store"); self.end_headers()
                q = queue.Queue()
                with lock:
                    clients.append(q)
                try:
                    self.wfile.write(("data: " + json.dumps(init) + "\n\n").encode())
                    while not stop.is_set():
                        try:
                            self.wfile.write(q.get(timeout=5))
                        except queue.Empty:
                            self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with lock:
                        clients.remove(q)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

    srv = ThreadingHTTPServer((a.host, a.port), H); srv.daemon_threads = True
    print(f"[replay] viewer: http://localhost:{a.port}  (speed x{a.speed})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        stop.set()


if __name__ == "__main__":
    main()
