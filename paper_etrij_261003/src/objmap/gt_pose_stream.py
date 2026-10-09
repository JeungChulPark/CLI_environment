"""GT camera poses in place of SLAM, for the live object-pose hub (objpose/pc/hub.py).

Speaks the hub's SLAM stream protocol (newline JSON over TCP, as slam_stream / lidar_stream):
    -> {"type":"hello", first_ns, last_ns, n_frames, ...}
    <- {"type":"start","t0_ns":..,"rate":..}
    -> {"type":"pose","t_ns":..,"state":"OK","T_wc":[16], "frame_idx":.., "ref_kf":null, ...}
    -> {"type":"end","processed":..,"dropped":0}

Poses are the re-fitted YCB-V cameras of gt/ycbv/<scene>/cameras.json (camera -> world, metres),
so the hub's world frame is the GT world frame (use the identity extrinsic). Stamps follow
ycbv_live/make_bags.py: frame n of the sorted sequence is at T0 + n/30 s. Each pose is sent
when its frame is due on the replay clock, like a SLAM backend with zero tracking time; no
keyframes, so the hub anchors objects in world coordinates.

    python gt_pose_stream.py --scene 52 --connect 127.0.0.1:17001
"""
import argparse
import json
import socket
import time

from common import GT_DIR

T0 = 1_700_000_000_000_000_000      # make_bags.py


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", type=int, required=True)
    ap.add_argument("--connect", default="127.0.0.1:17001")
    ap.add_argument("--gt", default="ycbv")
    a = ap.parse_args()
    cams = json.load(open(GT_DIR / a.gt / f"{a.scene:06d}" / "cameras.json"))
    frames = sorted(cams["frames"], key=lambda f: f["id"])
    stamps = [T0 + int(round(n * 1e9 / 30)) for n in range(len(frames))]

    host, port = a.connect.rsplit(":", 1)
    for _ in range(300):
        try:
            sock = socket.create_connection((host, int(port)))
            break
        except OSError:
            time.sleep(1.0)
    else:
        raise SystemExit("hub not reachable")
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    send = lambda o: sock.sendall((json.dumps(o) + "\n").encode())
    send({"type": "hello", "source": "gt_poses", "features": None, "session": f"ycbv {a.scene:06d} GT",
          "first_ns": stamps[0], "last_ns": stamps[-1], "n_frames": len(frames),
          "time_source": "frame", "time_domain": "make_bags synthetic 30 fps", "kf_updates": False})
    buf = b""
    while True:
        chunk = sock.recv(65536)
        if not chunk:
            raise SystemExit("hub closed before start")
        buf += chunk
        if b"\n" in buf:
            msg = json.loads(buf.split(b"\n", 1)[0])
            if msg.get("type") == "start":
                break
            buf = buf.split(b"\n", 1)[1]
    t0, rate = int(msg["t0_ns"]), float(msg.get("rate", 1.0))
    wall0 = time.monotonic()
    print(f"[gt_pose_stream] scene {a.scene:06d} start t0_ns={t0} rate={rate}", flush=True)
    sent = 0
    for n, (f, t) in enumerate(zip(frames, stamps)):
        if t < t0 - int(1e9):      # keep 1 s before t0 for interpolation
            continue
        due = wall0 + (t - t0) / 1e9 / rate
        while time.monotonic() < due:
            time.sleep(min(0.01, max(0.0, due - time.monotonic())))
        T = [round(float(v), 7) for row in f["T_wc"] for v in row]
        send({"type": "pose", "t_ns": t, "state": "OK", "T_wc": T, "track_ms": 0.0, "frame_idx": n,
              "dropped": 0, "map_id": 0, "map_changed": False, "ref_kf": None, "T_w_kf": None, "kf_map_id": None})
        sent += 1
    send({"type": "end", "processed": sent, "dropped": 0})
    print(f"[gt_pose_stream] done: {sent} poses", flush=True)
    time.sleep(1.0)
    sock.close()


if __name__ == "__main__":
    main()
