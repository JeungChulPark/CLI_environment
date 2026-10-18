#!/usr/bin/env python3
"""hub_record.py — stand-in for the object-pose hub that records one streamer run (hdl_stream.py, lidar_stream.py,
slam_stream --mode live) instead of displaying it.

Listens on --port, accepts one connection, answers the streamer's hello with {"type":"start","t0_ns":first_ns,"rate":R},
writes every received line to <out>/stream.jsonl, and on "end" writes two TUM files (T_world_sensor, state OK):
    live_tum.txt    the pose the streamer sent at that moment
    final_tum.txt   the same pose re-anchored to its reference keyframe's LAST estimate
                    (T_w_kf_final(ref) . T_w_kf_then(ref)^-1 . T_wc), i.e. the end-of-run optimised trajectory

    python3 hub_record.py --port 17101 --out DIR [--rate 1.0]
"""
from __future__ import annotations
import argparse, json, socket
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def tum_line(t_ns, T):
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    return f"{t_ns / 1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rate", type=float, default=1.0)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.port)); srv.listen(1)
    print(f"[hub_record] listening on 127.0.0.1:{a.port}", flush=True)
    con, _ = srv.accept()
    f = con.makefile("rb")
    poses, kfs_final, n = [], {}, 0
    with open(out / "stream.jsonl", "wb") as log:
        for raw in f:
            log.write(raw); n += 1
            d = json.loads(raw)
            t = d.get("type")
            if t == "hello":
                con.sendall((json.dumps({"type": "start", "t0_ns": int(d["first_ns"]), "rate": a.rate}) + "\n").encode())
                print(f"[hub_record] hello from {d.get('source')}: {d.get('n_frames')} frames", flush=True)
            elif t == "pose" and d.get("state") == "OK" and d.get("T_wc"):
                poses.append(d)
            elif t == "kf_update":
                for k in d.get("kfs", []):
                    kfs_final[int(k[0])] = np.array(k[1:17], float).reshape(4, 4)
            elif t == "end":
                print(f"[hub_record] end: {json.dumps(d)}", flush=True)
                break
    con.close(); srv.close()
    with open(out / "live_tum.txt", "w") as lf, open(out / "final_tum.txt", "w") as ff:
        moved = 0
        for d in poses:
            T = np.array(d["T_wc"], float).reshape(4, 4)
            lf.write(tum_line(d["t_ns"], T))
            ref = d.get("ref_kf")
            if ref is not None and d.get("T_w_kf") and int(ref) in kfs_final:
                T = kfs_final[int(ref)] @ np.linalg.inv(np.array(d["T_w_kf"], float).reshape(4, 4)) @ T
                moved += 1
            ff.write(tum_line(d["t_ns"], T))
    print(f"[hub_record] {n} lines, {len(poses)} OK poses ({moved} re-anchored), {len(kfs_final)} keyframes -> {out}", flush=True)


if __name__ == "__main__":
    main()
