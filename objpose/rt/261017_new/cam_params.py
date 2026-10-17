#!/usr/bin/env python3
"""cam_params.py — read the colour camera_info out of a session's SLAM bag and print it as slam_stream
`--param` overrides (Camera1.fx/fy/cx/cy and plumb_bob k1 k2 p1 p2 k3), one argument per line.

The 260826 / 260901 / 260915 recordings use three different D455F units with slightly different intrinsics,
so the settings yaml only supplies everything else and the intrinsics always come from the bag itself.

    python3 cam_params.py <session>/SLAM
"""
import sqlite3, struct, sys
from pathlib import Path


def camera_info(db):
    con = sqlite3.connect(f"file:{db}?immutable=1", uri=True)
    tid = con.execute("select id from topics where name like '%/color/camera_info'").fetchone()[0]
    b = bytes(con.execute("select data from messages where topic_id=? limit 1", (tid,)).fetchone()[0])
    o = 4                                           # CDR encapsulation header
    al = lambda o, n: 4 + (o - 4 + n - 1) // n * n   # CDR alignment counts from after the encapsulation header

    def u32(o):
        return struct.unpack_from("<I", b, o)[0], o + 4

    o += 8                                          # header.stamp (sec, nanosec)
    n, o = u32(o); o += n                           # header.frame_id
    o = al(o, 4); o += 8                            # height, width
    n, o = u32(o); model = b[o:o + n - 1].decode(); o += n
    o = al(o, 4); n, o = u32(o)
    o = al(o, 8); d = struct.unpack_from(f"<{n}d", b, o); o += 8 * n
    k = struct.unpack_from("<9d", b, o)
    return model, d, k


if __name__ == "__main__":
    db = next(Path(sys.argv[1]).glob("*.db3"))
    model, d, k = camera_info(db)
    d = list(d) + [0.0] * (5 - len(d))
    vals = {"Camera1.fx": k[0], "Camera1.fy": k[4], "Camera1.cx": k[2], "Camera1.cy": k[5],
            "Camera1.k1": d[0], "Camera1.k2": d[1], "Camera1.p1": d[2], "Camera1.p2": d[3], "Camera1.k3": d[4]}
    print(f"# {db.name}: {model}", file=sys.stderr)
    for key, v in vals.items():
        print("--param"); print(f"{key}={v:.6f}")
