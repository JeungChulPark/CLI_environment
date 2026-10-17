"""clock.py — correct per-frame timestamps for the raw RealSense sessions.

`rgbd_timestamp_associations.json` stamps frames with `associated_host_epoch_timestamp_ns`
under an index-based policy. On the SLAM session, whose colour stream drops frames (100 ms
and 200 ms gaps), those stamps drift from the real frame time by -2.1 s to +3.8 s across
the recording (a linear fit against the bag's own frame times leaves 1.1 s RMS). The SAM
session has no drops and its association stamps are within ~4 ms.

The colour stream's per-frame metadata `timestamp` is consistent with the bag's frame times
to 1.4 ms (SLAM, "Global Time" domain) and 0.1 ms (SAM, "System Time" domain), so it is
the frame clock used here. SAM times are moved onto the SLAM host clock with the peer-clock
offset from SLAM/peer_timestamp_comparison.json.
"""
from __future__ import annotations

import json
import sqlite3
import struct
from pathlib import Path

import numpy as np

SAM_MINUS_SLAM_CLOCK_NS = 18225662057
COLOR = "/device_0/sensor_1/Color_0/image/data"
META = "/device_0/sensor_1/Color_0/image/metadata"


def _str_msg(blob: bytes) -> str:
    n = struct.unpack_from("<I", blob, 4)[0]
    return blob[8:8 + n - 1].decode("utf-8", "replace")


def frame_clock(session_dir: str | Path, cache_dir: str | Path | None = None) -> dict:
    """{'assoc_ns', 'frame_ns', 'domain'} per association index; frame_ns on the SLAM host clock."""
    session_dir = Path(session_dir)
    kind = "SAM" if session_dir.name.upper() == "SAM" else "SLAM"
    if cache_dir is not None:
        cache = Path(cache_dir) / f"clock_{session_dir.parent.name}_{kind}.npz"
        if cache.exists():
            d = np.load(cache)
            return {"assoc_ns": d["assoc_ns"], "frame_ns": d["frame_ns"], "domain": str(d["domain"])}
    db = next(session_dir.glob("*.db3"))
    assoc = json.loads((session_dir / "rgbd_timestamp_associations.json").read_text())["associations"]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    cid = con.execute("select id from topics where name=?", (COLOR,)).fetchone()[0]
    mid = con.execute("select id from topics where name=?", (META,)).fetchone()[0]
    color_ids = [r[0] for r in con.execute("select id from messages where topic_id=? order by id", (cid,))]
    metas = [_str_msg(b) for (b,) in con.execute("select data from messages where topic_id=? order by id", (mid,))]
    if len(color_ids) != len(metas):
        raise ValueError(f"{db.name}: {len(color_ids)} colour frames but {len(metas)} metadata messages")
    pos = {c: i for i, c in enumerate(color_ids)}
    fields = [dict(kv.split("=", 1) for kv in m.split(";") if "=" in kv) for m in metas]
    domain = fields[0].get("timestamp_domain", "?")
    frame = np.array([round(float(fields[pos[a["color_message_id"]]]["timestamp"]) * 1e6) for a in assoc], np.int64)
    host = np.array([int(a["associated_host_epoch_timestamp_ns"]) for a in assoc], np.int64)
    if kind == "SAM":
        frame -= SAM_MINUS_SLAM_CLOCK_NS
        host -= SAM_MINUS_SLAM_CLOCK_NS
    out = {"assoc_ns": host, "frame_ns": frame, "domain": domain}
    if cache_dir is not None:
        Path(cache_dir).mkdir(parents=True, exist_ok=True)
        np.savez(cache, assoc_ns=host, frame_ns=frame, domain=domain)
    return out


def remap_tum(src: str | Path, dst: str | Path, clock: dict, tol_ns: int = 1_000_000) -> tuple[int, int]:
    """Rewrite a TUM trajectory stamped with association times onto the frame clock."""
    assoc, frame = clock["assoc_ns"], clock["frame_ns"]
    order = np.argsort(assoc)
    a_sorted = assoc[order]
    lines, dropped = [], 0
    for line in Path(src).read_text().splitlines():
        p = line.split()
        if len(p) != 8:
            continue
        t = int(round(float(p[0]) * 1e9))
        k = int(np.searchsorted(a_sorted, t))
        best = min((j for j in (k - 1, k) if 0 <= j < len(a_sorted)), key=lambda j: abs(int(a_sorted[j]) - t))
        if abs(int(a_sorted[best]) - t) > tol_ns:
            dropped += 1
            continue
        new_t = int(frame[order[best]])
        lines.append((new_t, " ".join(p[1:])))
    lines.sort()
    Path(dst).write_text("".join(f"{t / 1e9:.9f} {rest}\n" for t, rest in lines))
    return len(lines), dropped
