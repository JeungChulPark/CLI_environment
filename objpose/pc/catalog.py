#!/usr/bin/env python3
"""catalog.py — which datasets and SLAM backends this rig can actually run, and with what.

The viewer offers 날짜 > 데이터셋 > 백엔드, and a backend can only be offered if every piece
it needs is in place. The pieces live on two machines and in two places in this repo:

  * the SLAM source (SLAM/ for the cameras, lidar/ for the Velodyne) is replayed ON THE MAC,
    so the Mac must hold the dataset;
  * SAM/ is replayed on this PC and fed to SAM-6D, so this PC must hold it;
  * the rig extrinsic that puts the SAM camera against that backend's SLAM sensor must exist
    (rt/<date>/X_slam_sam_<date>.json for the cameras, rt/<date>/X_lidar_sam.json for the
    Velodyne), and gyro-aided ORB-SLAM3 additionally needs imu/ and a fitted rig json.

Extrinsics are per rig-day, not per recording: a dataset with no extrinsic of its own borrows
another recording's from the same date. That is how the rig actually behaves — but the clock
offset in it (sam_tau_s) was measured on that other recording, so such a run is marked
`borrowed` and the viewer says so rather than passing it off as measured here.
"""
from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DATASET_ROOT = Path("/home/jucpark/DeepLearning/Dataset")
MAC_ROOT = "~/Documents/DefenseMeta/Dataset"
RT = REPO / "objpose" / "rt"
SETTINGS = REPO / "objpose" / "mac_slam" / "settings"


@dataclass(frozen=True)
class Backend:
    id: str
    label: str
    slam: str                 # hub.py --slam value
    source: str               # dataset folder the SLAM host replays
    lidar_world: bool
    gyro: bool = False


BACKENDS = (
    Backend("orbslam3", "ORB-SLAM3", "orbslam3", "SLAM", False),
    Backend("orbslam3_imu", "ORB-SLAM3 + IMU", "orbslam3", "SLAM", False, gyro=True),
    Backend("lidar", "KISS-ICP", "lidar", "lidar", True),
    Backend("hdl", "hdl_graph_slam", "hdl", "lidar", True),
)
BY_ID = {b.id: b for b in BACKENDS}
# ORB-SLAM3 extractor sizes the viewer offers. slam_stream rewrites ORBextractor.nFeatures in
# whatever settings file it is given, so the nf2000 settings serve every size.
FEATURES = (2000, 3000, 4000)
DEFAULT_FEATURES = 2000


def date_of(name: str) -> str:
    m = re.match(r"(\d{6})", name)
    return m.group(1) if m else "기타"


def extrinsic_for(name: str, b: Backend) -> tuple[Path | None, str]:
    """(path, provenance) — the rig file for this dataset and backend.

    provenance is "fitted" when the file records that it was fitted on THIS recording,
    "borrowed" when it names a different one from the same rig-day (the geometry carries
    over, the clock offset in it does not), and "unknown" for files fitted before the
    estimators started recording which dataset they ran on.
    """
    d = date_of(name)
    own = RT / d / ("X_lidar_sam.json" if b.lidar_world else f"X_slam_sam_{d}.json")
    if own.exists():
        try:
            fitted_on = json.loads(own.read_text()).get("dataset")
        except Exception:
            fitted_on = None
        return own, "unknown" if not fitted_on else "fitted" if fitted_on == name else "borrowed"
    legacy = RT / "X_slam_sam.json"                      # the 260826 rig, before rt/<date>/
    if not b.lidar_world and legacy.exists() and d == "260826":
        return legacy, "unknown"
    return None, "none"


def mac_datasets(host: str = "mac", timeout: float = 20.0) -> dict[str, set[str]] | None:
    """{dataset: {streams}} on the SLAM host, or None when it cannot be reached."""
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={int(timeout)}", host,
           f"cd {MAC_ROOT} && for d in */; do n=${{d%/}}; "
           f'for s in SLAM SAM lidar imu; do [ -d "$n/$s" ] && echo "$n $s"; done; done']
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 10)
    except (subprocess.TimeoutExpired, OSError):
        return None
    found: dict[str, set[str]] = {}
    for line in out.stdout.split("\n"):
        parts = line.split()
        if len(parts) == 2:
            found.setdefault(parts[0], set()).add(parts[1])
    # the loop's exit status is that of its last `[ -d ]`, so a non-zero code is normal here;
    # what matters is whether anything came back
    return found or None


def streams(d: Path) -> set[str]:
    return {s for s in ("SLAM", "SAM", "lidar", "imu") if (d / s).is_dir()}


def backend_state(name: str, here: set[str], there: set[str] | None, b: Backend) -> dict:
    """Whether this backend can run on this dataset, and if not, the first missing piece."""
    x, provenance = extrinsic_for(name, b)
    why = None
    if b.source not in (there if there is not None else here):
        why = f"Mac 에 {name}/{b.source} 없음"
    elif "SAM" not in here:
        why = f"이 PC 에 {name}/SAM 없음 (SAM-6D 입력)"
    elif x is None:
        why = f"rt/{date_of(name)}/ 에 이 백엔드용 RT 없음"
    elif b.gyro:
        if "imu" not in here:
            why = f"이 PC 에 {name}/imu 없음"
        elif not (SETTINGS / f"rig_{date_of(name)}_gyro.json").exists():
            why = f"settings/rig_{date_of(name)}_gyro.json 없음"
    return {"id": b.id, "label": b.label, "runnable": why is None, "reason": why,
            "lidar_world": b.lidar_world,
            "extrinsic": str(x.relative_to(REPO)) if x else None,
            "extrinsic_provenance": provenance}


def scan(there_all: dict[str, set[str]] | None = None) -> dict:
    dates: dict[str, list] = {}
    for d in sorted(p for p in DATASET_ROOT.iterdir() if p.is_dir()):
        here = streams(d)
        if not here:
            continue
        there = there_all.get(d.name, set()) if there_all is not None else None
        info = {}
        if (d / "info.json").exists():
            try:
                info = json.loads((d / "info.json").read_text())
            except Exception:
                info = {}
        dur = next((info[k]["duration_s"] for k in ("SLAM", "SAM", "lidar")
                    if isinstance(info.get(k), dict) and info[k].get("duration_s")), None)
        dates.setdefault(date_of(d.name), []).append({
            "name": d.name,
            "short": d.name[7:] if re.match(r"\d{6}_", d.name) else d.name,
            "streams": sorted(here),
            "mac_streams": sorted(there) if there is not None else None,
            "duration_s": round(dur, 1) if dur else None,
            "backends": [backend_state(d.name, here, there, b) for b in BACKENDS],
        })
    return {"mac_reachable": there_all is not None, "features": list(FEATURES),
            "dates": [{"date": k, "datasets": v} for k, v in sorted(dates.items(), reverse=True)]}


def hub_args(dataset: str, backend_id: str, features: int = DEFAULT_FEATURES) -> list[str]:
    """The hub.py argv that runs this dataset on this backend (features: ORB-SLAM3 only)."""
    b = BY_ID[backend_id]
    orb = b.slam == "orbslam3"
    if orb and features not in FEATURES:
        raise ValueError(f"features {features} not in {FEATURES}")
    # f2000 keeps the plain name so earlier runs stay where the comparison expects them
    tag = f"_f{features}" if orb and features != DEFAULT_FEATURES else ""
    d = DATASET_ROOT / dataset
    x, _ = extrinsic_for(dataset, b)
    if x is None:
        raise ValueError(f"{dataset}: no extrinsic for {backend_id}")
    args = ["--slam", b.slam,
            "--extrinsic", str(x),
            "--sam-session", str(d / "SAM"),
            "--slam-session", str(d / b.source),
            "--mac-slam-session", f"{MAC_ROOT}/{dataset}/{b.source}",
            "--out", str(REPO / "objpose" / "output" / f"live_{dataset}_{backend_id}{tag}")]
    if orb:
        args += ["--features", str(features)]
    date = date_of(dataset)
    yaml = SETTINGS / f"orbslam3_{date}_slam_nf2000.yaml"
    if not b.lidar_world and yaml.exists():
        args += ["--slam-arg=--settings", f"--slam-arg=~/objpose/settings/{yaml.name}"]
    if b.gyro:
        args += [f"--slam-arg=--gyro", f"--slam-arg=~/objpose/settings/rig_{date}_gyro.json",
                 "--slam-arg=--imu", f"--slam-arg={MAC_ROOT}/{dataset}/imu"]
    return args


if __name__ == "__main__":
    c = scan(mac_datasets())
    print(f"Mac {'reachable' if c['mac_reachable'] else 'UNREACHABLE — showing PC-only view'}")
    for grp in c["dates"]:
        print(f"\n{grp['date']}")
        for ds in grp["datasets"]:
            print(f"  {ds['short']:<24} {'/'.join(ds['streams']):<22} {ds['duration_s'] or '-'} s")
            for b in ds["backends"]:
                mark = "실행가능" if b["runnable"] else f"불가: {b['reason']}"
                note = {"borrowed": " (RT 는 같은 날 다른 녹화에서 차용)",
                        "unknown": " (RT 출처 미기록)"}.get(b["extrinsic_provenance"], "") \
                    if b["runnable"] else ""
                print(f"      {b['label']:<18} {mark}{note}")
