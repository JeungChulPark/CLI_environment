#!/usr/bin/env python3
"""0807 파이프라인 공용 헬퍼 — 이 파일 자체는 실행 명령이 아니다.

세션 하나를 다루는 6개 명령(convert / sam6d / orb3 / rtabmap / hdl / fuse_video)이
경로 규칙·점군 읽기·2D 맵 생성을 여기서 공유한다.

경로 규칙
    입력   data_slam/0807_chungbuk/{SLAM,SAM}/260807/recording_20260807_<세션>/
    변환   integration/data/0807_chungbuk/<세션>/{slam,sam,lidar}/
    결과   integration/output/0807_chungbuk/<세션>/

<세션>은 접두사를 뗀 이름이다 (예: 185223__circle_ccw_8laps).
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

# 어떤 python 으로 불러도 되게 한다. 시스템 python3 에는 numpy/cv2 가 없으므로,
# 없으면 numpy 가 있는 인터프리터로 자기 자신을 다시 실행한다.
try:
    import numpy as np
except ModuleNotFoundError:                                   # pragma: no cover
    _py = Path.home() / "miniconda3" / "envs" / "sam_yolo" / "bin" / "python"
    if _py.is_file() and Path(sys.executable).resolve() != _py.resolve():
        os.execv(str(_py), [str(_py)] + sys.argv)
    raise

PIPELINE_DIR = Path(__file__).resolve().parent
INTEGRATION_DIR = PIPELINE_DIR.parent
ROOT = INTEGRATION_DIR.parent

# 데이터셋은 환경변수로 바꾼다 (기본 0807). 0724 처럼 이미 표준 bag 이 있는
# 데이터셋은 convert.py 없이 data/<DATASET>/<세션>/ 을 직접 채워 쓰면 된다.
#     INTEGRATION_DATASET=0724_chungbuk python integration/pipeline/orb3.py circle3d_183550
DATASET = os.environ.get("INTEGRATION_DATASET", "0807_chungbuk")
RAW_ROOT = ROOT / "data_slam" / DATASET
DATA_ROOT = INTEGRATION_DIR / "data" / DATASET
OUT_ROOT = INTEGRATION_DIR / "output" / DATASET
PREFIX = os.environ.get("INTEGRATION_PREFIX", "recording_20260807_")

# RTAB/hdl 실행기와 조밀화 스크립트는 0807 폴더에 살지만 내용은 데이터셋 독립이다
# (--pair/--manifest 로만 대상을 받는다). DATASET 을 바꿔도 여기는 따라가면 안 된다.
TOOLS_ROOT = ROOT / "data_slam" / "0807_chungbuk"

CONDA = Path.home() / "miniconda3" / "bin" / "conda"
CONDA_SH = Path.home() / "miniconda3" / "etc" / "profile.d" / "conda.sh"
PY_NUMPY = Path.home() / "miniconda3" / "envs" / "sam_yolo" / "bin" / "python"

# 광학 프레임(x=오른쪽, y=아래, z=앞) -> ROS z-up (REP-103). integration/to_zup.py 와 동일.
R_OPT2ROS = np.array([[0.0, 0.0, 1.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0]])


# --------------------------------------------------------------------------
# 로그 / 실행
# --------------------------------------------------------------------------
def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def die(msg: str) -> "None":
    print(f"오류: {msg}", file=sys.stderr)
    raise SystemExit(1)


def run(cmd, cwd: Path = ROOT, timeout: float = 7200, log_path: Path | None = None,
        label: str = "") -> int:
    """셸 명령 하나. 로그 경로를 주면 stdout+stderr 를 그리로 흘린다."""
    if label:
        log(f"{label} 시작")
    t0 = time.time()
    if isinstance(cmd, str):
        cmd = ["bash", "-lc", cmd]
    if log_path:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("wb") as f:
            p = subprocess.run(cmd, cwd=str(cwd), stdout=f, stderr=subprocess.STDOUT,
                               timeout=timeout)
    else:
        p = subprocess.run(cmd, cwd=str(cwd), timeout=timeout)
    if label:
        log(f"{label} 끝 ({time.time() - t0:.0f}s, rc={p.returncode})")
    return p.returncode


def conda_run(env: str, cmd: str, **kw) -> int:
    return run(f"source {CONDA_SH} && conda activate {env} && {cmd}", **kw)


# --------------------------------------------------------------------------
# 세션 이름
# --------------------------------------------------------------------------
def resolve_session(arg: str) -> str:
    """'185223__circle_ccw_8laps', 전체 폴더명, 원본 경로 어느 것으로도 부를 수 있다."""
    name = Path(str(arg).rstrip("/")).name
    if name.startswith(PREFIX):
        name = name[len(PREFIX):]
    if (RAW_ROOT / "SLAM" / "260807" / (PREFIX + name)).is_dir():
        return name
    # 원본이 0807 배치가 아닌 데이터셋(예: 0724)은 이미 만들어 둔 입력 폴더로 판단한다.
    if (DATA_ROOT / name / "info.json").is_file():
        return name
    avail = sorted(d.name for d in DATA_ROOT.iterdir() if d.is_dir()) if DATA_ROOT.is_dir() else []
    if (RAW_ROOT / "SLAM" / "260807").is_dir():
        avail += sorted(d.name[len(PREFIX):]
                        for d in (RAW_ROOT / "SLAM" / "260807").iterdir() if d.is_dir())
    die(f"세션을 찾을 수 없다: {arg}\n쓸 수 있는 이름:\n  " + "\n  ".join(sorted(set(avail))))


def raw_dirs(sess: str) -> dict:
    return {"slam": RAW_ROOT / "SLAM" / "260807" / (PREFIX + sess),
            "sam": RAW_ROOT / "SAM" / "260807" / (PREFIX + sess)}


def data_dir(sess: str) -> Path:
    return DATA_ROOT / sess


def out_dir(sess: str) -> Path:
    return OUT_ROOT / sess


def write_manifest(sess: str, stage: Path) -> Path:
    """run_slam_0807.py(RTAB/hdl) 가 읽는 세션 기술 파일. 세션마다 따로 쓴다.

    공용 manifest_0807.json 을 덮어쓰면 동시에 다른 세션을 돌릴 수 없기 때문이다.
    """
    info = load_info(sess)
    s = info["slam"]
    stage.mkdir(parents=True, exist_ok=True)
    m = [{"name": sess,
          "bag": str(data_dir(sess) / "slam"),
          "lidar_bag": info.get("lidar_bag", ""),
          "duration_sec": s.get("duration_s", 0.0),
          "rgb": s.get("color_topic", ""),
          "depth": s.get("depth_topic", ""),
          "camera_info": s.get("caminfo_topic", ""),
          "frame_id": "camera_color_optical_frame",
          "lidar": info.get("lidar_topic", "/velodyne_points")}]
    p = stage / "manifest.json"
    p.write_text(json.dumps(m, indent=1, ensure_ascii=False))
    return p


def load_info(sess: str) -> dict:
    p = data_dir(sess) / "info.json"
    if not p.is_file():
        die(f"{p} 가 없다 — 먼저 convert.py 를 돌려라")
    return json.loads(p.read_text())


# --------------------------------------------------------------------------
# 궤적 (TUM: t x y z qx qy qz qw)
# --------------------------------------------------------------------------
def read_tum(path: Path) -> tuple:
    rows = []
    for ln in Path(path).read_text().splitlines():
        ln = ln.strip()
        if not ln or ln.startswith("#"):
            continue
        v = ln.split()
        if len(v) >= 8:
            rows.append([float(x) for x in v[:8]])
    if not rows:
        die(f"궤적이 비어 있다: {path}")
    a = np.asarray(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def quat_to_R(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


# --------------------------------------------------------------------------
# 점군 읽기 — 세 백엔드가 서로 다른 포맷을 낸다
# --------------------------------------------------------------------------
def read_pcd(path: Path) -> np.ndarray:
    """ascii / binary PCD 의 xyz 만 (N,3) 으로."""
    raw = Path(path).read_bytes()
    end = raw.find(b"\n", raw.find(b"DATA"))
    header = raw[:end].decode("ascii", "replace").splitlines()
    body = raw[end + 1:]
    fields, size, count, npts, fmt = [], [], [], 0, "ascii"
    for ln in header:
        k, _, v = ln.partition(" ")
        if k == "FIELDS":
            fields = v.split()
        elif k == "SIZE":
            size = [int(x) for x in v.split()]
        elif k == "COUNT":
            count = [int(x) for x in v.split()]
        elif k == "POINTS":
            npts = int(v)
        elif k == "DATA":
            fmt = v.strip()
    if fmt == "ascii":
        a = np.fromstring(body.decode("ascii", "replace"), sep=" ")  # noqa: NPY003
        ncol = len(fields)
        a = a[:(len(a) // ncol) * ncol].reshape(-1, ncol)
        return a[:, :3]
    if fmt != "binary":
        die(f"지원하지 않는 PCD DATA 형식: {fmt} ({path})")
    if not count:
        count = [1] * len(fields)
    stride = sum(s * c for s, c in zip(size, count))
    arr = np.frombuffer(body[:npts * stride], dtype=np.uint8).reshape(npts, stride)
    off, out = 0, []
    for f, s, c in zip(fields, size, count):
        if f in ("x", "y", "z"):
            out.append(arr[:, off:off + 4].copy().view(np.float32).ravel())
        off += s * c
    return np.stack(out, axis=1).astype(np.float64)


def read_ply(path: Path) -> np.ndarray:
    """ascii PLY 의 vertex xyz 만 (N,3) 으로 (RTAB-Map 내보내기 형식)."""
    raw = Path(path).read_bytes()
    hend = raw.find(b"end_header\n") + len(b"end_header\n")
    header = raw[:hend].decode("ascii", "replace").splitlines()
    if not any(ln.startswith("format ascii") for ln in header):
        die(f"ascii PLY 만 지원한다: {path}")
    nvert, nprop, seen_vertex = 0, 0, False
    for ln in header:
        if ln.startswith("element vertex"):
            nvert, seen_vertex = int(ln.split()[2]), True
        elif ln.startswith("element ") and seen_vertex:
            seen_vertex = False
        elif ln.startswith("property") and seen_vertex:
            nprop += 1
    body = raw[hend:].decode("ascii", "replace")
    a = np.fromstring(body, sep=" ", count=nvert * nprop)  # noqa: NPY003
    return a.reshape(-1, nprop)[:, :3]


def read_hdl_graph(dump: Path) -> tuple:
    """hdl 의 graph_dump/<kf>/{cloud.pcd,data} 를 최적화 포즈로 합쳐 지도로 만든다."""
    pts, poses = [], []
    for d in sorted(p for p in Path(dump).iterdir() if p.is_dir()):
        cloud, meta = d / "cloud.pcd", d / "data"
        if not (cloud.is_file() and meta.is_file()):
            continue
        lines = meta.read_text().splitlines()
        try:
            i = lines.index("estimate")
        except ValueError:
            continue
        T = np.array([[float(x) for x in lines[i + 1 + r].split()] for r in range(4)])
        p = read_pcd(cloud)
        pts.append(p @ T[:3, :3].T + T[:3, 3])
        poses.append(T)
    if not pts:
        die(f"graph_dump 에서 키프레임을 읽지 못했다: {dump}")
    return np.vstack(pts), poses


# --------------------------------------------------------------------------
# 2D 점유격자
# --------------------------------------------------------------------------
def detect_floor(z: np.ndarray, res: float = 0.10) -> tuple | None:
    """점군 z 히스토그램의 이봉에서 (바닥, 천장) 높이를 찾는다. 못 찾으면 None.

    실내를 RGB-D 로 훑으면 바닥과 천장이 가장 큰 두 평면이라 뚜렷한 이봉이 된다
    (0807 실측: ORB3 바닥 -1.23/천장 +1.37, RTAB -1.20/+1.40 — 천장고 2.60 m 일치).
    반면 VLP-16 은 ±15° 라 바닥을 반경 3.7 m 밖에서 소수 빔으로만 맞고, 긴 복도에서
    맵 프레임이 1~2° 기울면 z 가 2 m 퍼져 단봉이 된다. 그래서 타당성 검사를 붙인다.
    """
    lo, hi = np.percentile(z, [1, 99])
    if hi - lo < 1.0:
        return None
    edges = np.arange(lo, hi + res, res)
    h, _ = np.histogram(z, bins=edges)
    ctr = 0.5 * (edges[:-1] + edges[1:])
    mid = np.median(z)
    below, above = ctr < mid, ctr >= mid
    if not below.any() or not above.any():
        return None
    fl = float(ctr[below][np.argmax(h[below])])
    ce = float(ctr[above][np.argmax(h[above])])
    med = max(np.median(h), 1.0)
    if not (2.0 <= ce - fl <= 5.0):                  # 실내 천장고
        return None
    if h[below].max() < 1.5 * med or h[above].max() < 1.5 * med:
        return None                                  # peak 가 배경보다 뚜렷하지 않다
    return fl, ce


def occupancy_grid(pts: np.ndarray, traj_xyz: np.ndarray, res: float = 0.05,
                   z_lo: float = 0.30, z_hi: float = 2.0, hits: int = 2,
                   max_ray: float = 12.0, max_pts: int = 200_000) -> dict:
    """z-up 점군 + 궤적 -> ROS 점유격자.

    z_lo/z_hi 는 **바닥 위** 높이다. 바닥은 점군에서 검출한다 — 궤적 z 를 바닥으로 쓰면
    그건 바닥이 아니라 센서 높이라, 카메라가 바닥 위 1.2 m 에 있는 이 데이터셋에서는
    슬라이스가 통째로 천장 대역(바닥 위 1.55~3.3 m)으로 올라가 기둥이 사라지고 천장
    구조물이 주행 경로 위 장애물로 찍혔다. 바닥을 못 찾으면(LiDAR) 센서 기준으로 되돌린다.

    자유 공간은 '각 점을 가장 가까운 궤적 포즈에서 봤다'고 가정하고 광선을 쏴서 채운다.
    관측 포즈를 모르기 때문에 근사이며, 그래서 이 격자는 **보기·비교용이지 항법용이
    아니다.** 세 백엔드에 완전히 같은 코드를 적용해 서로 비교할 수 있게 하는 것이 목적.
    """
    if traj_xyz.size == 0:
        die("궤적이 비어 격자를 만들 수 없다")
    fc = detect_floor(pts[:, 2])
    if fc is None:
        floor, z_top, basis = float(np.median(traj_xyz[:, 2])), None, "sensor"
    else:
        floor, ceil = fc
        z_top, basis = ceil - 0.20, "floor"
    z0 = floor + z_lo
    z1 = floor + z_hi if z_top is None else min(floor + z_hi, z_top)
    m = (pts[:, 2] > z0) & (pts[:, 2] < z1)
    P = pts[m][:, :2]
    if P.shape[0] == 0:
        die(f"슬라이스 [{z0:.2f}, {z1:.2f}] m 안에 점이 없다")

    T = traj_xyz[:, :2]
    lo = np.minimum(P.min(0), T.min(0)) - 1.0
    hi = np.maximum(P.max(0), T.max(0)) + 1.0
    W = int(np.ceil((hi[0] - lo[0]) / res))
    H = int(np.ceil((hi[1] - lo[1]) / res))
    occ = np.zeros((H, W), np.int32)
    free = np.zeros((H, W), np.int32)

    def cells(xy):
        c = np.floor((xy - lo) / res).astype(np.int64)
        np.clip(c[:, 0], 0, W - 1, out=c[:, 0])
        np.clip(c[:, 1], 0, H - 1, out=c[:, 1])
        return c

    pc = cells(P)
    np.add.at(occ, (pc[:, 1], pc[:, 0]), 1)              # 적중은 점 전부로 센다

    # 각 점에서 가장 가까운 포즈를 관측점으로 보고 광선을 쏜다 (덩어리로 나눠 벡터화).
    # 광선만 부표본해도 통과 횟수의 상대 분포는 유지된다.
    S = P
    if S.shape[0] > max_pts:
        S = S[np.random.default_rng(0).choice(S.shape[0], max_pts, replace=False)]
    step = res * 0.5
    nmax = int(max_ray / step)
    for s in range(0, S.shape[0], 4000):
        Q = S[s:s + 4000]
        d = np.linalg.norm(T[None, :, :] - Q[:, None, :], axis=2)
        O = T[np.argmin(d, axis=1)]
        v = Q - O
        L = np.linalg.norm(v, axis=1)
        keep = (L > res) & (L < max_ray)
        if not keep.any():
            continue
        O, v, L = O[keep], v[keep], L[keep]
        u = v / L[:, None]
        n = np.minimum((L / step).astype(int), nmax)
        k = np.arange(nmax)[None, :]
        valid = k < (n[:, None] - 1)                     # 끝점(=장애물)은 free 로 찍지 않는다
        Sp = O[:, None, :] + u[:, None, :] * (k[:, :, None] * step)   # S 를 덮으면 안 된다
        sc = cells(Sp[valid])
        np.add.at(free, (sc[:, 1], sc[:, 0]), 1)

    tc = cells(T)
    np.add.at(free, (tc[:, 1], tc[:, 0]), 1)             # 지나간 자리는 확실히 자유공간

    # 고정 임계로 '점이 있으면 점유'로 하면 안 된다. ORB3 dense 는 셀당 중앙값 14점이라
    # 비어있지 않은 셀이 전부 점유로 칠해져 지도가 까맣게 뭉갠다(실측 49067/60797).
    # 표준 점유격자대로 적중 대 통과로 판정한다: 광선이 자주 지나간 셀은 자유다.
    occupied = (occ >= hits) & (occ > free)
    seen = (occ + free) > 0
    grid = np.full((H, W), 205, np.uint8)                # 205 = 미지
    grid[seen] = 254                                     # 254 = 자유
    grid[occupied] = 0                                   # 0   = 점유
    return {"grid": grid, "origin": (float(lo[0]), float(lo[1])), "res": res,
            "floor": floor, "z_slice": [z0, z1], "z_basis": basis,
            "occupied": int((grid == 0).sum()), "free": int((grid == 254).sum())}


def save_grid(g: dict, stem: Path, name: str) -> None:
    """ROS map_server 규약대로 <stem>.pgm + <stem>.yaml."""
    grid = np.flipud(g["grid"])                          # PGM 은 위에서 아래로 쓴다
    H, W = grid.shape
    with (stem.with_suffix(".pgm")).open("wb") as f:
        f.write(f"P5\n# {name}\n{W} {H}\n255\n".encode())
        f.write(grid.tobytes())
    stem.with_suffix(".yaml").write_text(
        f"image: {stem.name}.pgm\n"
        f"resolution: {g['res']}\n"
        f"origin: [{g['origin'][0]:.4f}, {g['origin'][1]:.4f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        f"# z 슬라이스 {g['z_slice'][0]:.2f} ~ {g['z_slice'][1]:.2f} m "
        f"({'점군에서 검출한 바닥' if g['z_basis'] == 'floor' else '센서 높이'} {g['floor']:.2f} 기준)\n",
        encoding="utf-8")


def map_rotation(backend: str) -> np.ndarray:
    """백엔드의 map 프레임 -> z-up.

    ORB3 / RTAB-Map 의 map 은 첫 카메라의 **광학 프레임**이라 y 가 아래를 가리킨다.
    hdl 은 Velodyne 프레임이라 이미 z 가 위다.
    """
    return R_OPT2ROS if backend in ("orb3", "rtabmap") else np.eye(3)


def make_2d_map(backend: str, pts: np.ndarray, traj_xyz: np.ndarray, stem: Path,
                title: str, objects: list | None = None, **kw) -> dict:
    """점군 + 궤적(백엔드 고유 프레임) -> map_2d.{pgm,yaml,png}. 세 백엔드 공통 코드."""
    R = map_rotation(backend)
    g = occupancy_grid(pts @ R.T, traj_xyz @ R.T, **kw)
    objs = [{"name": o["name"], "position": (R @ np.asarray(o["position"]))[:2]}
            for o in (objects or [])]
    stem.parent.mkdir(parents=True, exist_ok=True)
    save_grid(g, stem, title)
    render_map_png(g, traj_xyz @ R.T, stem.with_suffix(".png"), title, objs)
    log(f"2D 맵: {g['grid'].shape[1]}x{g['grid'].shape[0]} @ {g['res']} m "
        f"· 기준 {g['z_basis']} {g['floor']:+.2f} · z {g['z_slice'][0]:+.2f}~{g['z_slice'][1]:+.2f} "
        f"· 점유 {g['occupied']} · 자유 {g['free']} -> {stem}.{{pgm,yaml,png}}")
    return g


def render_map_png(g: dict, traj_xyz: np.ndarray, path: Path, title: str,
                   objects: list | None = None, scale: int = 3) -> None:
    """격자 + 궤적 (+ 객체) 를 사람이 보는 PNG 로. 동영상 지도 패널과 같은 렌더러."""
    import cv2

    grid = g["grid"]
    img = np.full(grid.shape + (3,), 245, np.uint8)
    img[grid == 254] = (255, 255, 255)
    img[grid == 205] = (232, 232, 232)
    img[grid == 0] = (60, 60, 60)
    img = np.flipud(img)
    H = img.shape[0]
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

    def px(xy):
        c = (np.asarray(xy)[:2] - np.asarray(g["origin"])) / g["res"]
        return int(c[0] * scale), int((H - c[1]) * scale)

    pts = [px(p) for p in traj_xyz[:, :2]]
    for a, b in zip(pts[:-1], pts[1:]):
        cv2.line(img, a, b, (232, 118, 43), max(1, scale // 2), cv2.LINE_AA)
    if pts:
        cv2.circle(img, pts[0], 4 * scale // 2, (40, 160, 40), -1)
        cv2.circle(img, pts[-1], 4 * scale // 2, (60, 60, 220), -1)

    for o in (objects or []):
        c = px(o["position"])
        cv2.rectangle(img, (c[0] - 3 * scale, c[1] - 3 * scale),
                      (c[0] + 3 * scale, c[1] + 3 * scale), (40, 160, 40), -1)
        cv2.putText(img, o["name"], (c[0] + 5 * scale, c[1] - 3 * scale),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35 * scale, (20, 90, 20), 1, cv2.LINE_AA)

    bar = np.full((30, img.shape[1], 3), 255, np.uint8)
    cv2.putText(bar, f"{title}   {g['res']} m/cell   z {g['z_slice'][0]:.2f}~{g['z_slice'][1]:.2f} m",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), np.vstack([bar, img]))
