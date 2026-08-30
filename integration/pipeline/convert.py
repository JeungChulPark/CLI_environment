#!/usr/bin/env python3
"""1단계 — 원본 세션 하나를 SLAM 3종이 먹을 수 있는 표준 bag 으로 바꾼다.

    python integration/pipeline/convert.py data_slam/0807_chungbuk/185223__circle_ccw_8laps

바꾸는 것은 세 가지다. "RGB-D 하나"가 아니다:

    slam/   SLAM(husky) 카메라 RGB-D  — SDK 포맷 + depth 미정렬이라 변환 필요. 기준 시계.
    sam/    SAM(노트북) 카메라 RGB-D  — 위와 같고, 추가로 두 호스트의 시계차만큼 이동.
    lidar/  Velodyne VLP-16          — rosbag2 v9 라 그대로는 재생이 안 돼 v5 로 낮춘다
                                       (metadata.yaml 만 다시 쓰고 db3 는 심볼릭이라 0 바이트).

시각은 카메라 자체 Global Time(--time-source color_global)을 쓴다. association 파일의
clock_fit 보다 정확하다 (교차 산포 9.5 ms -> 1.1 ms).

출력: integration/data/0807_chungbuk/<세션>/{slam,sam,lidar,info.json}
이미 있는 것은 건너뛴다(멱등).
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common as C  # noqa: E402

CONVERT_RECORDING = C.ROOT / "data_slam" / "260714_frame_data" / "convert_recording.py"
CALIB_OFFSET = C.RAW_ROOT / "calib_offset.py"
DOWNGRADE = C.ROOT / "data_slam" / "0724_chungbuk" / "downgrade_bag_metadata.py"
ENV_CONVERT = "sam6d_ros_humble"          # rosbag2_py 가 있는 환경


def clock_offset_ns(sess: str) -> int:
    """SAM 카메라를 기준 시계로 옮기는 상수. genlock 된 프레임으로 실측한다."""
    r = subprocess.run([str(C.PY_NUMPY), str(CALIB_OFFSET), C.PREFIX + sess,
                        "--stream", "color"], capture_output=True, text=True)
    if r.returncode != 0:
        C.die(f"calib_offset.py 실패:\n{r.stdout}\n{r.stderr}")
    return int(r.stdout.strip().splitlines()[-1])


def convert_camera(src: Path, dst: Path, offset_ns: int, label: str, stride: int) -> None:
    if (dst / "metadata.yaml").is_file():
        C.log(f"{label}: 이미 있음 — 건너뜀 ({dst})")
        return
    if dst.exists():
        shutil.rmtree(dst)
    extra = f" --stride {stride}" if stride > 1 else ""
    rc = C.conda_run(
        ENV_CONVERT,
        f"python {CONVERT_RECORDING} {src} {dst} --time-source color_global "
        f"--offset-ns {offset_ns}{extra}",
        label=f"{label} 변환")
    if rc != 0 or not (dst / "metadata.yaml").is_file():
        C.die(f"{label} 변환 실패 ({src})")


def convert_lidar(src_session: Path, dst: Path) -> Path | None:
    velo = next(src_session.glob("cpr-a200-0881_velodyne_husky_*"), None)
    if velo is None:
        C.log("Velodyne bag 이 없다 — hdl 은 이 세션에서 못 돌린다")
        return None
    if (dst / "metadata.yaml").is_file():
        C.log(f"lidar: 이미 있음 — 건너뜀 ({dst})")
        return dst
    if dst.exists():
        shutil.rmtree(dst)
    tmp = dst.parent / "_lidar_tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    rc = C.run([str(C.PY_NUMPY), str(DOWNGRADE), str(velo), "--out-root", str(tmp)],
               label="Velodyne v9->v5")
    made = tmp / velo.name
    if rc != 0 or not (made / "metadata.yaml").is_file():
        shutil.rmtree(tmp, ignore_errors=True)
        C.die(f"Velodyne 다운그레이드 실패 ({velo})")
    made.rename(dst)
    shutil.rmtree(tmp, ignore_errors=True)
    return dst


def probe(bags: dict) -> dict:
    """토픽·내부파라미터·길이를 읽는다 (run_integration.py 의 프로브를 그대로 씀)."""
    sys.path.insert(0, str(C.INTEGRATION_DIR))
    import run_integration as RI

    with tempfile.TemporaryDirectory() as td:
        py = Path(td) / "probe.py"
        py.write_text(RI.PROBE_SRC, encoding="utf-8")
        out = Path(td) / "info.json"
        spec = json.dumps({k: str(v) for k, v in bags.items()})
        r = subprocess.run([str(C.PY_NUMPY), str(py), spec, str(out)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            C.die(f"bag 프로브 실패:\n{r.stdout}\n{r.stderr}")
        return json.loads(out.read_text())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="data_slam/0807_chungbuk/<세션> 또는 세션 이름")
    ap.add_argument("--stride", type=int, default=1, help="N 프레임마다 하나만 (기본 1=전부)")
    a = ap.parse_args()

    sess = C.resolve_session(a.session)
    raw = C.raw_dirs(sess)
    dst = C.data_dir(sess)
    dst.mkdir(parents=True, exist_ok=True)
    C.log(f"세션 {sess}  ->  {dst}")

    off = clock_offset_ns(sess)
    C.log(f"두 호스트 시계차 (SAM - SLAM) = {off / 1e6:.3f} ms — SAM 쪽에서 뺀다")

    convert_camera(raw["slam"], dst / "slam", 0, "SLAM 카메라", a.stride)
    convert_camera(raw["sam"], dst / "sam", -off, "SAM 카메라", a.stride)
    lidar = convert_lidar(raw["slam"], dst / "lidar")

    info = probe({"slam": dst / "slam", "sam": dst / "sam"})
    info["session"] = sess
    info["clock_offset_ns"] = off
    info["lidar_bag"] = str(lidar) if lidar else ""
    info["lidar_topic"] = "/velodyne_points"
    (dst / "info.json").write_text(json.dumps(info, indent=1, ensure_ascii=False))

    for role in ("slam", "sam"):
        i = info[role]
        C.log(f"{role}: {i.get('color_count', '?')} 프레임 · {i.get('duration_s', '?')} s "
              f"· {i.get('color_hz', '?')} Hz · {i.get('color_topic', '?')}")
    C.log(f"완료 — {dst}/info.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
