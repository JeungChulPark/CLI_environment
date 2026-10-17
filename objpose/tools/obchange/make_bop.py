#!/usr/bin/env python3
"""make_bop.py — ObChange 한 방의 bag 들을 이 저장소가 쓰는 BOP 장면 형식으로 바꾼다.

  <out>/test/<room>_s<N>/rgb/000001.png …  depth/000001.png (0.1 mm 단위, YCB-V 와 같은 depth_scale 0.1)
                         scene_camera.json (cam_K, depth_scale, cam_R_w2c, cam_t_w2c[mm]; 세계 = 데이터셋 tf 의 map)
                         scene_gt.json     (YCB-V 21종에 드는 정답 물체만. 자세는 .anno 의 4x4 를 그대로 옮긴 값 —
                                            모델 좌표계가 BOP 모델과 달라 방향·중심은 채점에 쓰지 않는다)
  <out>/poses_tf/<scene>_frames.txt        t(s, i/fps) x y z qx qy qz qw  (ORB-SLAM3 출력과 같은 형식, 세계 = map)
  <out>/gt/<scene>.json                    정답 물체 전체(21종 밖 포함): 이름, .anno 4x4

카메라 자세는 bag 의 tf (map → odom → base_footprint → … → head_rgbd_sensor_rgb_frame) 를 영상 시각에 가장 가까운 값으로 읽는다.
    python3 objpose/tools/obchange/make_bop.py --room Kitchen --raw <ObChange>/raw/GH30_kitchen --step 3
"""
import argparse, bisect, collections, json, re
from pathlib import Path
import cv2, numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

REPO = Path(__file__).resolve().parents[3]
OB = Path("/home/jucpark/DeepLearning/Datasets/ObChange")
CAM = "head_rgbd_sensor_rgb_frame"


def T_of(tr):
    q, t = tr.rotation, tr.translation; x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    M[:3, 3] = [t.x, t.y, t.z]; return M


def rot_to_quat(R):
    w = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2]); y = np.copysign(y, R[0, 2] - R[2, 0]); z = np.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


def read_bag(bag):
    ts = get_typestore(Stores.ROS1_NOETIC)
    tf = collections.defaultdict(list); rgb = []; dep = []; K = None
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    with Reader(bag) as r:
        for c in r.connections:
            if c.topic in ("/tf", "/tf_static"):
                try: ts.register(get_types_from_msg(c.msgdef.data if hasattr(c.msgdef, "data") else c.msgdef, c.msgtype))
                except Exception: pass
        for c, t, raw in r.messages():
            if c.topic in ("/tf", "/tf_static"):
                for tr in ts.deserialize_ros1(raw, c.msgtype).transforms:
                    tf[(tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))].append((st(tr.header), T_of(tr.transform)))
            elif c.topic.endswith("camera_info") and K is None:
                K = np.array(ts.deserialize_ros1(raw, c.msgtype).K).reshape(3, 3)
            elif c.topic.endswith("rgb/image_raw"):
                m = ts.deserialize_ros1(raw, c.msgtype)
                rgb.append((st(m.header), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1].copy()))
            elif "depth_registered" in c.topic:
                m = ts.deserialize_ros1(raw, c.msgtype)
                dep.append((st(m.header), np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()))
    for k in tf: tf[k].sort(key=lambda a: a[0])
    return tf, rgb, dep, K


def make_lookup(tf):
    keys = {k: [a[0] for a in v] for k, v in tf.items()}
    par = {c: p for (p, c) in tf}
    def lookup(t, frame=CAM, root="map"):
        M = np.eye(4); f = frame
        while f != root:
            p = par[f]; s = tf[(p, f)]; ks = keys[(p, f)]
            i = min(max(bisect.bisect_left(ks, t), 0), len(s) - 1)
            if i > 0 and abs(ks[i-1] - t) < abs(ks[i] - t): i -= 1
            M = s[i][1] @ M; f = p
        return M
    return lookup


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--room", required=True); ap.add_argument("--raw", required=True, type=Path)
    ap.add_argument("--out", type=Path, default=OB / "bop"); ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--scenes", nargs="*", type=int, default=None)
    a = ap.parse_args()
    table = json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]
    oid = {o["ycb"]: o["obj_id"] for o in table}
    fps = 30.0 / a.step
    (a.out / "poses_tf").mkdir(parents=True, exist_ok=True); (a.out / "gt").mkdir(exist_ok=True)
    bags = sorted(a.raw.glob("scene*.bag"), key=lambda p: int(re.findall(r"\d+", p.stem)[0]))
    for bag in bags:
        n_s = int(re.findall(r"\d+", bag.stem)[0])
        if a.scenes and n_s not in a.scenes: continue
        name = f"{a.room.lower()}_s{n_s}"
        sc = a.out / "test" / name
        (sc / "rgb").mkdir(parents=True, exist_ok=True); (sc / "depth").mkdir(exist_ok=True)
        tf, rgb, dep, K = read_bag(bag); lookup = make_lookup(tf)
        objs = []
        for an in sorted((OB / "anno" / a.room / f"scene{n_s}" / "planes").glob("*/merged_plane_clouds_ds002.anno")):
            for line in an.read_text().splitlines():
                m = re.match(r"(\S+)\s*\((\d+)\):\s*(.*)", line)
                if m: objs.append({"ycb": m.group(1), "plane": an.parent.name, "T_map_anno": np.array(m.group(3).split(), float).reshape(4, 4).tolist()})
        dts = [d[0] for d in dep]; cam = {}; gt = {}; rows = []; n = 0
        for i in range(0, len(rgb), a.step):
            t, im = rgb[i]; j = min(max(bisect.bisect_left(dts, t), 0), len(dep) - 1)
            if j > 0 and abs(dts[j-1] - t) < abs(dts[j] - t): j -= 1
            if abs(dts[j] - t) > 0.05: continue
            n += 1; fid = f"{n:06d}"
            cv2.imwrite(str(sc / "rgb" / f"{fid}.png"), im)
            cv2.imwrite(str(sc / "depth" / f"{fid}.png"), np.clip(dep[j][1].astype(np.uint32) * 10, 0, 65535).astype(np.uint16))
            Twc = lookup(t); Tcw = np.linalg.inv(Twc)
            cam[str(n)] = {"cam_K": K.reshape(-1).tolist(), "depth_scale": 0.1, "cam_R_w2c": Tcw[:3, :3].reshape(-1).tolist(),
                           "cam_t_w2c": (Tcw[:3, 3] * 1000).tolist(), "stamp": t}
            gt[str(n)] = [{"obj_id": oid[o["ycb"]], "cam_R_m2c": (Tcw @ np.array(o["T_map_anno"]))[:3, :3].reshape(-1).tolist(),
                           "cam_t_m2c": ((Tcw @ np.array(o["T_map_anno"]))[:3, 3] * 1000).tolist()} for o in objs if o["ycb"] in oid]
            q = rot_to_quat(Twc[:3, :3])
            rows.append(f"{(n - 1) / fps:.6f} {Twc[0,3]:.6f} {Twc[1,3]:.6f} {Twc[2,3]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
        (sc / "scene_camera.json").write_text(json.dumps(cam)); (sc / "scene_gt.json").write_text(json.dumps(gt))
        (a.out / "poses_tf" / f"{name}_frames.txt").write_text("\n".join(rows) + "\n")
        (a.out / "gt" / f"{name}.json").write_text(json.dumps({"scene": name, "fps": fps, "objects": objs}, indent=1))
        # tf 요약: map→odom 이 시간에 따라 바뀌는가(위치 추정이 살아 있는가)
        mo = np.array([m[1][:3, 3] for m in tf[("map", "odom")]]); ob = np.array([m[1][:3, 3] for m in tf[("odom", "base_footprint")]])
        print(f"{name}: frames {n}, objects {len(objs)} (21종 안 {sum(o['ycb'] in oid for o in objs)}), "
              f"map→odom 변화 폭 {np.ptp(mo, 0).round(3).tolist()} m, odom→base 이동 폭 {np.ptp(ob, 0).round(2).tolist()} m", flush=True)


if __name__ == "__main__":
    main()
