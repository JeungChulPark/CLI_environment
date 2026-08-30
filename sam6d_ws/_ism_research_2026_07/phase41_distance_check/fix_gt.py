#!/usr/bin/env python3
"""phase41 (1) — 측정용 객체 위치(GT)를 강건하게 다시 잡는다.

지금까지 GT 로 쓴 `fused/orb3/objects.json` 의 랜드마크는 lifecycle 의 **1/(n+1) 러닝 평균**이라
엉뚱한 물체에 붙은 소수 검출에도 끌려간다. 190624 의 choco 가 그 예로, 투영점이 실제 상자에서
49 px 벗어나 허공을 가리켰다.

여기서는 그 세션의 PEM 검출 전부를 map 좌표로 옮긴 뒤 **25 cm 반경 군집 중 최대 군집의 중앙값**을
쓴다. 소수 이상치가 위치를 끌지 못한다. 파이프라인은 건드리지 않는다 — 이건 **자로 쓰는 값**이다.

출력: 세션마다 `<출력>/landmarks_fixed.json`
"""
import argparse, glob, json, os, sqlite3, sys
from collections import defaultdict

import numpy as np

ROOT = "/home/ldh9501/temp_ws/CLI_environment"
OUTR = f"{ROOT}/integration/output/0807_chungbuk"
DATR = f"{ROOT}/integration/data/0807_chungbuk"
COLOR = "/camera/camera/color/image_raw"


def q2R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def color_stamps(sess):
    db = sorted(glob.glob(f"{DATR}/{sess}/sam/*.db3"))[0]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True); cur = con.cursor()
    cur.execute("select id,name from topics"); tops = cur.fetchall()
    tid = [t for t, n in tops if n == COLOR][0]
    cur.execute("select timestamp from messages where topic_id=? order by timestamp", (tid,))
    s = np.array([r[0] for r in cur.fetchall()])/1e9
    con.close()
    return s


def robust_landmarks(sess, radius=0.25, min_n=4):
    tr = np.loadtxt(f"{OUTR}/{sess}/orb3/trajectory.txt")
    tt = tr[:, 0]
    T = np.zeros((len(tr), 4, 4)); T[:, 3, 3] = 1
    for i, r in enumerate(tr):
        T[i, :3, :3] = q2R(r[4:8]); T[i, :3, 3] = r[1:4]
    X = np.load(sorted(glob.glob(f"{DATR}/{sess}/calib/*.npy"))[0])
    cts = color_stamps(sess)
    det = json.load(open(f"{OUTR}/{sess}/sam/objects.json"))["detections"]
    pts = defaultdict(list)
    for d in det:
        if not d.get("pem_ok") or d.get("R") is None or d.get("t_mm") is None:
            continue
        fi = d.get("frame_index")
        if fi is None or fi >= len(cts):
            continue
        t = cts[fi]
        k = int(np.clip(np.searchsorted(tt, t), 1, len(tt)-1))
        k = min([k-1, k], key=lambda j: abs(tt[j]-t))
        if abs(tt[k]-t) > 0.10:
            continue
        Tco = np.eye(4); Tco[:3, :3] = np.array(d["R"]); Tco[:3, 3] = np.array(d["t_mm"])/1000.0
        pts[d["object"]].append(((T[k] @ X @ Tco)[:3, 3], float(d.get("score", 0.0))))
    out = {}
    for name, lst in pts.items():
        P = np.array([p for p, _ in lst])
        if len(P) < min_n:
            continue
        cl = []
        for i, p in enumerate(P):
            for c in cl:
                if np.linalg.norm(p - c["c"]) < radius:
                    c["i"].append(i); c["c"] = P[c["i"]].mean(axis=0); break
            else:
                cl.append({"c": p.copy(), "i": [i]})
        cl.sort(key=lambda c: -len(c["i"]))
        main = cl[0]["i"]
        out[name] = {"xyz": [round(float(v), 4) for v in np.median(P[main], axis=0)],
                     "n_used": len(main), "n_total": len(P),
                     "n_clusters": len(cl),
                     "cluster_sizes": [len(c["i"]) for c in cl[:5]]}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", nargs="*", default=[])
    a = ap.parse_args()
    sess = a.sessions or sorted(d for d in os.listdir(OUTR)
                                if os.path.isfile(f"{OUTR}/{d}/sam/objects.json")
                                and not d.endswith("_phase39"))
    for s in sess:
        lm = robust_landmarks(s)
        old = {o["object_name"]: np.array(o["T_map_obj"])[:3, 3]
               for o in sorted(json.load(open(f"{OUTR}/{s}/fused/orb3/objects.json"))["objects"],
                               key=lambda o: o["observations"]) if o["observations"] >= 5}
        json.dump(lm, open(f"{OUTR}/{s}/landmarks_fixed.json", "w"), indent=1)
        print(f"\n=== {s}")
        print(f"  {'object':22s}{'검출':>6s}{'주군집':>7s}{'군집수':>7s}{'기존 대비 이동(m)':>18s}")
        for n in sorted(lm):
            d = np.linalg.norm(np.array(lm[n]["xyz"]) - old[n]) if n in old else float("nan")
            flag = "  <-- 크게 이동" if d == d and d > 0.20 else ""
            print(f"  {n:22s}{lm[n]['n_total']:6d}{lm[n]['n_used']:7d}{lm[n]['n_clusters']:7d}"
                  f"{d:18.3f}{flag}")


if __name__ == "__main__":
    main()
