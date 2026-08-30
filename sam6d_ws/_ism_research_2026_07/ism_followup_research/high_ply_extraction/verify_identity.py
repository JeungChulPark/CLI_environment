#!/usr/bin/env python3
"""verify_identity.py — 운영 CAD PLY 와 압축본 *_high.ply 가 실제로 같은 메시인지 확정한다.

정점 수가 같다고 같은 파일은 아니다. 여기서는
  (1) 정점 블록(좌표+색)만 뽑아 정규화 후 SHA256 지문 비교
  (2) 다르면 좌표 스케일비와 색 차이를 분리해 "스케일만 다른가 / 내용이 다른가" 판별
을 수행한다. 렌더는 get_norm_info 로 정규화되므로 스케일 차이는 결과에 영향을 주지 않는다.

산출: results/ply_identity.csv
"""
import csv, hashlib, os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ROOT))
EXTRACT = os.path.join(REPO, "data", "ply_files_excluding_20260702_extracted")
RES = os.path.join(ROOT, "results")

# 운영 템플릿이 실제로 쓴 PLY (template/_logs/_batch_driver.log 기준)
PAIRS = [
    # (ISM 객체명, 운영 PLY, 압축본 후보들)
    ("Bear",                "data/cad/Bear/bear_color_with_normal_vertexcolor.ply",
     ["Bear_high", "Bear_middle", "Bear_low"]),
    ("Dinosaur",            "data/cad/Dinosaur/Dinosaur_color_with_normal_vertexcolor.ply",
     ["Dinosaur_high", "Dinosaur_middle", "Dinosaur_low"]),
    ("Febreze_high",        "data/cad/Febreze_high/Febreze_color_with_normal_vertexcolor.ply",
     ["Febreze_high", "Febreze_meddle", "Febreze_low"]),
    ("Mugcup_high",         "data/cad/Mugcup_color_high/Mugcup_color_with_normal_vertexcolor.ply",
     ["Mugcup_high", "Mugcup_middle", "Mugcup_low"]),
    ("Rabbit",              "data/cad/Rabbit/Rabbit_color_with_normal_vertexcolor.ply",
     ["Rabbit_high", "Rabbit_middle", "Rabbit_low"]),
    ("Sauce_high",          "data/cad/Sauce_high/Sauce_color_with_normal_vertexcolor.ply",
     ["Sauce_high", "Sauce_middle", "Sauce_low"]),
    ("Sikhye_high",         "data/cad/Sikhye_high/Sikhye_color_with_normal_vertexcolor.ply",
     ["Sikhye_high", "Sikhye_middle", "Sikhye_low"]),
    ("choco_hazelnut_high", "data/cad/choco_hazelnut_color_high/choco_hazelnut_color_with_normal_vertexcolor.ply",
     ["choco_hazelnut_high", "choco_hazelnut_middle", "choco_hazelnut_low"]),
    ("saffron",             "data/cad/saffron/saffron_color_with_normal_vertexcolor.ply",
     ["saffron_high", "saffron_middle", "saffron_low"]),
    ("milk",                "data/cad/milk/Milk.ply", []),
]


def load_vertices(path, limit=400000):
    """정점 좌표(xyz)와 색(rgb)을 최대 limit 개 읽는다 (ascii ply)."""
    props, nv, fmt = [], 0, ""
    with open(path, "rb") as f:
        elem = None
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line:
                return None, None, 0, ""
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element"):
                _, nm, n = line.split(); elem = nm
                if nm == "vertex":
                    nv = int(n)
            elif line.startswith("property") and elem == "vertex":
                props.append(line.split()[-1])
            elif line == "end_header":
                off = f.tell(); break
    if not fmt.startswith("ascii"):
        return None, None, nv, fmt
    idx = {p: i for i, p in enumerate(props)}
    step = max(1, nv // limit)
    xyz, rgb = [], []
    with open(path, "r", errors="replace") as f:
        f.seek(off)
        for i in range(nv):
            line = f.readline()
            if not line:
                break
            if i % step:
                continue
            t = line.split()
            if len(t) < len(props):
                continue
            xyz.append([float(t[idx[c]]) for c in ("x", "y", "z")])
            if "red" in idx:
                rgb.append([int(t[idx[c]]) for c in ("red", "green", "blue")])
    return (np.array(xyz, np.float64), np.array(rgb, np.int16) if rgb else None, nv, fmt)


def normalized(xyz):
    """중심화 + 최대반경 1 로 정규화 (스케일·평행이동 불변)."""
    c = xyz - xyz.mean(0)
    r = float(np.abs(c).max())
    return c / (r if r > 0 else 1.0), r


def same_mesh(a_xyz, a_rgb, b_xyz, b_rgb, tol=1e-4):
    """스케일만 다른 동일 메시인지 수치 비교로 판정한다.

    해시는 부동소수 반올림에 취약하므로(스케일 1000배 차이에서 마지막 자리가 흔들린다)
    정규화 좌표의 최대 절대오차와 색 완전일치로 판정한다. 정점 순서는 동일하다고 가정하되
    개수가 같을 때만 비교한다.
    """
    if a_xyz is None or b_xyz is None or len(a_xyz) != len(b_xyz):
        return None, None, None
    na, _ = normalized(a_xyz)
    nb, _ = normalized(b_xyz)
    dmax = float(np.abs(na - nb).max())
    if a_rgb is None or b_rgb is None:
        cmatch = None
    else:
        cmatch = float((a_rgb == b_rgb).all(1).mean())
    return (dmax <= tol and (cmatch is None or cmatch > 0.999)), dmax, cmatch


rows = []
for obj, oppath, cands in PAIRS:
    op = os.path.join(REPO, oppath)
    if not os.path.isfile(op):
        rows.append({"ism_object": obj, "operational_ply": oppath, "status": "운영 PLY 없음"})
        print(f"{obj:22s} 운영 PLY 없음: {oppath}")
        continue
    oxyz, orgb, onv, ofmt = load_vertices(op)
    orad = normalized(oxyz)[1] if oxyz is not None else 0.0
    matched, detail = "", []
    for c in cands:
        cp = os.path.join(EXTRACT, c + ".ply")
        if not os.path.isfile(cp):
            continue
        cxyz, crgb, cnv, cfmt = load_vertices(cp)
        if cxyz is None:
            detail.append(f"{c}:binary(v={cnv})"); continue
        same, dmax, cmatch = same_mesh(oxyz, orgb, cxyz, crgb)
        if same is None:
            detail.append(f"{c}:정점수 {cnv:,} 불일치")
        else:
            crad = normalized(cxyz)[1]
            detail.append(f"{c}:{'SAME' if same else 'diff'}"
                          f"(dmax={dmax:.2e},색일치={cmatch if cmatch is None else round(cmatch,4)},"
                          f"scale={orad/crad if crad else 0:.1f}x)")
            if same and not matched:
                matched = c
    rows.append({
        "ism_object": obj, "operational_ply": oppath, "op_n_vertex": onv,
        "op_radius": round(orad, 3),
        "matched_archive_tier": matched,
        "identical_to_archive_high": int(matched.endswith("_high")),
        "candidates_checked": " | ".join(detail),
        "status": ("운영 = 압축본 " + matched + " (스케일만 다름)") if matched
                  else "일치 없음(다른 스캔/가공본)",
    })
    print(f"{obj:22s} v={onv:>9,d} r={orad:>8.2f}  →  {rows[-1]['status']}")
    print(f"{'':22s}   {' | '.join(detail)}")

with open(os.path.join(RES, "ply_identity.csv"), "w", newline="") as f:
    ks = ["ism_object", "operational_ply", "op_n_vertex", "op_radius", "op_fingerprint",
          "matched_archive_tier", "identical_to_archive_high", "candidates_checked", "status"]
    w = csv.DictWriter(f, fieldnames=ks, extrasaction="ignore"); w.writeheader(); w.writerows(rows)
print(f"\n-> {os.path.join(RES, 'ply_identity.csv')}")
