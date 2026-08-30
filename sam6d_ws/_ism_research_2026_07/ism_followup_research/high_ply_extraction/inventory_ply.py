#!/usr/bin/env python3
"""inventory_ply.py — 압축 해제본 *_high.ply 와 운영 CAD PLY 를 대조한다 (READ-ONLY).

헤더만 읽으므로 수백 MB 파일도 즉시 처리된다.
산출: results/high_ply_inventory.csv
"""
import csv, glob, hashlib, os, re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                                   # ism_followup_research
REPO = os.path.dirname(os.path.dirname(ROOT))                  # sam6d_ws
EXTRACT = os.path.join(REPO, "data", "ply_files_excluding_20260702_extracted")
CAD = os.path.join(REPO, "data", "cad")
TPL = os.path.join(REPO, "template")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)


def read_header(path):
    """PLY 헤더를 파싱한다. end_header 까지만 읽는다."""
    info = {"format": "", "n_vertex": 0, "n_face": 0, "props": [], "comments": []}
    with open(path, "rb") as f:
        elem = None
        for _ in range(200):
            raw = f.readline()
            if not raw:
                break
            line = raw.decode("ascii", "replace").strip()
            if line.startswith("format"):
                info["format"] = line.split(None, 1)[1]
            elif line.startswith("comment"):
                info["comments"].append(line[8:])
            elif line.startswith("element"):
                _, name, n = line.split()
                elem = name
                if name == "vertex":
                    info["n_vertex"] = int(n)
                elif name == "face":
                    info["n_face"] = int(n)
            elif line.startswith("property") and elem == "vertex":
                info["props"].append(line.split()[-1])
            elif line == "end_header":
                info["header_bytes"] = f.tell()
                break
    return info


def first_vertices(path, n=20000):
    """헤더 직후 정점 n개의 좌표/색을 읽어 스케일과 색 통계를 낸다 (ascii 전용)."""
    h = read_header(path)
    if not h["format"].startswith("ascii"):
        return None, h
    props = h["props"]
    idx = {p: i for i, p in enumerate(props)}
    xs, cols = [], []
    with open(path, "r", errors="replace") as f:
        while True:
            line = f.readline()
            if not line or line.strip() == "end_header":
                break
        for _ in range(min(n, h["n_vertex"])):
            line = f.readline()
            if not line:
                break
            t = line.split()
            if len(t) < len(props):
                continue
            xs.append([float(t[idx[c]]) for c in ("x", "y", "z")])
            if "red" in idx:
                cols.append([int(t[idx[c]]) for c in ("red", "green", "blue")])
    return (xs, cols), h


def stats(path):
    (xc), h = first_vertices(path)
    row = {"format": h["format"], "n_vertex": h["n_vertex"], "n_face": h["n_face"],
           "has_normal": int("nx" in h["props"]), "has_uv": int("s" in h["props"]),
           "has_vertex_color": int("red" in h["props"]),
           "size_mb": round(os.path.getsize(path) / 1e6, 2)}
    if xc is None:
        row["note"] = "binary ply — 좌표/색 통계 생략"
        return row
    xs, cols = xc
    if xs:
        ext = [max(v[i] for v in xs) - min(v[i] for v in xs) for i in range(3)]
        row["sample_extent_xyz"] = ";".join(f"{e:.2f}" for e in ext)
        row["max_abs_coord_sample"] = round(max(abs(c) for v in xs for c in v), 2)
    if cols:
        n = len(cols)
        row["mean_rgb"] = ";".join(str(round(sum(c[i] for c in cols) / n, 1)) for i in range(3))
        row["n_unique_rgb_sample"] = len({tuple(c) for c in cols})
        row["n_sampled"] = n
    return row


rows = []

# 1) 압축 해제본 전체
for p in sorted(glob.glob(os.path.join(EXTRACT, "*.ply"))):
    name = os.path.basename(p)[:-4]
    m = re.match(r"^(.*?)_(high|middle|meddle|low|ani)$", name)
    obj, tier = (m.group(1), m.group(2)) if m else (name, "")
    r = {"source": "archive_20260702", "file": name, "object_base": obj, "tier": tier,
         "path": os.path.relpath(p, REPO)}
    r.update(stats(p))
    rows.append(r)

# 2) 운영 CAD (data/cad/<dir>/*.ply)
for p in sorted(glob.glob(os.path.join(CAD, "*", "*.ply"))) + \
         sorted(glob.glob(os.path.join(CAD, "fail", "*", "*.ply"))):
    d = os.path.basename(os.path.dirname(p))
    tpl = os.path.join(TPL, d, "templates")
    n_rgb = len(glob.glob(os.path.join(tpl, "rgb_*.png")))
    r = {"source": "operational_cad", "file": os.path.basename(p)[:-4], "object_base": d,
         "tier": "", "path": os.path.relpath(p, REPO),
         "template_dir": os.path.relpath(tpl, REPO) if n_rgb else "",
         "template_view_count": n_rgb}
    r.update(stats(p))
    rows.append(r)

cols = ["source", "object_base", "tier", "file", "path", "size_mb", "format",
        "n_vertex", "n_face", "has_normal", "has_uv", "has_vertex_color",
        "mean_rgb", "n_unique_rgb_sample", "n_sampled",
        "sample_extent_xyz", "max_abs_coord_sample",
        "template_dir", "template_view_count", "note"]
out = os.path.join(RES, "high_ply_inventory.csv")
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
    w.writeheader()
    w.writerows(rows)
print(f"-> {out}  ({len(rows)} rows)")

for r in rows:
    print(f"{r['source'][:9]:9s} {r['object_base']:26s} {r.get('tier',''):7s} "
          f"v={r['n_vertex']:>9,d} f={r['n_face']:>9,d} rgb={r.get('mean_rgb','-'):>18s} "
          f"uniq={r.get('n_unique_rgb_sample','-'):>6} tpl={r.get('template_view_count','')}")
