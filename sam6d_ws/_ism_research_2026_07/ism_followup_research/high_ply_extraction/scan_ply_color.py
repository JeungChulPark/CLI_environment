#!/usr/bin/env python3
"""scan_ply_color.py — PLY vertex color 를 파일 전체에 걸쳐 표본조사한다 (READ-ONLY).

inventory_ply.py 는 선두 20k 정점만 봤으므로 "앞부분이 우연히 단색"인 경우를 구분할 수 없다.
여기서는 파일 전체를 균등 간격으로 훑어 vertex color 가 실제로 존재하는지 확정한다.
binary_little_endian 도 처리한다.

산출: results/ply_color_scan.csv
"""
import csv, glob, os, struct

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ROOT))
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)

PLY_TYPE = {"char": ("b", 1), "uchar": ("B", 1), "int8": ("b", 1), "uint8": ("B", 1),
            "short": ("h", 2), "ushort": ("H", 2), "int16": ("h", 2), "uint16": ("H", 2),
            "int": ("i", 4), "uint": ("I", 4), "int32": ("i", 4), "uint32": ("I", 4),
            "float": ("f", 4), "float32": ("f", 4), "double": ("d", 8), "float64": ("d", 8)}


def parse_header(f):
    fmt, n_vertex, props, elem = "", 0, [], None
    while True:
        line = f.readline().decode("ascii", "replace").strip()
        if not line:
            break
        if line.startswith("format"):
            fmt = line.split()[1]
        elif line.startswith("element"):
            _, name, n = line.split()
            elem = name
            if name == "vertex":
                n_vertex = int(n)
        elif line.startswith("property") and elem == "vertex":
            t = line.split()
            props.append((t[-1], t[1]))
        elif line == "end_header":
            break
    return fmt, n_vertex, props, f.tell()


def scan(path, want=30000):
    with open(path, "rb") as f:
        fmt, nv, props, off = parse_header(f)
        names = [p[0] for p in props]
        if "red" not in names:
            return {"has_vertex_color": 0, "n_vertex": nv, "format": fmt}
        cols = []
        if fmt.startswith("ascii"):
            # 전체 라인을 스트리밍하며 step 간격으로 채집
            idx = {n: i for i, n in enumerate(names)}
            step = max(1, nv // want)
            f.seek(off)
            for i in range(nv):
                line = f.readline()
                if not line:
                    break
                if i % step:
                    continue
                t = line.split()
                if len(t) < len(names):
                    continue
                try:
                    cols.append((int(t[idx["red"]]), int(t[idx["green"]]), int(t[idx["blue"]])))
                except ValueError:
                    continue
        else:
            endian = "<" if "little" in fmt else ">"
            sfmt = endian + "".join(PLY_TYPE[t][0] for _, t in props)
            size = struct.calcsize(sfmt)
            ri = names.index("red")
            step = max(1, nv // want)
            for i in range(0, nv, step):
                f.seek(off + i * size)
                buf = f.read(size)
                if len(buf) < size:
                    break
                v = struct.unpack(sfmt, buf)
                cols.append((v[ri], v[ri + 1], v[ri + 2]))
    if not cols:
        return {"has_vertex_color": 1, "n_vertex": nv, "format": fmt, "n_sampled": 0}
    n = len(cols)
    uniq = {c for c in cols}
    gray = sum(1 for c in cols if c[0] == c[1] == c[2])
    mean = [sum(c[i] for c in cols) / n for i in range(3)]
    var = [sum((c[i] - mean[i]) ** 2 for c in cols) / n for i in range(3)]
    return {"has_vertex_color": 1, "n_vertex": nv, "format": fmt, "n_sampled": n,
            "n_unique_rgb": len(uniq), "unique_ratio": round(len(uniq) / n, 5),
            "mean_r": round(mean[0], 1), "mean_g": round(mean[1], 1), "mean_b": round(mean[2], 1),
            "std_r": round(var[0] ** .5, 2), "std_g": round(var[1] ** .5, 2),
            "std_b": round(var[2] ** .5, 2),
            "gray_pixel_ratio": round(gray / n, 4),
            "color_status": ("NO_COLOR_uniform" if len(uniq) <= 2 else
                             "WEAK_lowvariety" if len(uniq) / n < 0.01 else "OK")}


targets = []
for p in sorted(glob.glob(os.path.join(REPO, "data", "ply_files_excluding_20260702_extracted", "*.ply"))):
    targets.append(("archive_20260702", os.path.basename(p)[:-4], p))
for p in sorted(glob.glob(os.path.join(REPO, "data", "cad", "*", "*.ply"))):
    targets.append(("operational_cad", os.path.basename(os.path.dirname(p)), p))

rows = []
for src, name, p in targets:
    r = {"source": src, "name": name, "path": os.path.relpath(p, REPO),
         "size_mb": round(os.path.getsize(p) / 1e6, 2)}
    try:
        r.update(scan(p))
    except Exception as e:
        r["color_status"] = f"ERROR: {type(e).__name__}: {e}"
    rows.append(r)
    print(f"{src[:9]:9s} {name:30s} v={r.get('n_vertex',0):>9,d} "
          f"uniq={r.get('n_unique_rgb','-'):>7} ratio={r.get('unique_ratio','-'):>8} "
          f"rgb=({r.get('mean_r','-')},{r.get('mean_g','-')},{r.get('mean_b','-')}) "
          f"std=({r.get('std_r','-')},{r.get('std_g','-')},{r.get('std_b','-')}) "
          f"gray={r.get('gray_pixel_ratio','-'):>6} {r.get('color_status','')}")

cols = ["source", "name", "path", "size_mb", "format", "n_vertex", "has_vertex_color",
        "n_sampled", "n_unique_rgb", "unique_ratio", "mean_r", "mean_g", "mean_b",
        "std_r", "std_g", "std_b", "gray_pixel_ratio", "color_status"]
out = os.path.join(RES, "ply_color_scan.csv")
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
    w.writeheader(); w.writerows(rows)
print(f"\n-> {out}")
