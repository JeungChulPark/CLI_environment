#!/usr/bin/env python3
"""00_preflight.py — validate templates, CADs, and bag playability before a run.

Writes outputs_e2e/_logs/preflight.json with per-object and per-bag status and
skip lists. Never aborts; missing assets are reported and skipped.

Run (ROS env, for bag checks):
  conda run -n sam6d_ros_humble python tools/e2e_pipeline/00_preflight.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

REQUIRED_TOPIC_KINDS = {"color": "Image", "depth": "Image", "info": "CameraInfo"}


def check_objects():
    objs = {}
    for obj in L.list_objects():
        ok_t, det_t = L.template_assets_ok(obj)
        cad = L.resolve_cad(obj)
        ok = ok_t and cad is not None
        objs[obj] = {
            "ok": ok,
            "templates": det_t,
            "cad": cad,
            "reason": None if ok else ("template: " + det_t if not ok_t else "no CAD .ply"),
        }
    return objs


def check_bag(bag):
    path = L.resolve_bag_path(bag)
    if path is None:
        return {"ok": False, "reason": "no .db3/.mcap found", "path": None}
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        storage_id = "mcap" if path.endswith(".mcap") else "sqlite3"
        reader = SequentialReader()
        reader.open(StorageOptions(uri=path, storage_id=storage_id), ConverterOptions("", ""))
        topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    except Exception as e:
        return {"ok": False, "reason": f"reader open failed: {e}", "path": path}

    img_topics = [n for n, ty in topics.items() if "Image" in ty]
    info_topics = [n for n, ty in topics.items() if "CameraInfo" in ty]
    color = next((n for n in img_topics if "color" in n and "depth" not in n), None)
    depth = next((n for n in img_topics if "depth" in n), None)
    info = next((n for n in info_topics if "color" in n), info_topics[0] if info_topics else None)
    ok = bool(color and depth)
    return {
        "ok": ok, "path": path, "color_topic": color, "depth_topic": depth,
        "info_topic": info, "all_image_topics": img_topics,
        "reason": None if ok else "missing color/depth Image topic",
    }


def main():
    os.makedirs(os.path.join(L.OUT, "_logs"), exist_ok=True)
    objs = check_objects()
    bags = {b: check_bag(b) for b in L.TARGET_BAGS}

    ok_objs = [o for o, v in objs.items() if v["ok"]]
    skip_objs = [o for o, v in objs.items() if not v["ok"]]
    ok_bags = [b for b, v in bags.items() if v["ok"]]
    skip_bags = [b for b, v in bags.items() if not v["ok"]]

    report = {
        "objects": objs, "bags": bags,
        "summary": {
            "objects_ok": ok_objs, "objects_skip": skip_objs,
            "bags_ok": ok_bags, "bags_skip": skip_bags,
            "n_objects_ok": len(ok_objs), "n_bags_ok": len(ok_bags),
        },
    }
    out_path = os.path.join(L.OUT, "_logs", "preflight.json")
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)

    print("=" * 60)
    print(f"PREFLIGHT  objects OK {len(ok_objs)}/{len(objs)}  bags OK {len(ok_bags)}/{len(bags)}")
    if skip_objs:
        print("  SKIP objects:")
        for o in skip_objs:
            print(f"    - {o}: {objs[o]['reason']}")
    if skip_bags:
        print("  SKIP bags:")
        for b in skip_bags:
            print(f"    - {b}: {bags[b]['reason']}")
    print(f"  wrote {out_path}")


if __name__ == "__main__":
    main()
