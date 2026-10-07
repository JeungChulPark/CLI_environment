"""Proposal sources (the detector in front of our ISM gates) for the detector study.

Every proposer turns one BGR image into [(xyxy, obj_id, conf), ...] for the active target
objects. run_ours.py --proposer <spec> puts one in place of the deployed YOLO-World call;
everything after it (DINOv2 / MobileSAM gates, exclusive assignment, PEM, pose verification)
is unchanged.

  text:<weights>[:<prompts.json>]   YOLO-World or YOLOE with text prompts. prompts.json maps
                                    obj_id -> prompt or [prompts]; several prompts per object
                                    are run in one pass and merged per object (NMS 0.7, keep
                                    the highest score). Default prompts = paths.PROMPTS.
  visual:<weights>[:<n_views>[:multi]] YOLOE with visual prompt embeddings built from the CAD
                                    template renders (mask prompts, mean over n_views views).
  fastsam                           FastSAM-x segment-everything boxes, upstream SAM-6D settings
                                    (iou 0.9, conf 0.25, max_det 200, imgsz 640; boxes smaller than
                                    5 % of the image side or masks below 3e-4 of the image dropped),
                                    offered to every active object (option A of the ladder study).
  union:<spec>+<spec>               boxes of both, merged per object.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import paths as P  # noqa: E402

WDIR = P.SR / "weights" / "det_study"


def _w(name):
    for c in (Path(name), WDIR / name, P.SR / name):
        if c.exists():
            return str(c)
    raise FileNotFoundError(name)


def nms_per_object(dets, iou=0.7):
    out = []
    by = {}
    for d in dets:
        by.setdefault(d[1], []).append(d)
    for oid, ds in by.items():
        ds.sort(key=lambda d: -d[2])
        keep = []
        for d in ds:
            if all(_iou(d[0], k[0]) < iou for k in keep):
                keep.append(d)
        out += keep
    return out


def _iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    i = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


class TextProposer:
    def __init__(self, weights, prompts=None, device="cuda:0", imgsz=640):
        self.weights = _w(weights)
        self.yoloe = "yoloe" in Path(self.weights).name
        if self.yoloe:
            from ultralytics import YOLOE
            self.m = YOLOE(self.weights)
        else:
            from ultralytics import YOLOWorld
            self.m = YOLOWorld(self.weights)
        self.m.to(device)
        pm = {i + 1: [p] for i, p in enumerate(P.PROMPTS)}
        if prompts:
            for k, v in (json.load(open(prompts)) if isinstance(prompts, str) else prompts).items():
                pm[int(k)] = [v] if isinstance(v, str) else list(v)
        self.pm = pm
        self.device, self.imgsz = device, imgsz
        self.active = None
        self.name = f"text:{Path(self.weights).name}"

    def activate(self, oids):
        key = tuple(oids)
        if key == self.active:
            return
        self.cls_oid, names = [], []
        for o in oids:
            for p in self.pm[o]:
                if p not in names:          # a prompt shared by two objects proposes for both
                    names.append(p)
                self.cls_oid.append((names.index(p), o))
        self.names = names
        if self.yoloe:
            self.m.set_classes(names, self.m.get_text_pe(names))
        else:
            self.m.set_classes(names)
            cm = getattr(self.m.model, "clip_model", None)
            if cm is not None:
                cm.device = next(cm.model.parameters()).device
        self.route = {}
        for ci, o in self.cls_oid:
            self.route.setdefault(ci, []).append(o)
        self.active = key

    def predict(self, bgr, conf=0.02):
        r = self.m.predict(bgr, conf=conf, imgsz=self.imgsz, verbose=False, device=self.device)[0]
        out = []
        if r.boxes is not None:
            for xy, c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()):
                for o in self.route.get(int(c), []):
                    out.append((xy, o, float(s)))
        return nms_per_object(out)


class VisualProposer:
    """YOLOE with visual prompt embeddings (VPE) from the rendered CAD templates (zero-shot: CAD only)."""

    def __init__(self, weights="yoloe-26l-seg.pt", n_views=12, device="cuda:0", imgsz=640, multi=False):
        from ultralytics import YOLOE
        self.weights = _w(weights)
        self.m = YOLOE(self.weights)
        self.m.to(device)
        self.device, self.imgsz, self.n_views = device, imgsz, int(n_views)
        self.multi = bool(multi)          # every view its own class (routed to the object) instead of the mean
        cache = P.WORK / "det_study" / f"vpe_views_{Path(self.weights).stem}_{self.n_views}.pt"
        if cache.exists():
            self.vpe = torch.load(cache)
        else:
            self.vpe = self._build()
            cache.parent.mkdir(parents=True, exist_ok=True)
            torch.save(self.vpe, cache)
        self.active = None
        self.name = f"visual:{Path(self.weights).name}:{self.n_views}" + (":multi" if self.multi else "")

    def _build(self):
        from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor
        pr = YOLOEVPSegPredictor(overrides={"task": "segment", "mode": "predict", "save": False, "verbose": False,
                                            "batch": 1, "device": self.device, "imgsz": self.imgsz})
        self.m.model.model[-1].nc = 1
        pr.setup_model(model=self.m.model, verbose=False)
        vpe = {}
        for oid in range(1, 22):
            td = P.TEMPLATES / f"obj_{oid:06d}" / "templates"
            n_all = len(list(td.glob("rgb_*.png")))
            views = np.linspace(0, n_all - 1, self.n_views).round().astype(int)
            embs = []
            for v in views:
                rgb = cv2.imread(str(td / f"rgb_{v}.png"))
                mk = cv2.imread(str(td / f"mask_{v}.png"), cv2.IMREAD_GRAYSCALE)
                if rgb is None or mk is None or mk.max() == 0:
                    continue
                # paste the render onto a mid-grey 640x480 canvas at a typical YCB-V object size
                ys, xs = np.nonzero(mk)
                x1, y1, x2, y2 = xs.min(), ys.min(), xs.max() + 1, ys.max() + 1
                crop, cm = rgb[y1:y2, x1:x2], mk[y1:y2, x1:x2]
                s = 200.0 / max(crop.shape[:2])
                crop = cv2.resize(crop, None, fx=s, fy=s); cm = cv2.resize(cm, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
                can = np.full((480, 640, 3), 114, np.uint8); cmask = np.zeros((480, 640), np.uint8)
                h, w = cm.shape; oy, ox = (480 - h) // 2, (640 - w) // 2
                sel = cm > 0
                can[oy:oy + h, ox:ox + w][sel] = crop[sel]
                cmask[oy:oy + h, ox:ox + w] = sel
                pr.set_prompts({"cls": np.array([0]), "bboxes": np.array([[ox, oy, ox + w, oy + h]], float)})
                e = pr.get_vpe(can)                       # (1, 1, D)
                embs.append(torch.nn.functional.normalize(e.reshape(-1).float(), dim=0))
            vpe[oid] = torch.stack(embs).cpu()          # (views, D), each L2-normalised
            print(f"[vpe] obj {oid}: {len(embs)} views", flush=True)
        return vpe

    def activate(self, oids):
        key = tuple(oids)
        if key == self.active:
            return
        if self.multi:
            rows = [(o, e) for o in oids for e in self.vpe[o]]
        else:
            rows = [(o, torch.nn.functional.normalize(self.vpe[o].mean(0), dim=0)) for o in oids]
        names = [f"c{j}" for j in range(len(rows))]
        emb = torch.stack([e for _, e in rows])[None].to(self.device)
        self.m.model.model[-1].nc = len(names)
        self.m.model.set_classes(names, emb)
        self.m.predictor = None
        self.oids = [o for o, _ in rows]
        self.active = key

    def predict(self, bgr, conf=0.02):
        r = self.m.predict(bgr, conf=conf, imgsz=self.imgsz, verbose=False, device=self.device)[0]
        out = []
        if r.boxes is not None:
            for xy, c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()):
                out.append((xy, self.oids[int(c)], float(s)))
        return nms_per_object(out)


class FastSAMProposer:
    def __init__(self, device="cuda:0"):
        from ultralytics import YOLO
        self.m = YOLO(str(P.FASTSAM_X))
        self.args = dict(iou=0.9, conf=0.25, max_det=200, imgsz=640, verbose=False, device=device, half=False,
                         save=False)
        self.oids = []
        self.name = "fastsam-x (all boxes to every object)"

    def activate(self, oids):
        self.oids = list(oids)

    def predict(self, bgr, conf=0.02):
        r = self.m.predict(bgr, **self.args)[0]
        if r.boxes is None or r.masks is None:
            return []
        h, w = bgr.shape[:2]
        area = r.masks.data.float().sum(dim=(1, 2)) / float(r.masks.data.shape[1] * r.masks.data.shape[2])
        out = []
        for xy, s, ma in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), area.tolist()):
            bw, bh = xy[2] - xy[0], xy[3] - xy[1]
            if bw * bh / (w * h) <= 0.05 ** 2 or ma <= 3e-4:
                continue
            out += [(xy, o, float(s)) for o in self.oids]
        return out


class UnionProposer:
    def __init__(self, parts):
        self.parts = parts
        self.name = "union:" + "+".join(p.name for p in parts)

    def activate(self, oids):
        for p in self.parts:
            p.activate(oids)

    def predict(self, bgr, conf=0.02):
        return nms_per_object([d for p in self.parts for d in p.predict(bgr, conf)])


def build(spec, device="cuda:0"):
    if "+" in spec:
        return UnionProposer([build(s, device) for s in spec.split("+")])
    if spec == "fastsam":
        return FastSAMProposer(device)
    kind, *rest = spec.split(":")
    if kind == "text":
        return TextProposer(rest[0], rest[1] if len(rest) > 1 else None, device)
    if kind == "visual":
        return VisualProposer(rest[0] if rest else "yoloe-26l-seg.pt", rest[1] if len(rest) > 1 else 12, device,
                              multi=len(rest) > 2 and rest[2] == "multi")
    raise ValueError(spec)
