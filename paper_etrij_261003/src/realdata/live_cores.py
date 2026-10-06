"""Recognisers for the real-data live comparison (260901_cbnu_bigeightcircle), loaded by
sam6d_infer.py through OBJPOSE_RECOGNIZER (hub --recognizer <this file>).

The object list is the deployed one (the hub's ISM config: enabled objects, their 42-view
template folders and the PEM model points assets/model_points/<name>.npy, sampled from the CAD
in mm). The upstream code reads the CAD mesh only to sample points, so the model points stand in.

OBJPOSE_LIVE_MODE
  orig     original SAM-6D (run_orig.run_frame): FastSAM-x, DINOv2 (OBJPOSE_ORIG_DESC, default
           ViT-L), upstream scores, top-1 per object, PEM if ISM score > OBJPOSE_ORIG_THRESH
  hybrid   the same original ISM front end, then OUR recogniser's PEM + pose verification
           (Sam6DCore with its ISM replaced; same as run_ours.py --orig-ism)
"""
import os
import sys
from pathlib import Path

import numpy as np
import torch
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "ycbv"))
import paths as P  # noqa: E402

# run_orig puts the UPSTREAM SAM-6D Pose_Estimation_Model on sys.path. It must be imported only
# after Sam6DCore has loaded OUR pose_estimation_model (with pose verification); importing it
# first makes Sam6DCore pick up the upstream module, which returns no verification at all.
RO = None


def _load_ro():
    global RO
    if RO is None:
        import run_orig
        RO = run_orig
    return RO

SR = P.SR


class _Pts:
    def __init__(self, pts_m):
        self.p = pts_m

    def sample(self, n):
        idx = np.random.choice(len(self.p), n, replace=len(self.p) < n)
        return self.p[idx] * 1000.0          # upstream divides by 1000 (mesh in mm)


def _objects(cfg):
    path = cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml")
    path = path if os.path.isabs(path) else str(SR / path)
    d = yaml.safe_load(open(path))
    objs = []
    for o in d["objects"]:
        if not o.get("enabled", True):
            continue
        td = o["template_dir"]
        td = Path(td if os.path.isabs(td) else SR / td)
        pts = SR / "assets" / "model_points" / f"{o['name']}.npy"
        if td.exists() and pts.exists():
            objs.append((o["name"], td, np.load(pts).astype(np.float32) / 1000.0))
    return objs


def build_orig(cfg, desc):
    _load_ro()
    objs = _objects(cfg)
    names = [n for n, _, _ in objs]
    tdirs = {n: t for n, t, _ in objs}
    pts = {n: p for n, _, p in objs}
    RO.OIDS = names
    RO.tdir = lambda n: tdirs[n]
    RO.cad = lambda n: n
    import trimesh
    real = trimesh.load_mesh
    trimesh.load_mesh = lambda key, *a, **k: _Pts(pts[key]) if key in pts else real(key, *a, **k)
    try:
        ism, bank = RO.build_ism(model_name=desc)
        net, tcfg, pobj = RO.build_pem()
    finally:
        trimesh.load_mesh = real
    ism.ref_data = bank                      # bank built for exactly these objects, in order
    return names, ism, net, tcfg, pobj


class LiveCore:
    def __init__(self, cfg):
        np.random.seed(0); torch.manual_seed(0)
        self.mode = os.environ.get("OBJPOSE_LIVE_MODE", "orig")
        self.thresh = float(os.environ.get("OBJPOSE_ORIG_THRESH", 0.2))
        self.desc = os.environ.get("OBJPOSE_ORIG_DESC", "dinov2_vitl14")
        self.extra_proposals = None
        self.last_frame_diag = {"pem_candidates": [], "rejections": [], "pem_error": None}
        if self.mode == "orig":
            self.names, self.ism, self.net, self.tcfg, self.pobj = build_orig(cfg, self.desc)
            self.objs = [{"name": n} for n in self.names]
            self.verify = {}
        else:
            self._build_hybrid(cfg)
        print(f"[live-core] mode={self.mode} objects={len(self.objs)} ISM>{self.thresh} {self.desc}", flush=True)

    # ---------------------------------------------------------------- hybrid
    def _build_hybrid(self, cfg):
        sys.path.insert(0, str(SR / "realtime")); sys.path.insert(0, str(SR))
        import sam6d_core
        import yolo_ism_object_n as o_n
        import verify_config as VC
        rt = cfg.get("runtime", {})
        cwd = os.getcwd(); os.chdir(SR)
        self.core = sam6d_core.Sam6DCore(cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"),
                                         cfg.get("ism", {}).get("objects", []), rt.get("device", "cuda:0"),
                                         rt.get("det_score_thresh", 0.2), appe_rerank=rt.get("appe_rerank"),
                                         verify=rt.get("verify", VC.UNSET), pem_diagnostic=rt.get("pem_diagnostic"))
        os.chdir(cwd)
        _load_ro()                                   # only now (see RO above)
        names = [o["name"] for o in self.core.objs]
        import trimesh
        patched = trimesh.load_mesh
        objs = {n: (t, p) for n, t, p in _objects(cfg)}
        RO.OIDS = [n for n in names if n in objs]
        RO.tdir = lambda n: objs[n][0]
        RO.cad = lambda n: n
        trimesh.load_mesh = lambda key, *a, **k: _Pts(objs[key][1])
        try:
            self.ism, bank = RO.build_ism(model_name=self.desc)
        finally:
            trimesh.load_mesh = patched
        self.ism.ref_data = bank
        self.objs, self.verify = self.core.objs, self.core.verify
        cur = self.cur = {}
        ism, thr = self.ism, self.thresh

        def orig_recognize(groups, pb, bgr, rgb, norm_full, *a, **k):
            res = {n: {"accepted": False, "mask": None, "decision": "no-object(orig)"} for n in names}
            props = ism.segmentor_model.generate_masks(rgb)
            if props is None:
                return res
            dets = RO.Detections(props)
            dets.remove_very_small_detections(config=ism.post_processing_config.mask_post_processing)
            if len(dets) == 0:
                return res
            qd, qa = ism.descriptor_model(rgb, dets)
            sel, pio, sem, best = ism.compute_semantic_score(qd)
            dets.filter(sel); qa = qa[sel, :]
            if len(dets) == 0:
                return res
            appe, ref_aux = ism.compute_appearance_score(best, pio, qa)
            batch = {"depth": torch.from_numpy(cur["raw"].astype(np.int32)).unsqueeze(0).to(RO.DEV),
                     "cam_intrinsic": torch.from_numpy(cur["K"]).unsqueeze(0).to(RO.DEV),
                     "depth_scale": torch.from_numpy(np.array(1.0)).unsqueeze(0).to(RO.DEV)}
            uv = ism.project_template_to_image(best, pio, batch, dets.masks)
            geo, vis = ism.compute_geometric_score(uv, dets, qa, ref_aux, visible_thred=ism.visible_thred)
            final = (sem + appe + geo * vis) / (1 + 1 + vis)
            dets.add_attribute("scores", final); dets.add_attribute("object_ids", pio)
            dets.apply_nms_per_object_id(nms_thresh=ism.post_processing_config.nms_thresh)
            for kk in torch.unique(dets.object_ids).tolist():
                idx = torch.nonzero(dets.object_ids == kk)[:, 0]
                j = int(idx[torch.argmax(dets.scores[idx])]); sc = float(dets.scores[j])
                if sc <= thr:
                    continue
                m = (dets.masks[j] > 0.5).cpu().numpy()
                ys, xs = np.nonzero(m)
                if len(ys) == 0:
                    continue
                res[RO.OIDS[int(kk)]] = {"accepted": True, "mask": m, "decision": "detected(orig)",
                                         "box": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1],
                                         "best_sem": sc, "masked_appe": sc}
            return res

        class _B:
            def __init__(s):
                s.xyxy = torch.zeros((0, 4)); s.cls = torch.zeros(0); s.conf = torch.zeros(0)

            def __len__(s):
                return 0

        class _R:
            boxes = _B()
        o_n.recognize_frame_auto = orig_recognize
        self.core.yolo.predict = lambda bgr, **k: [_R()]

    # ---------------------------------------------------------------- frame
    def process(self, bgr, depth, K, want_mask=False, diagnostic_references=None, slam_context=None):
        K = np.asarray(K, np.float64)
        if self.mode == "orig":
            raw = np.asarray(depth).astype(np.int32)
            T, out = RO.run_frame(self.ism, self.net, self.tcfg, self.pobj, self.names, bgr, raw,
                                  np.asarray(depth, np.float32), K, 1.0, det_thresh=self.thresh)
            rows = [{"object": p["obj_id"], "score": round(float(p["score"]), 4),
                     "R": [[round(float(v), 6) for v in r] for r in p["R"]],
                     "t_mm": [round(float(v), 4) for v in p["t_mm"]], "pose_source": "sam6d_original"}
                    for p in out["preds"]]
            self.last_frame_diag = {"pem_candidates": [], "rejections": [], "pem_error": None}
            ms = {"yolo": round(T.get("proposals", 0)), "ism": round(T.get("descriptors", 0) + T.get("matching", 0)),
                  "pem": round(T.get("pem", 0)), "total": round(T["total"])}
            return rows, ms, out["n_fastsam"], None
        self.cur.update(raw=np.asarray(depth).astype(np.int32), K=K)
        r = self.core.process(bgr, depth, K, want_mask=want_mask, slam_context=slam_context)
        self.last_frame_diag = self.core.last_frame_diag
        return r
