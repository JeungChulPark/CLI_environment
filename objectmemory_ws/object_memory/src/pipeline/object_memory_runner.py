"""Object_memory runner + streaming core.

``StreamingObjectMemory`` holds the persistent state (store, live instances,
ages) and exposes ``step(dets, pose, ts)`` -- the per-frame body -- so the SAME
association/existence/fusion logic drives both the offline batch runner and the
real-time streaming driver. The only difference between them is HOW a camera
pose for a detection's stamp is obtained and in what order frames are fed:

  - offline batch (``run_object_memory``): all poses known up front; a
    detection's pose comes from ``_nearest_pose`` (may peek +/-1 sample, i.e.
    the future) and frames are walked in index order.
  - real-time (``scripts_test/run_realtime_object_memory.py``): poses arrive on
    a stream into a PoseBuffer; a late detection's pose is looked up at its
    capture stamp (past only). Same ``step``.

Association (multi-instance, Story 4/5 minimal form): a class (object_name) may
hold SEVERAL concurrent persistent instances, each its own landmark/object_id.
Each detection is matched, in map space, to the NEAREST live landmark of the
same class whose translation residual is within ``assoc_trans_gate_m`` (and,
if enabled, rotation within ``assoc_rot_gate_deg``); within a frame the pairing
is one-to-one (greedy by ascending distance, each landmark claimed once). A
detection matching NO live instance of its class SPAWNS a fresh tentative
instance instead of being rejected -- so a genuine second object, or a good
cluster that a poisoned first pose would otherwise shadow, forms its own track.
Transient noise that spawns a tentative is pruned by the existence filter
(below), so spawning-on-miss does not leak phantoms into the active set.
INV-002: Object Memory owns object_id; INV-003: detection_id is never identity.

Lifecycle: a landmark's ``confidence`` field holds its Bernoulli existence
probability r (core.existence), and every frame each unseen landmark is
penalised only in proportion to its expected detection probability P_D
(core.visibility) - a landmark outside the camera view keeps its r unchanged,
so time spent out of view never kills a real object. Discrete states are
derived from r thresholds:

    unmatched detection  -> spawn tentative, r = r_init       (initiator)
    accepted detection   -> r rises (Bayes); obs >= PROMOTE_HITS
                            and r > r_promote -> active
    unseen, in view      -> r falls; active drops below r_lost -> lost
    r below r_retire     -> tentative: deleted (ghost FP)     (deleter)
                            lost: deleted if obs < MIN_OBS_LONGTERM,
                                  remembered otherwise (same id kept)
    tentative older than tentative_max_age frames -> deleted (safety cap
                            for ghosts whose claimed position stays out of view)
    redetected           -> lost/remembered -> active
    depth check (opt-in, depth_check=DepthCheckConfig, 2026-09-17): an unseen in-view landmark whose spot the aligned
                            depth image sees THROUGH for confirm_frames consecutive frames -> deleted (any status,
                            remembered included, reason "depth_free_space"); an OCCLUDED spot suppresses the miss
                            penalty. Skipped while the camera moves faster than max_speed_mps / max_rot_dps.

A deleted landmark is removed from the live instance list of its class (it stays
in the store as a tombstone; INV-002: ids are never reused). Its siblings of the
same class persist, and a later detection either re-associates to a surviving
sibling or spawns a fresh instance.

INV-008: when no SLAM pose is available for a frame (tracking gap), object
memory is NOT updated for that frame; detections are dropped, not fused.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from core.existence import (
    update_existence_detected,
    update_existence_loglr,
    update_existence_missed,
)
from core.features import QualityWeights, detection_log_lr
from core.fusion import fuse_pose, update_rotation_modes
from core.memory_store import ObjectMemoryStore
from core.visibility import PD_BASE, expected_p_d
from core.depth_check import FREE, OCCLUDED, SURFACE, DepthCheckConfig, DepthFrame, depth_evidence
from core.map_occlusion import occluded_fraction
from core.models import (AssociationDecision, DecisionType, FusedPoseSample,
                         SlamCameraPose)
from core.state_machine import ObjectStatus
from core.transforms import (
    compute_T_map_obj,
    rotation_to_quat,
    transform_distance_rotation_deg,
    transform_distance_translation,
)

PROMOTE_HITS = 2
# short-term association gate defaults (map space).
ASSOC_TRANS_GATE_M = 0.15
ASSOC_ROT_GATE_DEG = 45.0
SCORE_FLOOR = 0.0
TENTATIVE_GATE_MULT = 3.0
# existence probability defaults (confidence field == r).
CLUTTER_RATIO = 0.1           # accepted-detection false-alarm likelihood ratio
R_INIT = 0.7                  # existence of a freshly seeded tentative
R_PROMOTE = 0.6               # tentative -> active needs r above this
R_LOST = 0.3                  # active -> lost when r falls below this
R_RETIRE = 0.05               # tentative/lost end their life below this
TENTATIVE_MAX_AGE = 100       # frames; safety cap for out-of-view ghosts
MIN_OBS_LONGTERM = 5          # retire split: fewer obs -> delete, else remember
# retire split also needs the evidence spread over time (2026-09-22): a false positive seen in one
# short burst (9 hits in 5.6 s on a GPU box read as 'milk') must not earn long-term memory.
EPISODE_GAP_S = 2.0           # an observation more than this after the previous one opens a new episode
MIN_EPISODES_LONGTERM = 2     # remember only with >= this many episodes ...
MIN_SPAN_LONGTERM_S = 8.0     # ... or observations spanning at least this long (10 s dropped a real Bear seen for 9.9 s; the milk FP spanned 5.6 s)
# remembered landmarks still die from in-view misses, just slowly: this many in-view frames without a
# re-detection since the last one (depth-OCCLUDED frames excluded) deletes them. None disables.
# a new instance spawned where same-class instances were retired for lack of evidence inherits their
# episodes and observations for the long-term decision (2026-09-22: a real Dinosaur first seen in one
# 4.9 s burst was retired, then re-seen twice in short bursts, and stayed off the map for 100 s).
INHERIT_REASONS = ("existence_floor", "tentative_timeout", "low_evidence")
# a mapped landmark farther than this from the camera is not updated from a miss at all (no existence decay,
# no remembered-miss count, no depth check): SAM-6D's detection rate falls from ~40-90 % inside 1 m to <10 %
# beyond 1.5 m and 0 % beyond 2.5 m (260826 and 260901 runs, 2026-09-22; ~10 % at 1-1.5 m), so not seeing a far object is no
# evidence that it is gone. Detections are still accepted at any range. None disables.
MAX_UPDATE_RANGE_M = 1.0
REMEMBERED_MISS_LIMIT = 60      # ~22 s in view at SAM-6D ~2.7 fps; real objects peaked at 23 on 260826 (Bear)
# Story 5 (quality-weighted confidence, opt-in via quality_weighting=True):
R_PRIOR = 0.15                # prior existence of an unconfirmed single detection


@dataclass
class ObjectFrameSnapshot:
    """Per-frame view of one known object's memory state (for visualization)."""
    object_id: int
    object_name: str
    status: ObjectStatus
    T_map_obj: tuple            # fused authoritative pose
    visible: bool               # measurement ACCEPTED into memory this frame
    rejected_this_frame: bool = False  # detected but every measurement rejected
    observation_count: int = 0  # total accepted observations so far
    T_map_obj_meas: Optional[tuple] = None  # this frame's raw measurement
    residual_t: Optional[float] = None      # meas vs fused, metres
    residual_deg: Optional[float] = None     # meas vs fused, degrees


@dataclass
class FrameResult:
    frame_idx: int
    stamp: Optional[float]
    slam_ok: bool
    decisions: List[AssociationDecision] = field(default_factory=list)
    # snapshot for visualization / diagnostics (populated per frame):
    camera_T_map_cam: Optional[tuple] = None
    objects: List[ObjectFrameSnapshot] = field(default_factory=list)


@dataclass
class RunResult:
    store: ObjectMemoryStore
    frame_results: List[FrameResult]
    # live instances per class at end of run (authoritative, multi-instance).
    live_ids_by_name: Dict[str, List[int]]
    # compat view: name -> first live object_id (ambiguous when a class holds
    # more than one live instance; prefer live_ids_by_name). Names with no live
    # instance are absent.
    object_ids_by_name: Dict[str, int]
    frames_total: int
    frames_with_slam: int
    frames_without_slam: int
    detections_total: int


def _snapshot(store, live_ids_by_name, seen_ids, meas_by_id=None,
              rejected_names=None):
    """Snapshot every live instance's current memory state for this frame."""
    meas_by_id = meas_by_id or {}
    rejected_names = rejected_names or set()
    snap: List[ObjectFrameSnapshot] = []
    for name, ids in live_ids_by_name.items():
        for object_id in ids:
            lm = store.get(object_id)
            if lm is None or lm.status in (ObjectStatus.deleted,
                                           ObjectStatus.merged):
                continue
            m = meas_by_id.get(object_id)
            seen = object_id in seen_ids
            snap.append(
                ObjectFrameSnapshot(
                    object_id=object_id,
                    object_name=name,
                    status=lm.status,
                    T_map_obj=lm.T_map_obj,
                    visible=seen,
                    rejected_this_frame=(name in rejected_names and not seen),
                    observation_count=lm.observation_count,
                    T_map_obj_meas=m[0] if m else None,
                    residual_t=m[1] if m else None,
                    residual_deg=m[2] if m else None,
                )
            )
    return snap


def _run_initiator(store, live_ids_by_name, det, name, score, T_meas, ts,
                   pose, r_init):
    """Initiator stage (Stone Soup pattern): spawn a fresh tentative instance of
    an object_name whose measurement matched no live instance (first sighting,
    a genuine second instance, or a good cluster shadowed by a poisoned pose)."""
    lm = store.create_tentative(
        object_name=name,
        T_map_obj=T_meas,
        first_seen_time=ts,
        confidence=r_init,
        class_id=det.class_id,
        last_detection_id=det.detection_id,
        last_sam6d_score=score,
        source_slam_status=pose.tracking_status,
    )
    live_ids_by_name.setdefault(name, []).append(lm.object_id)
    # 첫 측정도 한 표로 세어 둔다. 안 그러면 두 번째 측정이 첫 봉우리가 되어
    # 씨앗 자세가 통째로 버려진다.
    update_rotation_modes(lm.rot_modes,
                          tuple(tuple(T_meas[r][c] for c in range(3)) for r in range(3)),
                          name)
    return lm


def _inherit_retired_evidence(store, lm, T_meas, gate_m, consumed):
    """Give a freshly spawned instance the episodes and observations of same-class instances that were
    retired for lack of evidence within gate_m of it. Each tombstone is inherited once (``consumed``); its
    own inheritance comes along, so a chain of short sightings of one spot adds up."""
    episodes = obs = 0
    for old in store.all_landmarks():
        if (old.object_id == lm.object_id or old.object_id in consumed or old.object_name != lm.object_name
                or old.status is not ObjectStatus.deleted or not old.transition_history
                or old.transition_history[-1].reason not in INHERIT_REASONS):
            continue
        if transform_distance_translation(old.T_map_obj, T_meas) > gate_m:
            continue
        consumed.add(old.object_id)
        episodes += old.episode_count
        obs += old.observation_count + old.inherited_obs
    if episodes:
        store.update_landmark(lm.object_id, episode_count=lm.episode_count + episodes, inherited_obs=obs)


def _run_deleter(store, live_ids_by_name, seen_ids, ts, pose, ages,
                 cam_K, img_size, pd_base, r_lost, r_retire,
                 tentative_max_age, min_obs_longterm,
                 depth_frame=None, depth_cfg=None, depth_active=False, free_counts=None, depth_log=None,
                 min_episodes_longterm=1, min_span_longterm_s=0.0, remembered_miss_limit=None,
                 max_update_range_m=None, occlusion_extents=None, occlusion_fraction=0.5, occlusion_log=None):
    """Deleter stage (Stone Soup pattern): decay existence of unseen live
    instances in proportion to their expected detection probability P_D and
    retire the ones whose existence ran out. An instance outside the camera view
    (P_D = 0) keeps its r - out-of-view time never kills a real object. Returns
    the (object_name, object_id) pairs deleted this frame so the caller can drop
    them from the live instance lists."""
    released = []
    free_counts = free_counts if free_counts is not None else {}
    for name, ids in live_ids_by_name.items():
        for object_id in ids:
            if object_id in seen_ids:
                free_counts.pop(object_id, None)
                continue
            lm = store.get(object_id)
            if lm is None or lm.status in (ObjectStatus.deleted, ObjectStatus.merged):
                continue
            if max_update_range_m is not None and math.dist(
                    [pose.T_map_cam[i][3] for i in range(3)], [lm.T_map_obj[i][3] for i in range(3)]) > max_update_range_m:
                # too far to be detected: a miss says nothing. The tentative age cap still applies, so a
                # ghost claimed far away cannot linger.
                if lm.status is ObjectStatus.tentative and ages.get(object_id, 0) >= tentative_max_age:
                    store.apply_transition(object_id, ObjectStatus.deleted, "tentative_timeout", stamp=ts)
                    released.append((name, object_id))
                continue
            # map-based occlusion (opt-in): hidden behind established landmarks nearer the camera -> no miss this
            # frame. Catches occluders a few cm in front, which the depth check reads as the object's own surface.
            if occlusion_extents is not None:
                others = [store.get(i) for ids2 in live_ids_by_name.values() for i in ids2 if i != object_id]
                others = [o for o in others if o is not None and o.status in
                          (ObjectStatus.active, ObjectStatus.lost, ObjectStatus.remembered)]
                frac = occluded_fraction(pose.T_map_cam, lm, others, occlusion_extents, cam_K, img_size)
                if frac is not None and frac >= occlusion_fraction:
                    free_counts.pop(object_id, None)
                    if occlusion_log is not None:
                        occlusion_log[object_id] = occlusion_log.get(object_id, 0) + 1
                    continue
            # depth consistency (opt-in): runs for remembered landmarks too — they are exactly the stale copies
            if depth_cfg is not None and depth_active:
                verdict, info = depth_evidence(pose.T_map_cam, lm.T_map_obj, depth_frame, depth_cfg)
                if depth_log is not None:
                    depth_log["verdicts"][verdict] = depth_log["verdicts"].get(verdict, 0) + 1
                    if "why" in info:
                        key = f'{verdict}:{info["why"]}'
                        depth_log["unknown_why"][key] = depth_log["unknown_why"].get(key, 0) + 1
                    if len(depth_log["samples"]) < 400:
                        depth_log["samples"].append({"ts": round(ts, 3), "object_id": object_id, "name": name,
                                                     "status": lm.status.value, "verdict": verdict, **info,
                                                     "cam_t": [round(pose.T_map_cam[i][3], 3) for i in range(3)],
                                                     "cam_zaxis": [round(pose.T_map_cam[i][2], 3) for i in range(3)],
                                                     "obj_t": [round(lm.T_map_obj[i][3], 3) for i in range(3)]})
                if verdict == FREE:
                    status_before = lm.status.value
                    free_counts[object_id] = free_counts.get(object_id, 0) + 1
                    if free_counts[object_id] >= depth_cfg.confirm_frames:
                        store.apply_transition(object_id, ObjectStatus.deleted, "depth_free_space", stamp=ts,
                                               metadata={**info, "frames": free_counts[object_id],
                                                         "from": status_before, "obs": lm.observation_count})
                        if depth_log is not None:
                            depth_log["deleted"].append({"ts": ts, "object_id": object_id, "name": name,
                                                         "from": status_before, "obs": lm.observation_count, **info})
                        free_counts.pop(object_id, None)
                        released.append((name, object_id))
                        continue
                elif verdict == SURFACE:
                    free_counts.pop(object_id, None)
                elif verdict == OCCLUDED:
                    free_counts.pop(object_id, None)
                    continue          # hidden behind something: no miss penalty this frame
            if lm.status is ObjectStatus.remembered:
                # remembered = stable long-term memory: its existence no longer decays, but a spot that stays
                # in view without a re-detection for remembered_miss_limit frames is let go (a false positive
                # stuck on a real surface, which the depth check cannot see through).
                if remembered_miss_limit is None:
                    continue
                if expected_p_d(pose.T_map_cam, lm.T_map_obj, cam_K, img_size, pd_base=pd_base) > 0.0:
                    missed = lm.missed_count + 1
                    store.update_landmark(object_id, missed_count=missed)
                    if missed >= remembered_miss_limit:
                        store.apply_transition(object_id, ObjectStatus.deleted, "remembered_missed", stamp=ts,
                                               metadata={"missed": missed, "obs": lm.observation_count})
                        released.append((name, object_id))
                continue
            p_d = expected_p_d(pose.T_map_cam, lm.T_map_obj, cam_K, img_size,
                               pd_base=pd_base)
            r = update_existence_missed(lm.confidence, p_d)
            store.update_landmark(
                object_id,
                # missed_count counts IN-VIEW misses only (the ones that matter).
                missed_count=lm.missed_count + (1 if p_d > 0.0 else 0),
                confidence=r,
            )
            if lm.status is ObjectStatus.active and r < r_lost:
                store.apply_transition(
                    object_id, ObjectStatus.lost, "missed_threshold", stamp=ts
                )
            elif lm.status is ObjectStatus.tentative and r < r_retire:
                store.apply_transition(
                    object_id, ObjectStatus.deleted, "existence_floor", stamp=ts
                )
                released.append((name, object_id))
            elif (lm.status is ObjectStatus.tentative
                    and ages.get(object_id, 0) >= tentative_max_age):
                store.apply_transition(
                    object_id, ObjectStatus.deleted, "tentative_timeout", stamp=ts
                )
                released.append((name, object_id))
            elif lm.status is ObjectStatus.lost and r < r_retire:
                # Retire split: keep as long-term memory only if the track was
                # actually established. At retire time r has already decayed to
                # ~0, so it cannot distinguish here; an observation-count floor
                # is a pragmatic proxy in both modes (peak-confidence tracking is
                # a deferred refinement). This keeps a promoted-then-abandoned
                # weak track from earning permanent 'remembered' status.
                span = lm.last_seen_time - lm.first_seen_time
                if (lm.observation_count + lm.inherited_obs >= min_obs_longterm
                        and (lm.episode_count >= min_episodes_longterm or span >= min_span_longterm_s)):
                    store.apply_transition(
                        object_id, ObjectStatus.remembered, "long_term_memory",
                        stamp=ts,
                    )
                else:
                    store.apply_transition(
                        object_id, ObjectStatus.deleted, "low_evidence", stamp=ts
                    )
                    released.append((name, object_id))
    return released


def _nearest_pose(
    stamps: List[float],
    poses: List[SlamCameraPose],
    ts: float,
    tol: float,
) -> Optional[SlamCameraPose]:
    if not stamps:
        return None
    i = bisect.bisect_left(stamps, ts)
    best = None
    best_dt = tol
    for j in (i - 1, i, i + 1):
        if 0 <= j < len(stamps):
            dt = abs(stamps[j] - ts)
            if dt <= best_dt:
                best_dt = dt
                best = poses[j]
    return best


class StreamingObjectMemory:
    """Persistent object memory driven one frame at a time.

    Holds the store, the live-instance lists and the tentative ages, and applies
    a single frame's detections against a camera pose via ``step``. A frame with
    no pose (tracking gap) is handled by ``step_no_pose`` (INV-008). The offline
    ``run_object_memory`` and the real-time driver are both thin loops over these
    two methods -- so the fusion/existence/association behaviour is identical and
    the existing test-suite guards both."""

    def __init__(
        self,
        assoc_trans_gate_m: float = ASSOC_TRANS_GATE_M,
        assoc_rot_gate_deg: Optional[float] = ASSOC_ROT_GATE_DEG,
        score_floor: float = SCORE_FLOOR,
        tentative_gate_mult: float = TENTATIVE_GATE_MULT,
        pd_base: float = PD_BASE,
        clutter_ratio: float = CLUTTER_RATIO,
        r_init: float = R_INIT,
        r_promote: float = R_PROMOTE,
        r_lost: float = R_LOST,
        r_retire: float = R_RETIRE,
        tentative_max_age: int = TENTATIVE_MAX_AGE,
        min_obs_longterm: int = MIN_OBS_LONGTERM,
        cam_K=None,
        img_size=None,
        quality_weighting: bool = False,
        weights: Optional[QualityWeights] = None,
        r_prior: float = R_PRIOR,
        depth_check: Optional[DepthCheckConfig] = None,
        episode_gap_s: float = EPISODE_GAP_S,
        min_episodes_longterm: int = MIN_EPISODES_LONGTERM,
        min_span_longterm_s: float = MIN_SPAN_LONGTERM_S,
        remembered_miss_limit: Optional[int] = REMEMBERED_MISS_LIMIT,
        inherit_evidence: bool = True,
        max_update_range_m: Optional[float] = MAX_UPDATE_RANGE_M,
        occlusion_extents: Optional[dict] = None,
        occlusion_fraction: float = 0.5,
    ):
        self.assoc_trans_gate_m = assoc_trans_gate_m
        self.assoc_rot_gate_deg = assoc_rot_gate_deg
        self.rot_gate_off = assoc_rot_gate_deg is None or assoc_rot_gate_deg < 0.0
        self.score_floor = score_floor
        self.tentative_gate_mult = tentative_gate_mult
        self.pd_base = pd_base
        self.clutter_ratio = clutter_ratio
        self.r_init = r_init
        self.r_promote = r_promote
        self.r_lost = r_lost
        self.r_retire = r_retire
        self.tentative_max_age = tentative_max_age
        self.min_obs_longterm = min_obs_longterm
        self.episode_gap_s = episode_gap_s
        self.min_episodes_longterm = min_episodes_longterm
        self.min_span_longterm_s = min_span_longterm_s
        self.remembered_miss_limit = remembered_miss_limit
        self.inherit_evidence = inherit_evidence
        self.max_update_range_m = max_update_range_m
        # map-based occlusion (opt-in): {object_name: 8x3 CAD box corners in metres}; None keeps the old behaviour
        self.occlusion_extents = occlusion_extents
        self.occlusion_fraction = occlusion_fraction
        self.occlusion_log: Dict[int, int] = {}
        self._inherited: set = set()      # tombstone ids already inherited by a later instance
        self.cam_K = cam_K
        self.img_size = img_size
        self.quality_weighting = quality_weighting
        self.weights = weights if weights is not None else QualityWeights()
        self.logit_prior = math.log(r_prior / (1.0 - r_prior))

        self.depth_check = depth_check
        self._depth_free_counts: Dict[int, int] = {}
        self._prev_cam = None             # (T_map_cam, ts) for the depth check's speed gate
        self.depth_log = {"verdicts": {}, "unknown_why": {}, "samples": [], "deleted": [], "frames_checked": 0, "frames_skipped_speed": 0}
        self.store = ObjectMemoryStore()
        # multi-instance: a class name maps to a LIST of live object_ids.
        self.live_ids_by_name: Dict[str, List[int]] = {}
        self.ages: Dict[int, int] = {}  # frames since creation (tentative cap)
        self.detections_total = 0

    def step_no_pose(self, dets, ts, frame_idx: int) -> FrameResult:
        """INV-008: no camera pose this frame -> drop detections, freeze memory."""
        self.detections_total += len(dets)
        fr = FrameResult(frame_idx=frame_idx, stamp=ts, slam_ok=False)
        for det in dets:
            fr.decisions.append(
                AssociationDecision(
                    decision_type=DecisionType.rejected,
                    detection_id=det.detection_id,
                    score=min(max(det.score, 0.0), 1.0),
                    reject_reason="no SLAM pose within tolerance",
                )
            )
        fr.objects = _snapshot(self.store, self.live_ids_by_name, set())
        return fr

    def step(self, dets, pose: SlamCameraPose, ts, frame_idx: int, depth: Optional[DepthFrame] = None) -> FrameResult:
        """Apply one frame's detections against ``pose`` (at time ``ts``). ``depth``: aligned depth of the same
        frame for the opt-in depth check (ignored when the runner was built without ``depth_check``)."""
        self.detections_total += len(dets)
        store = self.store
        live_ids_by_name = self.live_ids_by_name
        ages = self.ages

        fr = FrameResult(frame_idx=frame_idx, stamp=ts, slam_ok=True)
        fr.camera_T_map_cam = pose.T_map_cam
        # seen == set of object_ids that ACCEPTED a measurement this frame.
        # An instance whose only detection went (as a gross outlier) to spawn a
        # different instance must NOT count as seen, or it freezes forever.
        seen_ids = set()
        rejected_names = set()
        meas_by_id = {}

        # --- stage 1: measurements + score-floor rejection -------------------
        pending = []   # (det, name, score, T_meas) that clear the score floor
        for det in dets:
            name = det.object_name
            score = min(max(det.score, 0.0), 1.0)
            T_meas = compute_T_map_obj(pose.T_map_cam, det.T_cam_obj)
            if score < self.score_floor:
                rejected_names.add(name)
                fr.decisions.append(AssociationDecision(
                    decision_type=DecisionType.rejected,
                    detection_id=det.detection_id,
                    score=score,
                    reject_reason="score below floor",
                    debug_info={"object_name": name},
                ))
                continue
            pending.append((det, name, score, T_meas))

        # --- stage 2: candidate (detection, live instance) pairs within gate -
        pairs = []   # (dt, idx_in_pending, object_id, dth)
        for i, (det, name, score, T_meas) in enumerate(pending):
            for object_id in live_ids_by_name.get(name, []):
                lm = store.get(object_id)
                if lm is None or lm.status in (ObjectStatus.deleted,
                                               ObjectStatus.merged):
                    continue
                dt = transform_distance_translation(lm.T_map_obj, T_meas)
                dth = transform_distance_rotation_deg(lm.T_map_obj, T_meas)
                # status-aware gate (looser while a tentative estimate forms).
                mult = (self.tentative_gate_mult
                        if lm.status is ObjectStatus.tentative else 1.0)
                if dt > self.assoc_trans_gate_m * mult:
                    continue
                if not self.rot_gate_off and dth > self.assoc_rot_gate_deg * mult:
                    continue
                pairs.append((dt, i, object_id, dth))

        # greedy one-to-one assignment: nearest pair first, each detection and
        # each live instance claimed at most once.
        pairs.sort(key=lambda p: p[0])
        assigned_det = {}     # idx_in_pending -> (object_id, dt, dth)
        claimed_ids = set()
        for dt, i, object_id, dth in pairs:
            if i in assigned_det or object_id in claimed_ids:
                continue
            assigned_det[i] = (object_id, dt, dth)
            claimed_ids.add(object_id)

        # --- stage 3: apply assignments (accept) then spawn the rest ----------
        for i, (det, name, score, T_meas) in enumerate(pending):
            match = assigned_det.get(i)
            if match is None:
                # matched no live instance of its class -> spawn a fresh one.
                # Quality mode: start from a low prior and let the first
                # detection's quality set the seed r (a degenerate/low-score
                # first sighting is born nearly dead; a clean one earns credit).
                if self.quality_weighting:
                    log_lr = detection_log_lr(
                        score, det.T_cam_obj[2][3], None, self.assoc_trans_gate_m,
                        self.pd_base, self.clutter_ratio, self.weights)
                    spawn_r = 1.0 / (1.0 + math.exp(-(self.logit_prior + log_lr)))
                else:
                    spawn_r = self.r_init
                lm = _run_initiator(store, live_ids_by_name, det, name,
                                    score, T_meas, ts, pose, spawn_r)
                if self.inherit_evidence:
                    _inherit_retired_evidence(store, lm, T_meas, self.assoc_trans_gate_m, self._inherited)
                ages[lm.object_id] = 0
                seen_ids.add(lm.object_id)
                meas_by_id[lm.object_id] = (T_meas, 0.0, 0.0)
                fr.decisions.append(AssociationDecision(
                    decision_type=DecisionType.new_tentative,
                    detection_id=det.detection_id,
                    object_id=lm.object_id,
                    score=score,
                    debug_info={"object_name": name},
                ))
                continue

            object_id, dt, dth = match
            lm = store.require(object_id)
            meas_by_id[object_id] = (T_meas, dt, dth)
            # accept -> incremental-mean fuse + Bayes existence update.
            seen_ids.add(object_id)
            n_prev = lm.observation_count
            T_fused = fuse_pose(lm.T_map_obj, n_prev, T_meas, lm.object_name, lm.rot_modes,
                                weight=float(getattr(det, "weight", 1.0) or 1.0))
            # 융합 포즈의 수렴 과정을 남긴다. fuse_pose 는 tracking 처럼 포즈를 제자리에서
            # 갱신하므로, 최종값만 두면 초반 큰 보정이 점점 잦아드는 모습이 사라진다.
            lm.pose_history.append(FusedPoseSample(
                stamp=det.stamp,
                observation_count=n_prev + 1,
                xyz=(T_fused[0][3], T_fused[1][3], T_fused[2][3]),
                quat_xyzw=tuple(rotation_to_quat(
                    tuple(tuple(T_fused[r][c] for c in range(3)) for r in range(3)))),
            ))
            if self.quality_weighting:
                log_lr = detection_log_lr(
                    score, det.T_cam_obj[2][3], dt, self.assoc_trans_gate_m,
                    self.pd_base, self.clutter_ratio, self.weights)
                new_r = update_existence_loglr(lm.confidence, log_lr)
            else:
                new_r = update_existence_detected(lm.confidence, self.pd_base,
                                                  self.clutter_ratio)
            new_episode = ts - lm.last_seen_time > self.episode_gap_s
            store.update_landmark(
                object_id,
                T_map_obj=T_fused,
                episode_count=lm.episode_count + (1 if new_episode else 0),
                last_seen_time=ts,
                observation_count=n_prev + 1,
                missed_count=0,
                confidence=new_r,
                last_detection_id=det.detection_id,
                last_sam6d_score=score,
                source_slam_status=pose.tracking_status,
            )
            # promote / reacquire.
            # Legacy: promote needs a hit count (obs >= PROMOTE_HITS) AND r.
            # Quality mode: promote on the confidence band alone (r >= r_promote).
            # Because a garbage detection now adds little/negative evidence, an
            # evidence-mass gate replaces the hit count: two clean detections or
            # one excellent one crosses it, two poor ones do not.
            if self.quality_weighting:
                promote_ok = new_r >= self.r_promote
            else:
                promote_ok = (lm.observation_count >= PROMOTE_HITS
                              and new_r > self.r_promote)
            if lm.status in (ObjectStatus.lost, ObjectStatus.remembered):
                store.apply_transition(
                    object_id, ObjectStatus.active, "redetected", stamp=ts
                )
            elif lm.status is ObjectStatus.tentative and promote_ok:
                store.apply_transition(
                    object_id, ObjectStatus.active, "promoted", stamp=ts
                )
            fr.decisions.append(AssociationDecision(
                decision_type=DecisionType.short_term_match,
                detection_id=det.detection_id,
                object_id=object_id,
                score=score,
                debug_info={
                    "object_name": name,
                    "residual_t": round(dt, 4),
                    "residual_deg": round(dth, 2),
                },
            ))

        # age all live tentatives (for the out-of-view ghost safety cap).
        for oid in list(ages):
            ages[oid] += 1

        # missed instances this frame: decay existence and retire dead ones.
        depth_active = False
        if self.depth_check is not None and depth is not None:
            depth_active = True
            if self._prev_cam is not None and ts > self._prev_cam[1]:
                Tp, tp = self._prev_cam
                dt = ts - tp
                dpos = math.sqrt(sum((pose.T_map_cam[i][3] - Tp[i][3]) ** 2 for i in range(3)))
                tr = sum(sum(pose.T_map_cam[i][k] * Tp[i][k] for i in range(3)) for k in range(3))
                drot = math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))
                if dpos / dt > self.depth_check.max_speed_mps or drot / dt > self.depth_check.max_rot_dps:
                    depth_active = False
                    self.depth_log["frames_skipped_speed"] += 1
            if depth_active:
                self.depth_log["frames_checked"] += 1
        self._prev_cam = (pose.T_map_cam, ts)
        released = _run_deleter(
            store, live_ids_by_name, seen_ids, ts, pose, ages,
            self.cam_K, self.img_size, self.pd_base, self.r_lost, self.r_retire,
            self.tentative_max_age, self.min_obs_longterm,
            depth_frame=depth, depth_cfg=self.depth_check, depth_active=depth_active,
            free_counts=self._depth_free_counts, depth_log=self.depth_log,
            min_episodes_longterm=self.min_episodes_longterm, min_span_longterm_s=self.min_span_longterm_s,
            remembered_miss_limit=self.remembered_miss_limit, max_update_range_m=self.max_update_range_m,
            occlusion_extents=self.occlusion_extents, occlusion_fraction=self.occlusion_fraction,
            occlusion_log=self.occlusion_log,
        )
        for name, object_id in released:
            # drop the deleted instance from its class's live list.
            ids = live_ids_by_name.get(name)
            if ids and object_id in ids:
                ids.remove(object_id)
            if ids == []:
                del live_ids_by_name[name]

        fr.objects = _snapshot(store, live_ids_by_name, seen_ids,
                               meas_by_id, rejected_names)
        return fr

    def object_ids_by_name(self) -> Dict[str, int]:
        return {name: ids[0]
                for name, ids in self.live_ids_by_name.items() if ids}


def run_object_memory(
    slam_poses: List[SlamCameraPose],
    detections_by_frame: Dict[int, list],
    timestamps: List[float],
    time_tolerance: float = 0.05,
    assoc_trans_gate_m: float = ASSOC_TRANS_GATE_M,
    assoc_rot_gate_deg: Optional[float] = ASSOC_ROT_GATE_DEG,
    score_floor: float = SCORE_FLOOR,
    tentative_gate_mult: float = TENTATIVE_GATE_MULT,
    pd_base: float = PD_BASE,
    clutter_ratio: float = CLUTTER_RATIO,
    r_init: float = R_INIT,
    r_promote: float = R_PROMOTE,
    r_lost: float = R_LOST,
    r_retire: float = R_RETIRE,
    tentative_max_age: int = TENTATIVE_MAX_AGE,
    min_obs_longterm: int = MIN_OBS_LONGTERM,
    cam_K=None,
    img_size=None,
    quality_weighting: bool = False,
    weights: Optional[QualityWeights] = None,
    r_prior: float = R_PRIOR,
) -> RunResult:
    """Offline batch runner: walk frames in index order, fusing each frame's
    detections against the nearest SLAM pose (``_nearest_pose`` may use a future
    sample -- this is the batch-merge behaviour the streaming driver replaces)."""
    poses = sorted(slam_poses, key=lambda p: p.stamp)
    stamps = [p.stamp for p in poses]

    mem = StreamingObjectMemory(
        assoc_trans_gate_m=assoc_trans_gate_m,
        assoc_rot_gate_deg=assoc_rot_gate_deg,
        score_floor=score_floor,
        tentative_gate_mult=tentative_gate_mult,
        pd_base=pd_base,
        clutter_ratio=clutter_ratio,
        r_init=r_init,
        r_promote=r_promote,
        r_lost=r_lost,
        r_retire=r_retire,
        tentative_max_age=tentative_max_age,
        min_obs_longterm=min_obs_longterm,
        cam_K=cam_K,
        img_size=img_size,
        quality_weighting=quality_weighting,
        weights=weights,
        r_prior=r_prior,
    )

    frame_results: List[FrameResult] = []
    frames_with_slam = 0
    frames_without_slam = 0

    for frame_idx in sorted(detections_by_frame.keys()):
        dets = detections_by_frame[frame_idx]
        if 0 <= frame_idx < len(timestamps):
            ts = timestamps[frame_idx]
        elif dets:
            ts = dets[0].stamp
        else:
            ts = None

        pose = (_nearest_pose(stamps, poses, ts, time_tolerance)
                if ts is not None else None)
        if pose is None:
            frames_without_slam += 1
            fr = mem.step_no_pose(dets, ts, frame_idx)
        else:
            frames_with_slam += 1
            fr = mem.step(dets, pose, ts, frame_idx)
        frame_results.append(fr)

    return RunResult(
        store=mem.store,
        frame_results=frame_results,
        live_ids_by_name=mem.live_ids_by_name,
        object_ids_by_name=mem.object_ids_by_name(),
        frames_total=len(detections_by_frame),
        frames_with_slam=frames_with_slam,
        frames_without_slam=frames_without_slam,
        detections_total=mem.detections_total,
    )
