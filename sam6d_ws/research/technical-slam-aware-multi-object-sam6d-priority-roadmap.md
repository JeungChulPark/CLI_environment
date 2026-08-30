# SLAM-Aware Multi-Object SAM-6D 파이프라인: 우선순위 기반 단계형 구현 로드맵 (Technical Research)

> **작성일**: 2026-06-25
> **연구자**: jucpark
> **목적**: SAM-6D 출력 6D pose를 SLAM/map context와 결합하여 실제 로봇 환경에서 안정적으로 사용하는 구조를 조사하고, 1~2개월 내 중간에 멈추더라도 논문/보고서 contribution으로 방어 가능한 우선순위·stop-point 기반 구현 계획을 제안한다.
> **제약**: 코드 미수정(조사·설계만), PEM 내부 알고리즘 수정은 우선순위에서 제외, 한국어 보고서.

---

## 0. 핵심 사실 검증 (이 보고서의 전제)

본 보고서의 모든 판단은 다음 검증된 사실 위에 서 있다.

1. **SAM-6D의 PEM은 single-instance-per-call이다.** PEM은 segmented mask 1개 + CAD 템플릿 1세트를 입력받아 pose 1개를 출력하는 partial-to-partial point matching 구조다. 멀티객체는 PEM 내부에서 joint로 처리되지 않으며, ISM이 N개 mask를 내보내면 PEM을 **per-mask로 반복 호출**하는 것이 정석이다. → 즉 멀티객체화는 **SAM-6D 자체 수정이 아니라 wrapper 기반 orchestration**이며, 이미 작성된 `yolo_ism_object_n.py`가 올바른 접근이다. (SAM-6D, Lin et al., CVPR 2024, [arXiv:2311.15707](https://arxiv.org/abs/2311.15707))

2. **SAM-6D는 per-frame, untracked, jittery 검출기다.** 시간적 일관성·트래킹·멀티뷰 융합 기능이 원천적으로 없다. 따라서 "SLAM context 결합"은 SAM-6D가 **갖지 못한 축**을 채우는 것이지, 중복 개선이 아니다.

3. **Object-pose-as-landmark factor graph 계보는 명확히 존재한다.** SLAM++(CVPR 2013), Fusion++(3DV 2018), QuadricSLAM(RA-L 2019), CubeSLAM(T-RO 2019), NodeSLAM(3DV 2020)이 camera node + object landmark node + observation edge 구조를 g2o/GTSAM으로 최적화한다. 즉 **object landmark graph는 검증된 패러다임**이며, 새로 발명할 필요가 없다.

4. **2024–25 기준 미점유 공간**: SAM류 foundation segmentation + zero-shot CAD 기반 6D pose 추정기(SAM-6D)를 **persistent factor-graph object map**에 연결하고, multi-view로 pose를 융합하는 "open-set foundation-model → factor-graph object-SLAM" 통합은 거의 비어 있다. → **시스템 통합(engineering) 논문**으로 방어 가능한 틈이다.

5. **필터링 단독은 incremental.** One Euro filter(CHI 2012), SE(3) EKF는 확립된 공학이며 그 자체로는 contribution이 약하다. **object-memory-conditioned / multi-view-fused stabilization**으로 묶을 때만 방어 가능하다.

---

## 1. Executive Summary

### 1.1 전체 구현 가능성 판단

전체 구상(멀티객체 wrapper → object memory → jitter reduction → landmark graph)을 **1~2개월에 모두 완성하는 것은 비현실적**이다. 그러나 **그것이 문제가 되지 않도록 설계**하는 것이 본 보고서의 핵심이다. 4개 기능은 서로 느슨하게 결합되어 있어, 각각이 독립적 stop-point를 형성한다.

핵심 판단:

- **1순위(MVP)는 Multi-object SAM-6D Wrapper가 맞다.** 단, 이것만으로는 논문 contribution이 약하다(엔지니어링 통합). 반드시 그 위에 **정량 평가(멀티객체 검출/세그/포즈 정확도 + runtime)**를 얹어야 한다.
- **진짜 논문적 핵심(권장 완성선)은 SLAM 기반 Object Memory + Multi-view Pose Fusion**이다. 이것이 SAM-6D가 못 하는 일(시간·공간 일관성)을 채우며, 위 4번의 미점유 공간을 직접 겨냥한다.
- **Pose Jitter Reduction은 독립 기능이 아니라 Object Memory의 부산물/평가 축으로 흡수**해야 한다. 별도 1순위로 두면 incremental 비판을 받는다.
- **Object Landmark Graph는 schema + 저장 + 시각화 + post-hoc 분석 수준으로 제한**하면 충분히 후순위 contribution이 되며, full optimization은 future work로 안전하게 미룰 수 있다.

### 1.2 가장 추천하는 핵심 Contribution

> **"YOLO-World+MobileSAM(ISM) 기반 zero-shot 멀티객체 segmentation과 SAM-6D(PEM) 6D pose 추정기를, ORB-SLAM3/RTAB-Map의 카메라 궤적을 이용해 world 좌표계의 persistent multi-object map으로 융합하는 ROS2 통합 시스템. 멀티뷰 관측을 객체별 memory로 association·누적하여, 단일 프레임 SAM-6D 대비 pose jitter와 false re-detection을 줄이고 객체 재관측 시 일관된 ID를 유지한다."**

이것은 **알고리즘 발명이 아니라 통합·실증** 기반 contribution이며, 석사 논문/기술 보고서에서 가장 방어하기 안전하다.

### 1.3 시간 부족 시 권장 Stop-point

- **최악(2~3주만 가능)**: Phase A까지 — 멀티객체 wrapper + 정량 벤치. "config-driven open-set 멀티객체 6D pose ROS2 파이프라인 + 벤치마크" 주장.
- **권장(4~6주)**: Phase B까지 — 위 + SLAM pose 기반 object memory/re-ID + multi-view pose 안정화. **여기서 멈추는 것을 목표로 설계하라.**
- **여유(7~8주+)**: Phase C — object landmark graph schema/저장/시각화/post-hoc consistency 분석.
- **후속 연구**: Phase D — full factor-graph joint optimization, loop-closure에 object landmark 활용.

---

## 2. Priority Roadmap

| Priority | Feature | Goal | Minimum Output | 논문 Contribution | 예상 난이도 | Stop 가능 여부 |
|---|---|---|---|---|---|---|
| **1 (MVP)** | Multi-object SAM-6D Wrapper | 최대 5객체 per-mask orchestration, ≤1s/frame 지향 | N객체 6D pose JSON + 시각화 + runtime/정확도 표 | (약) open-set 멀티객체 zero-shot 6D pose **ROS2 통합·벤치마크** | 낮음~중 | ✅ 단독 stop 가능 (보고서급) |
| **2 (권장 완성선)** | SLAM pose 기반 Object Memory / Re-ID + Multi-view Fusion | world 좌표 object map, 재관측 association, 멀티뷰 pose 융합 | object map(JSON/DB), association 정확도, jitter↓·ID유지 지표 | (강) SAM-6D를 **persistent multi-object map으로 만드는** 통합 contribution | 중 | ✅ **여기서 멈추는 것을 권장** |
| **3 (흡수형 확장)** | Pose Jitter Reduction | memory/멀티뷰 조건부 SE(3) 안정화 | jitter(static std/velocity) vs ADD trade-off 곡선 | (보강) memory-conditioned stabilization, naive smoothing 함정 실측 | 낮음~중 | ✅ Phase B에 흡수 |
| **4 (후순위/최종)** | Object Landmark Graph | keyframe+object node, observation edge **schema/저장/시각화/post-hoc** | graph 파일(g2o/GTSAM 호환 schema), 시각화, drift 분석 | (확장) factor-graph object-SLAM의 토대 + future-work 명시 | 중~높음(full opt 시 높음) | ✅ schema 수준에서 stop 가능 |

> **순서 재검토 결론**: 사용자가 제시한 1→2→3→4 순서는 **대체로 타당하나 1곳 수정**한다. **Pose Jitter Reduction(3)을 독립 단계가 아니라 Object Memory(2)의 내부 구성요소/평가 축으로 강등**하라. 이유: (a) jitter 감소는 멀티뷰 융합·memory의 자연스러운 결과이므로 별도 단계로 분리하면 중복·incremental 비판을 받고, (b) memory 없이 단일 객체에 필터만 거는 것은 논문적으로 가장 약한 기여다. 따라서 실효 순서는 **1: Wrapper → 2: Memory(+jitter 흡수) → 3: Landmark Graph(schema)**의 3단계로 보는 것이 정확하다.

---

## 3. Stop-point Based Development Plan

### Phase A — 1개월 최소 MVP (실제로는 2~3주 목표)

**구현 범위**
- `yolo_ism_object_n.py`를 정식 멀티객체 orchestrator로 확립: config(`yolo_ism_objects.yaml`)에 정의된 N객체에 대해 YOLO-World+MobileSAM이 bbox/mask proposal 생성 → 각 proposal에 대해 PEM을 per-mask 호출 → N개 6D pose 출력.
- ISM→PEM 브릿지(이미 보유: RGB/aligned depth/cam intrinsic/RLE seg → run_pem.sh)를 멀티객체 루프로 일반화.
- 객체별 confidence/score 기록, top-k cap, class-gating(기존에 발견한 색-속성 프롬프트 해법 반영).
- ROS2 노드화(최소): bag/카메라 입력 → `/objects/poses`(PoseArray + class id) 퍼블리시.

**산출물**
- 멀티객체 6D pose 결과(프레임별 JSON: class, mask(RLE), pose(R|t), score).
- 시각화(이미지 위 mask + 3D bbox/axis overlay).
- **벤치마크 표**: 객체 수 vs runtime(초/frame), per-class 검출률, (가능하면 pseudo-GT 기반) pose sanity.

**평가 지표**
- 멀티객체 **detection/segmentation**: precision/recall, mask IoU(pseudo-GT 대비), per-class.
- **6D pose**: ADD/ADD-S, AUC (CAD GT 또는 single-view 일관성 proxy). GT가 없으면 멀티뷰 reprojection consistency로 proxy.
- **Runtime**: 객체 1~5개에 대한 frame당 시간, ≤1s/frame 목표 달성 여부.

**이 단계에서 멈췄을 때 가능한 논문 주장 (안전)**
- "Config-driven, open-set 멀티객체(최대 5) zero-shot 6D pose ROS2 파이프라인을 구축하고, 객체 수에 따른 runtime/정확도 trade-off를 정량화했다."
- **과장 금지**: "새로운 6D pose estimator를 제안했다"(아님 — wrapper다), "실시간"(미달 시 금지).

---

### Phase B — 1~2개월 권장 완성선 (이 단계 도달을 목표로 설계)

**구현 범위**
- **SLAM pose 수신**: ORB-SLAM3/RTAB-Map의 카메라 pose(`T_world_camera`)를 ROS2 TF/topic으로 구독.
- **World 좌표 변환**: 각 프레임 object pose `T_camera_object`를 `T_world_object = T_world_camera · T_camera_object`로 변환해 map 좌표계에 저장.
- **Object Memory(association)**: 재관측 시 기존 memory와 매칭. association 기준을 **계층적 게이트**로 설계:
  1. class label gate (필수)
  2. 3D centroid proximity(world) — 1차 핵심
  3. extent/bbox 크기 일관성
  4. (선택) appearance feature(잘라낸 patch의 임베딩)
  5. pose/depth consistency(관측 누적 시 outlier 제거)
- **Multi-view pose fusion**: 같은 object id에 누적된 관측을 SE(3) 평균(translation 가중평균 + rotation geodesic/Karcher mean, confidence 가중)으로 융합 → **jitter 자동 감소**(Phase 3 흡수).
- 재관측 시 일관된 **persistent object id** 유지.

**산출물**
- **Object map**(JSON/SQLite): `{obj_id, class, T_world_object, extent, n_observations, confidence, first/last_seen_keyframe, fused_pose}`.
- 로봇 이동 → 재관측 시나리오에서 association 로그(매칭/신규/병합 이벤트).
- 단일 프레임 SAM-6D vs memory-fused pose의 **jitter 비교 그래프**.

**평가 지표**
- **Re-identification accuracy**: 재관측 객체의 올바른 association율, **ID switch 수**(MOTA류), false merge/false new 비율.
- **Tracking continuity**: 객체 관측 trajectory의 끊김·복구.
- **Pose jitter**: static 구간 pose std, pose velocity/acceleration 크기 (single-frame vs fused).
- **6D pose accuracy 유지**: ADD-AUC가 융합으로 **악화되지 않음**(또는 개선) 입증 — naive smoothing 함정 대비.
- **Map consistency**: 동일 객체의 멀티뷰 world pose 분산.

**이 단계에서 멈췄을 때 가능한 논문 주장 (강·안전)**
- "SAM-6D의 per-frame 멀티객체 pose를 SLAM 궤적으로 world 좌표 persistent object map에 융합하고, class+geometry 기반 association으로 재관측 시 일관된 객체 id와 안정화된 pose를 유지하는 ROS2 시스템을 제안·실증했다."
- "Multi-view fusion이 단일 프레임 대비 pose jitter를 정량적으로 줄이면서 pose accuracy를 유지함을 보였다."
- **과장 금지**: "object-level SLAM을 풀었다"(full optimization 없음), "loop closure에 기여"(미구현 시 금지).

---

### Phase C — 시간이 남을 때 추가 구현

**구현 범위**
- **Object Landmark Graph schema** 구축(최적화는 하지 않음): keyframe node, object landmark node, observation edge(camera↔object 상대 pose) 자료구조 정의 및 저장.
- g2o/GTSAM **호환 포맷으로 export**(예: `.g2o` 또는 GTSAM `NonlinearFactorGraph` 직렬화)하되, 실제 solve는 post-hoc 오프라인 분석으로만.
- **시각화**: RViz/Open3D에서 카메라 궤적 + object landmark + observation edge 표시.
- **Post-hoc consistency 분석**: 관측 잔차(residual) 분포, drift 추정, association 품질 진단.
- (선택) 명시적 **adaptive jitter filter**(One Euro / SE(3) EKF)를 memory pose 위에 얹어 motion-adaptive 안정화 비교.

**산출물**
- graph 파일(노드/엣지 + 공분산 placeholder), 시각화 캡처, residual/consistency 리포트.
- One Euro vs naive EMA vs fused-mean의 jitter–lag–ADD trade-off 곡선(있으면).

**평가 지표**
- **Graph consistency**: observation residual 통계, 멀티뷰 pose 분산 감소.
- **Ablation**: (no memory) vs (memory) vs (memory+graph schema) — jitter, ID switch, ADD.
- Filter ablation: naive smoothing의 lag/accuracy 악화 vs adaptive 개선 실측.

**Contribution 강화 요소**
- "Object landmark를 factor-graph로 표현할 수 있는 schema와 멀티뷰 일관성 분석을 제공"하여, full optimization으로 가는 **명확한 토대**를 제시. → Phase B 주장에 "확장 가능성 실증"을 더한다.

---

### Phase D — 최종 확장 또는 후속 연구 (지금 구현 안 함)

**구현 범위(미래)**
- Full **factor-graph joint optimization**(GTSAM/iSAM2): camera pose + object landmark를 함께 최적화, object observation을 SLAM back-end에 진짜 factor로 주입.
- **Object-level loop closure / relocalization**: object landmark로 장소 재인식.
- Symmetric object 처리(ADD-S 대칭성 factor), uncertainty/covariance propagation(학습 pose의 공분산을 graph에 전파).
- 동적 객체 처리.

**산출물(미래)**
- 최적화된 global object map, drift 감소 정량화, loop-closure 성공률.

**지금 구현하지 않아도 되는 이유**
- (a) full optimization은 검증·튜닝 비용이 크고 1~2개월에 안정적 결과를 내기 어렵다. (b) Phase B의 contribution(통합·융합·재실증)은 **D 없이도 독립적으로 완결**된다. (c) 학습 기반 pose의 공분산 추정이 비자명해 factor weight 설계가 추가 연구 주제다.

**Future work 표현 방식**
- "본 시스템은 object landmark를 factor-graph schema로 export하며, 이를 GTSAM/iSAM2 기반 joint optimization과 object-level loop closure로 확장하는 것은 향후 연구로 남긴다." — 안전하고 자연스러운 마무리.

---

## 4. Technical Feasibility Analysis (기능별)

### 4.1 Multi-object SAM-6D Wrapper
- **구현 난이도**: 낮음~중. PEM per-mask 루프 + 결과 집계가 핵심이며, 이미 단일객체 브릿지와 `yolo_ism_object_n.py` 보유.
- **필요 입력**: RGB, aligned depth, camera intrinsic, N객체 CAD(mm 단위 — BOP/SAM-6D 가정, meter면 ×1000 보정 필수), config의 class prompt.
- **필요 출력**: 프레임별 N개 {class, mask, T_camera_object, score}.
- **병목**: PEM이 객체당 ~1.5s(RTX3090) → 5객체면 frame당 수 초. ≤1s/frame 목표는 객체 수↓·배치화·해상도/template 수 조정 없이는 어려움.
- **실패 가능성**: 멀티클래스 cross-talk(이미 색-속성 프롬프트로 완화), proposal-miss(기존 audit에서 FN의 68%), template 불일치.
- **Fallback**: 객체 수 제한(3개), 순차 처리 허용(1s/frame 목표를 "near-interactive"로 완화), top-k cap.
- **3개월 내 가능성**: 매우 높음(거의 완료 상태). **1개월 내 가능성**: 높음.

### 4.2 SLAM 기반 Object Memory / Re-ID + Multi-view Fusion
- **구현 난이도**: 중. SLAM TF 동기화 + 좌표 변환 + association 로직 + SE(3) 융합.
- **필요 입력**: `T_world_camera`(ORB-SLAM3/RTAB-Map), 프레임별 object pose, 타임스탬프 동기화.
- **필요 출력**: persistent object map, 재관측 association, fused pose.
- **병목**: (a) **시간 동기화**(SAM-6D 느린 추론 vs SLAM 빠른 pose) — keyframe 시점 정렬 필요. (b) association 임계값 튜닝. (c) rotation averaging의 대칭 객체 처리.
- **실패 가능성**: SLAM drift가 association을 오염, depth 노이즈, 적은 재관측.
- **Fallback**: keyframe-only 처리(매 프레임 대신), centroid+class만으로 association(appearance 생략), 정적 씬·소수 객체로 scope 축소.
- **3개월 내 가능성**: 높음. **1개월 내 가능성**: 중(Phase A가 빨리 끝나면 가능).

### 4.3 Pose Jitter Reduction (흡수형)
- **구현 난이도**: 낮음~중. One Euro/SE(3) EKF/confidence-weighted mean 중 택1.
- **필요 입력**: 시계열 object pose + confidence + (선택) motion 추정.
- **필요 출력**: 안정화된 pose, jitter 지표.
- **병목**: naive low-pass는 fast motion 시 **lag↑·accuracy↓**(핵심 함정). adaptive(One Euro)/confidence-gating 필요.
- **실패 가능성**: 잘못된 필터가 정확도를 악화시켜 "개선 없음"으로 귀결.
- **Fallback**: multi-view fused-mean만으로도 jitter 감소 효과를 보고(별도 필터 불필요).
- **3개월 내 가능성**: 매우 높음. **1개월 내 가능성**: 높음(단독으로는 약함).

### 4.4 Object Landmark Graph
- **구현 난이도**: schema/시각화/post-hoc는 중, **full optimization은 높음**.
- **필요 입력**: keyframe pose, object observation(상대 pose), association 결과.
- **필요 출력**: graph 파일(node/edge), 시각화, residual 분석.
- **병목**: full opt 시 학습 pose의 공분산/factor weight 설계, 수렴 안정성, 대칭 객체.
- **실패 가능성**: full opt가 발산/악화. schema 수준은 실패 위험 낮음.
- **Fallback**: optimization 없이 schema+export+시각화+오프라인 분석으로 제한(권장).
- **3개월 내 가능성**: schema 높음 / full opt 낮음~중. **1개월 내 가능성**: schema만 중, full opt 낮음.

---

## 5. Recommended System Architecture

### 5.1 전체 Pipeline

```
RGB-D camera / ros2 bag
        │
        ├─────────────► SLAM (ORB-SLAM3 / RTAB-Map)
        │                     │  T_world_camera (TF), keyframes
        ▼                     │
  ISM (YOLO-World + MobileSAM)│
   N masks/bboxes per frame   │
        │                     │
        ▼                     │
  PEM (SAM-6D), per-mask loop │
   N × T_camera_object        │
        │                     │
        ▼                     ▼
   ┌──────────────────────────────────┐
   │  Object Memory / Fusion Node      │
   │  · T_world_object = Twc · Tco     │
   │  · association (class+geom+app)   │
   │  · multi-view SE(3) fusion        │
   │  · persistent object map + id     │
   └──────────────────────────────────┘
        │                     │
        ▼                     ▼
  /objects/map (markers)   Object Landmark Graph
  RViz 시각화               (schema/export/post-hoc, Phase C)
```

### 5.2 ROS2 Node Graph (권장)

| Node | 구독(Subscribe) | 발행/제공(Publish/Service/Action) |
|---|---|---|
| `slam_node` (기존 ORB-SLAM3/RTAB-Map) | `/camera/rgb`, `/camera/depth` | TF `world→camera`, `/slam/keyframes` |
| `ism_node` | `/camera/rgb`, `/camera/depth` | `/ism/proposals`(masks RLE + bbox + class + score) |
| `pem_node` (멀티객체 wrapper) | `/ism/proposals`, `/camera/depth`, `/camera/info` | `/objects/poses_cam`(PoseArray + class + score) |
| `object_memory_node` | `/objects/poses_cam`, TF `world→camera` | `/objects/map`(MarkerArray), `/objects/map_state`(custom msg), service `query_object`, service `reset_map` |
| `graph_node` (Phase C) | `/objects/map_state`, `/slam/keyframes` | graph 파일 export, `/objects/graph_viz`(MarkerArray) |

설계 권장:
- **무거운 추론(ISM/PEM)은 service/action으로 분리** 고려(매 프레임 강제 X). 실시간이 아니므로 **keyframe-triggered** 또는 rate-limited 처리.
- **시간 동기화**: `message_filters`(ApproximateTime) 또는 keyframe 타임스탬프 기준 정렬. SAM-6D 지연을 감안해 **pose는 관측 시점 TF로 역산**.
- 좌표·단위 일관성: CAD는 mm, ROS는 m → 변환 지점 명시(검출0 사고 방지).

### 5.3 Data Flow 요약
1. SLAM이 `T_world_camera`를 지속 생성.
2. (keyframe 또는 rate-limit 시) ISM이 N proposal → PEM이 N개 `T_camera_object`.
3. memory node가 world로 변환, 기존 객체와 association, 멀티뷰 융합, id 부여.
4. map 발행·시각화. (Phase C) graph로 export·분석.

### 5.4 Object Memory Schema (제안)

```jsonc
{
  "object_id": 7,                       // persistent id
  "class": "milk_carton",
  "T_world_object": [[r11..r33,t],[0001]], // 4x4, fused
  "extent": [w, d, h],                  // bbox/CAD extent (m)
  "cad_ref": "Milk_color_uv_...mm.ply",
  "confidence": 0.82,                   // aggregated
  "n_observations": 14,
  "observations": [                     // raw 관측 누적(융합 입력)
    {
      "keyframe_id": 102,
      "stamp": 1719300000.12,
      "T_world_object_obs": [...],      // 단일 관측 world pose
      "T_camera_object": [...],         // raw PEM 출력
      "score": 0.79,
      "mask_rle": "...",
      "appearance_feat": [/* optional embedding */]
    }
  ],
  "first_seen_kf": 88,
  "last_seen_kf": 140,
  "symmetry": "none|z-axis|..."         // ADD-S 대칭 처리용
}
```

### 5.5 Object Landmark Graph Schema (Phase C, optimization 없이)

```jsonc
{
  "nodes": {
    "keyframes": [ {"kf_id":102, "T_world_camera":[...], "stamp":...} ],
    "objects":   [ {"obj_id":7, "class":"milk_carton",
                    "T_world_object":[...], "extent":[...]} ]
  },
  "edges": [
    {                                  // observation / pose constraint edge
      "type": "camera_object_observation",
      "kf_id": 102, "obj_id": 7,
      "T_camera_object": [...],        // 측정값 z
      "information": [[6x6]],          // 공분산^-1 (placeholder: score 기반)
      "residual": null                 // post-hoc solve 시 채움
    }
  ],
  "export": { "g2o": "object_map.g2o", "gtsam": "object_map.fg" }
}
```
> g2o/GTSAM가 바로 읽을 수 있는 포맷으로 export하되, **solve는 오프라인 분석**으로만 수행(Phase D 전까지 back-end에 주입하지 않음).

---

## 6. Evaluation Plan (단계별)

| 축 | 지표 | 적용 Phase | GT/Proxy |
|---|---|---|---|
| **Multi-object detection/seg** | precision, recall, mask IoU, per-class | A | pseudo-GT(기존 9-bag audit 방식) |
| **6D pose accuracy** | ADD, ADD-S, AUC | A,B | CAD GT 또는 멀티뷰 reprojection consistency proxy |
| **Runtime** | 초/frame (객체 1~5개), node별 latency | A,B | 실측 |
| **Object re-ID accuracy** | 올바른 association율, ID switch 수, false merge/new 비율 | B | 시나리오 라벨(이동 후 재관측) |
| **Tracking continuity** | 관측 trajectory 끊김/복구, 평균 track 길이 | B | 시나리오 |
| **Pose jitter** | static 구간 pose std, pose velocity/accel 크기 | B,C | single-frame vs fused 비교 |
| **Graph consistency** | observation residual 분포, 멀티뷰 world pose 분산 | C | post-hoc |
| **Ablation** | (single-frame) vs (memory) vs (memory+filter) vs (graph) | B,C | 위 지표 교차 |

**핵심 실험 메시지**: "멀티뷰 융합/memory가 jitter를 줄이면서 ADD를 악화시키지 않는다"(naive smoothing 함정 대비)와 "재관측 시 ID 일관성 유지"가 가장 강한 정량 근거다.

---

## 7. Thesis Contribution Framing

### 버전 1 — Phase A까지만 구현

- **논문/보고서 제목 후보**: *"Config-Driven Open-Set Multi-Object Zero-Shot 6D Pose Estimation in ROS2: A SAM-6D Orchestration Pipeline and Benchmark"*
- **Contribution bullet**:
  - YOLO-World+MobileSAM 기반 open-set 멀티객체 proposal과 SAM-6D PEM의 per-mask orchestration을 ROS2로 통합.
  - 객체 수에 따른 runtime/정확도 trade-off 정량화.
- **Abstract 문장(예)**: "We present a config-driven ROS2 pipeline that orchestrates SAM-6D over multiple open-set object proposals and benchmark its accuracy–runtime trade-off for up to five objects."
- **과장 금지**: "new 6D estimator", "real-time"(미달 시), "SLAM 기여".
- **안전한 주장**: "통합·실증·벤치마크".

### 버전 2 — Phase B까지 구현 (권장 목표)

- **논문 제목 후보**: *"SLAM-Aware Persistent Multi-Object 6D Mapping: Fusing Zero-Shot SAM-6D Poses with Robot Trajectory for Stable Object Memory"*
- **Contribution bullet**:
  - SAM-6D의 per-frame 멀티객체 pose를 SLAM 궤적으로 world 좌표 **persistent object map**에 융합.
  - class+geometry 기반 **re-identification/association**으로 재관측 시 일관된 객체 id 유지.
  - **multi-view SE(3) fusion**이 단일 프레임 대비 pose jitter를 줄이면서 정확도를 유지함을 정량 입증.
- **Abstract 문장(예)**: "By transforming per-frame SAM-6D poses into a global frame via the SLAM trajectory and fusing multi-view observations with class- and geometry-based association, our system maintains a persistent, jitter-reduced multi-object map with consistent re-identification across revisits."
- **과장 금지**: "object-level SLAM 해결"(full opt 없음), "loop closure 개선".
- **안전한 주장**: "persistent mapping, re-ID, multi-view stabilization 실증".

### 버전 3 — Phase C/D까지 구현

- **논문 제목 후보**: *"Toward Factor-Graph Object-Level SLAM with Foundation-Model 6D Poses: A SAM-6D-to-GTSAM Object Landmark Framework"*
- **Contribution bullet**: 위 + object landmark **graph schema/export/post-hoc consistency 분석**, adaptive(One Euro/SE(3)EKF) vs naive smoothing의 jitter–lag–accuracy trade-off 실측, factor-graph 확장 토대.
- **Abstract 문장(예)**: "We further structure the fused object map as a camera–object factor-graph and analyze its multi-view consistency, providing a foundation for full object-level pose-graph optimization."
- **과장 금지(D 미완 시)**: "joint optimization으로 drift를 줄였다", "object loop closure".
- **안전한 주장**: "graph schema와 consistency 분석 제공, full optimization은 future work".

---

## 8. Final Recommendation

### 8.1 지금 당장 구현해야 할 1순위
**Multi-object SAM-6D Wrapper(Phase A)를 정식화하고 벤치마크 표까지 완성.** 이미 자산(`yolo_ism_object_n.py`, ISM→PEM 브릿지, CAD mm 보정, 9-bag audit 방법론)이 있으므로 2~3주 내 닫고, **즉시 Phase B(Object Memory + Multi-view Fusion)로 진입**하라. Phase B가 논문의 진짜 핵심이다.

### 8.2 중간에 멈춰도 되는 기준
- **Phase A 완료 + 벤치마크 표 확보** → 보고서/워크숍급 방어 가능(최소선).
- **Phase B 완료 + re-ID/jitter 정량 결과** → **본 연구의 권장 stop-point.** 여기서 멈춰도 독립적으로 완결된 contribution.
- 이후는 시간 여유에 따른 보강일 뿐, 없어도 연구가 무너지지 않음.

### 8.3 뒤로 미뤄야 할 기능
- **Pose Jitter Reduction의 독립 구현**(→ Phase B multi-view fusion에 흡수, 여유 시 adaptive filter ablation으로만).
- **Object Landmark Graph full optimization**(→ Phase D, future work).
- **PEM 내부 알고리즘 수정**(범위 외, 제외 확정).
- appearance feature 기반 association의 고도화(centroid+class로 먼저, 여유 시 추가).

### 8.4 다음 `/bmad-architect` 또는 `/bmad-prd`로 넘길 요구사항
1. **`pem_node` 멀티객체 wrapper**: per-mask PEM 호출, rate-limit/keyframe-trigger, ≤1s/frame 목표(객체≤5), 출력 `/objects/poses_cam`(PoseArray+class+score).
2. **`object_memory_node`**: TF 동기화 → world 변환 → 계층적 association(class→centroid→extent→app) → SE(3) multi-view fusion → persistent id, 출력 `/objects/map` + `/objects/map_state`, service `query_object`/`reset_map`.
3. **Object map schema**(§5.4)와 **graph schema/export**(§5.5) 자료구조 확정.
4. **평가 하네스**: §6 지표 자동 산출(re-ID, ID switch, jitter std/velocity, ADD-AUC, ablation 토글).
5. **시간 동기화 정책**(keyframe-triggered vs ApproximateTime) 결정 — 아키텍처 단계 핵심 의사결정.
6. **비기능 요구**: CAD mm/ROS m 단위 변환 단일 지점, 정적 씬·소수 객체(≤5) scope, ORB-SLAM3 우선(RTAB-Map fallback).

---

## 9. Research Questions 직접 답변

1. **반드시 먼저 구현해야 할 기능?** → Multi-object Wrapper(Phase A). 나머지의 입력을 모두 공급하는 전제이며 가장 빠르게 닫힌다.
2. **1개월만 가능하면 어디까지?** → Phase A 완료 + Phase B 일부(world 변환·기본 association·centroid 기반 fusion). 최소 방어선은 "멀티객체 통합+벤치", 가능하면 "world object map 초안"까지.
3. **1~2개월이면 어디까지?** → **Phase B 완전 완료**(re-ID + multi-view fusion + jitter/ID 정량). contribution이 충분히 강해지는 지점.
4. **과감히 포기해도 되는 기능?** → Landmark graph full optimization, 독립 jitter filter 모듈, appearance-heavy association, PEM 수정. 모두 포기해도 Phase B 주장은 유지된다.
5. **각 단계 contribution 표현?** → §7의 3개 버전(과장 금지/안전 주장 포함) 그대로 사용.
6. **wrapper 없이 memory부터 가능한가?** → 기술적으로는 단일 객체 memory가 가능하나, **논문적으로 약하다**(멀티객체성이 SAM-6D 통합의 핵심 차별점이고, 단일 객체 memory+filter는 가장 흔한 incremental 패턴). wrapper를 먼저 두는 것이 옳다.
7. **graph를 schema/viz/post-analysis로 제한해도 contribution?** → **된다.** factor-graph 표현 가능성·멀티뷰 consistency 분석 제공만으로 "확장 토대"라는 정직한 기여가 성립하며, full opt는 future work로 안전하게 분리된다.
8. **기존 연구 대비 차별성?** → 있다. SLAM++/Fusion++ 등은 **closed-set CAD-instance**, semantic SLAM(Kimera/Hydra/SuMa++)은 대개 **centroid/quadric 수준(정밀 6D 아님)**. **open-set foundation segmentation(SAM/YOLO-World) + zero-shot CAD 6D pose(SAM-6D)를 SLAM 궤적으로 persistent object map에 융합**하는 통합은 2024–25 기준 미점유. 단 **알고리즘 발명이 아닌 시스템 통합·실증**으로 포지셔닝해야 안전하다.
9. **ROS2 구조?** → §5.2 노드 그래프. 무거운 추론은 service/action·keyframe-trigger, memory는 topic+service, 동기화는 TF/타임스탬프 기준.
10. **단계별 acceptance criteria·평가지표?** → §6 표 + 각 Phase의 평가 지표. 핵심 acceptance: A=벤치 표 산출, B=re-ID ID-switch↓ & jitter↓ & ADD 유지, C=graph export+consistency 분석.

---

## 10. 참고문헌 (검증 출처)

- SAM-6D: Lin et al., *Segment Anything Model Meets Zero-Shot 6D Object Pose Estimation*, CVPR 2024. [arXiv:2311.15707](https://arxiv.org/abs/2311.15707) · [code](https://github.com/JiehongLin/SAM-6D)
- SLAM++: Salas-Moreno et al., CVPR 2013.
- Fusion++: McCormac et al., 3DV 2018. [arXiv:1808.08378](https://arxiv.org/abs/1808.08378)
- QuadricSLAM: Nicholson, Milford, Sünderhauf, RA-L/ICRA 2019. [arXiv:1804.04011](https://arxiv.org/abs/1804.04011)
- CubeSLAM: Yang & Scaramuzza, T-RO 2019.
- NodeSLAM: Sucar et al., 3DV 2020.
- Kimera: Rosinol et al., ICRA 2020. [arXiv:2101.06894](https://arxiv.org/pdf/2101.06894)
- Hydra: Hughes et al., RSS 2022.
- SuMa++: Chen et al., IROS 2019. [arXiv:2105.11320](https://arxiv.org/abs/2105.11320)
- Probabilistic Data Association for Semantic SLAM: Bowman et al., ICRA 2017.
- FoundationPose: Wen et al., CVPR 2024 (Highlight). [arXiv:2312.08344](https://arxiv.org/abs/2312.08344)
- BundleSDF: Wen et al., CVPR 2023. · BundleTrack: Wen & Bekris, IROS 2021. [arXiv:2108.00516](https://arxiv.org/pdf/2108.00516)
- se(3)-TrackNet: Wen et al., IROS 2020. [arXiv:2007.13866](https://arxiv.org/pdf/2007.13866)
- One Euro filter: Casiez, Roussel, Vogel, CHI 2012. [PDF](https://gery.casiez.net/publications/CHI2012-casiez.pdf)
- SE(3) EKF: [arXiv:2003.12978](https://arxiv.org/abs/2003.12978) · Invariant Smoothing on Lie Groups: [arXiv:1803.02076](https://arxiv.org/pdf/1803.02076)
- 6D pose metrics survey: MDPI Applied Sciences 2025, [15(6):3284](https://www.mdpi.com/2076-3417/15/6/3284)
- GTSAM: [gtsam.org](https://gtsam.org/tutorials/intro.html)

---

## 11. 다음 액션 제안

본 리서치는 조사·설계만 수행했으며 코드는 수정하지 않았습니다. 다음 중 선택해 주세요:

1. **`/bmad-prd`로 진행** — Phase A+B를 요구사항 명세(PRD)로 구체화(§8.4의 6개 항목을 epic/story로 전개).
2. **`/bmad-architect`로 진행** — §5 아키텍처(노드 그래프·schema·시간 동기화 정책)를 정식 solution design으로 확정.
3. **이 보고서 보강** — 특정 섹션(예: 평가 하네스 상세, association 알고리즘 의사코드, runtime 예산 분해)을 더 깊게 파고들기.
4. **메모리에 연구 방향 저장** — 이 우선순위/stop-point 결정을 프로젝트 메모리에 기록해 다음 세션에서 이어가기.

**질문**: 권장 stop-point를 **Phase B(SLAM 기반 Object Memory + Multi-view Fusion)**로 설정하는 데 동의하시나요? 동의하시면 바로 `/bmad-prd`로 넘겨 Phase A+B 요구사항을 구체화하겠습니다. 혹시 시간 제약이 1개월로 더 빡빡하다면 Phase A에 집중하도록 로드맵을 재조정하겠습니다.
