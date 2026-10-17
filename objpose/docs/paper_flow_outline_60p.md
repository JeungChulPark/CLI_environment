# Paper Flow Outline (60-page edition) — objpose: Real-Time Object 6D Pose Mapping via SLAM–SAM-6D Fusion

**이 파일은 35장 본안(`paper_flow_outline.md`)의 확장판이다.** 본안은 그대로 두고, 발표 시간이 60장 규모일 때 쓰는 구성을 따로 관리한다.
확장 원칙: **한 장에 들어갈 내용을 늘어뜨리지 않는다.** 이미 독립된 근거·수치를 가진 내용만 쪼개서 한 장씩 준다.

| 파트 | 35장 본안 | 60장판 | 주로 늘어난 것 |
|---|---|---|---|
| 0. 개요 | 2 | **4** | 실행 형상, LiDAR 동기 실측 |
| 1. 사전 작업 | 4 | **7** | 대칭 근거, 특징 캐시, RT 결과·기각 방법 |
| 2. SLAM 백엔드 | 6 | **9** | 공통 프로토콜, kf_update, KISS 흔들림 분석 |
| 3. 개선된 SAM-6D | 7 | **10** | ISM 3단 분리, PEM 입력·추론·검증 분리, 속도 옵션 |
| 4. 융합 | 3 | **5** | 자료구조, 추정 처리 과정 |
| 5. 객체 메모리 | 5 | **6** | 회전 모드 융합 독립 |
| 6. 결과 | 5 | **8** | 라이브/재생 분리, 비교 2단 |
| 7. 공개 데이터셋 | 3 | **4** | 이식 절차 |
| 8. 운영·시각화 *(신설)* | 0 | **4** | 뷰어·런처·산출물·운영 |
| 9. 한계·향후 *(신설)* | 0 | **3** | 한계 2장 + 향후·기여 |
| **합계** | **35** | **60** | |

**주의:** Part 7(P50–P53)은 아직 실행하지 않은 절이라 4장 중 3장이 빈 페이지다. 실제 내용이 있는 장은 **57장**으로 보는 것이 정확하다.
**시간 배분:** 장당 1분이면 60분. 30~40분 발표라면 본안 35장을 본문으로 쓰고 나머지 25장을 백업으로 돌리는 편이 안전하다.

**상태 범례:** `확정` / `검토` / `초안` / `예정`(아직 근거 없음)

**최종 갱신:** 2026-09-19

---

## Part 0 — System Overview (P01–P04)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P01** | Hardware rig, dual RGB-D D455f, VLP-16, cart | 카트 위에 SLAM용 카메라와 인식용 카메라, VLP-16 라이다를 고정한 리그 구성과 각 센서의 역할을 보여 준다. | 확정 |
| **P02** | Deployment topology, SLAM host vs. inference workstation, TCP pose protocol, link modes (tunnel/direct/manual/local) | 무거운 추론은 워크스테이션 한 대에 두고 SLAM은 데이터가 있는 호스트에서 돌리며, 둘을 줄 단위 JSON TCP로 잇는 실행 형상을 설명한다. | 초안 |
| **P03** | Time synchronization, 9-pin exposure sync, timestamp domain, host clock offset, residual τ | 두 카메라의 노출 순간은 9핀 외부 동기로 맞추고, 그것만으로 맞지 않는 타임스탬프는 도메인 통일·시계차 차감·잔여 τ 보정의 3단계로 하나의 시간축에 올린다. | 검토 |
| **P04** | LiDAR time alignment, no hardware trigger, UDP clock offset, τ from calibration, measured values | VLP-16은 트리거 입력이 없어 하드웨어 동기가 성립하지 않으므로 스탬프 도메인 정리·시계차 실측·τ 추정으로만 정렬하며, 그 실측값(시계차 18.23 s, 카메라 τ −48 ms, 라이다 τ +4~8 ms)을 함께 보고한다. | 검토 |

---

## Part 1 — Offline Preparation (P05–P11)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P05** | CAD models, colored PLY (mm, object-centred), 8,192 surface points, bounding-box extents | 인식 대상 9종을 컬러 PLY로 확보하고 PEM 입력용 8,192점 샘플과 오버레이용 바운딩박스 치수를 뽑아 둔다. | 초안 |
| **P06** | Symmetry declaration, self-alignment error, manual conservative table | 물체 좌표계에서 각도가 원리적으로 정의되지 않는 회전축만 손으로 선언하며(Sikhye 0.63 %·Sauce 1.54 %·Mugcup 2.22 %), 자동 유도는 곰·공룡에 가짜 대칭을 만들어 기각했다. | 초안 |
| **P07** | Template rendering, 42 viewpoints, rgb/mask/xyz + pose per view | 각 CAD를 42개 시점에서 렌더해 뷰마다 컬러·마스크·좌표 3종과 포즈 행렬을 만든다. | 초안 |
| **P08** | Feature caches, DINOv2 cls/patch for ISM, HSV reference, PEM tensors (tp/tf/tc) | 렌더 결과를 한 번만 모델에 통과시켜 ISM용 의미·외형 특징과 HSV 기준값, PEM용 점·특징·색 텐서로 캐시해 실행 중에는 GPU에 상주시킨다. | 초안 |
| **P09** | Offline SLAM trajectories, real-time vs optimized (loop closure + GBA), no prior-map reuse | 두 카메라 세션을 오프라인 SLAM으로 돌려 실시간 궤적과 최적화 궤적을 남기고, 보정에는 최적화 궤적을 쓴다(사전 맵 재사용은 지도 변형을 상속해 기각). | 초안 |
| **P10** | Rig extrinsic decomposition, gravity from trajectory plane, planar SE(2) hand-eye (AX = XB), floor-plane height, joint τ | 관측 가능한 성분을 각각 가장 잘 보이는 데이터로 풀도록 X를 세 조각(중력·수평 오프셋과 요·높이)으로 나누고 τ를 함께 추정한다. | 초안 |
| **P11** | Calibration result and uncertainty, rejected alternatives (AX = ZB, shared-map localization) | 잔차 중앙 0.41 cm·부트스트랩 2 mm/0.05°의 결과와 함께, 지도 전체를 가로지르는 두 대안을 왜 버렸는지 객체 일관성 수치(0.85 vs 1.95/2.46 cm)로 보인다. | 검토 |

---

## Part 2 — SLAM Backends (P12–P20)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P12** | SLAM input data, RGB-D 640×480@30, depth-to-colour alignment, VLP-16 10 Hz, per-point time | 같은 주행을 RGB-D 카메라와 VLP-16이 동시에 녹화하며, 두 입력은 같은 시간축 위에서 서로 다른 백엔드에 병렬로 공급된다. | 초안 |
| **P13** | Streaming protocol, hello → start → pose → kf_update → end, lane abstraction | 모든 SLAM 백엔드는 줄 단위 JSON 5종만 말하면 되며, 새 SLAM을 붙이는 비용이 "얇은 브리지 하나"로 고정된다. | 초안 |
| **P14** | ORB-SLAM3 tracking, ROS-free session reader, depth alignment, real-time pacing with frame skip | 특징점 기반 RGB-D SLAM을 ROS 없이 직접 구동해 프레임마다 자세와 참조 키프레임을 내보내며, 밀리면 프레임을 건너뛰어 실시간성을 지킨다. | 초안 |
| **P15** | kf_update, full keyframe pose broadcast, periodic + post-loop (+0.5 s, +3 s), correction magnitude | 1초마다와 루프 클로저 직후 전체 키프레임 자세를 통째로 보내, SLAM이 과거를 고쳐 쓴 사실을 파이프라인 전체가 알 수 있게 한다(관측된 최대 보정 84 cm). | 초안 |
| **P16** | RTAB-Map, graph SLAM, frequent map corrections, real-time vs final pose gap | 같은 RGB-D 입력의 대조군으로, 지도 보정이 165회로 잦아 실시간 자세와 최종 자세의 차이가 크다는 특성을 보인다. | 초안 |
| **P17** | KISS-ICP, LiDAR odometry without ROS, no keyframes, lowest latency | venv에서 도는 최소 구성의 라이다 오도메트리로, 루프 클로저와 키프레임이 없는 대신 가장 매끄럽고 지연이 작다. | 초안 |
| **P18** | Deskew failure under acceleration, iterative deskew, delayed `pose_refine` | 궤적 스파이크의 원인이 등속 가정 deskew임을 가속 상관(0.63~0.70)으로 규명하고, 반복 deskew와 원시·평활 이중 전송으로 고주파 흔들림을 절반으로 줄였다. | 초안 |
| **P19** | hdl_graph_slam, ROS 2 graph SLAM, keyframes + loop closure on LiDAR | 라이다에서도 키프레임과 루프 클로저를 제공해 카메라 백엔드와 동일한 방식으로 지도 보정을 전달한다(최대 보정 2.7~2.85 cm). | 초안 |
| **P20** | Backend comparison (SLAM side), timing, arrival latency, loop closure, keyframe support | 네 백엔드를 센서·처리 시간·도착 지연·루프 클로저·키프레임 지원으로 정리하고, 어느 쌍이 동일 조건 비교인지 명시한다. | 초안 |

---

## Part 3 — Improved SAM-6D (P21–P30)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P21** | Input/output contract, per-frame independence | 이 블록은 RGB + 정렬 depth + K와 CAD·템플릿만 받아 프레임 독립적으로 카메라 기준 6D 자세를 돌려주며, 카메라 위치도 과거 프레임도 알지 못한다. | 초안 |
| **P22** | Original SAM-6D, SAM everything-mode, DINOv2 matching, PEM coarse+fine, why it cannot be used as-is | 원본 구조를 소개하고 실시간 속도·특정 물체 탐색·실내 유사 형상 오인식 세 가지에서 그대로 쓸 수 없었던 이유를 제시한다. | 초안 |
| **P23** | ISM ①, YOLO-World text prompts, single shared pass, per-object thresholds | 전 영역 분할 대신 텍스트 프롬프트 기반 검출기가 한 번의 패스로 모든 물체의 후보 박스를 내고, 물체별 임계값은 그 뒤에 적용한다. | 초안 |
| **P24** | ISM ②, DINOv2 semantic gate, MobileSAM box-prompted mask, masked appearance gate | 승자 박스를 의미 점수로 고른 뒤 그 박스만 분할하고, 마스크 안의 patch 특징으로 외형 게이트를 통과시킨다. | 초안 |
| **P25** | ISM ③, HSV hard gate, multi-label, relative assignment, cross-object NMS, 959-cell GT | 색 게이트와 후보 간 상대 비교를 넣어 사람이 판정한 959셀 기준으로 F1 0.676 → 0.809, 오검출 334 → 28건으로 줄였다. | 초안 |
| **P26** | PEM input construction, mask ∩ valid depth, radius inliers, silent-drop diagnostics | 자세 추정은 마스크 안의 유효 depth 점군으로만 가능하므로, 점이 부족해 버려지는 조건을 진단으로 남겨 실패를 관측 가능하게 만들었다. | 초안 |
| **P27** | PEM inference, coarse 6000→300, fine matching, template tensors, score | 템플릿 dense 특징과 관측 점군을 대응시켜 자세와 점수를 내는 원본 PEM 경로를 그대로 유지한다. | 초안 |
| **P28** | Stage V verification, mask IoU → texture → cluster occupancy, symmetry-aware | 기하 점수 argmax로 승자를 정하던 것을 마스크 IoU·텍스처·클러스터 수렴 순의 검증으로 바꾸고, 통과하지 못한 후보는 이유와 함께 버린다. | 초안 |
| **P29** | Speed options, adopted (fine_npoint 1024, mask cache) vs rejected (bf16, PEM budget) | 문헌 권장 설정을 그대로 쓰지 않고 이 배치에서 재측정해 절반을 기각했으며(bf16은 검출 채택률 17~22 %로 붕괴), 기각한 것도 재현 가능하게 남겼다. | 초안 |
| **P30** | Real-time architecture + parity, process split, shared-memory seqlock, newest-frame policy | 인식을 ROS 없는 별도 프로세스로 분리하고 최신 한 장만 주고받게 해 뒤처짐이 누적되지 않게 했으며, 판정이 오프라인 정본과 동일함을 먼저 확인했다. | 초안 |

---

## Part 4 — Fusion (P31–P35)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P31** | Fusion equation, coordinate chain, world-fixed static object | `T_w_obj = T_w_slam(s)·X·T_sam_obj`를 세우고, 필요한 것이 결국 "SAM 프레임 시각의 SLAM 자세" 하나임을 보인다. | 초안 |
| **P32** | PoseBuffer and KeyframeMap, time-ordered samples, map switch vs loop closure, cull redirect | 자세를 시각순 배열로 담아 이진 탐색으로 꺼내고, 키프레임 id → 최신 자세 사전으로 지도 보정을 반영하며, 컬링된 키프레임은 직전 스냅샷으로 재앵커한다. | 초안 |
| **P33** | SE(3) interpolation, Slerp + linear translation, max-gap 0.25 s, measured validity | 서로 다른 순간을 찍는 두 스트림을 잇기 위해 앞뒤 자세를 SE(3) 보간하고, 간격이 0.25초를 넘으면 포기한다(간격 ≤3 s에서 오차 중앙 0.5 cm). | 초안 |
| **P34** | Keyframe anchoring, `T_kf_obj`, correction propagation, loop-closure guard | 물체를 월드가 아니라 참조 키프레임 기준으로 저장해 지도 보정이 과거 전체에 전파되게 한다(루프 전후 위치 차 5.4 → 0.8 cm). | 초안 |
| **P35** | One estimate end-to-end, six steps, pose-wait for late poses, logging | 검출 한 건이 변환·조회·앵커·루프 근접 판정을 거쳐 관측으로 저장되는 과정을 따라가고, 자세가 늦게 오는 경우를 최대 2초 기다려 손실을 막는 장치를 설명한다. | 초안 |

---

## Part 5 — Object Memory (P36–P41)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P36** | Why memory, four per-frame limits, low update rate | 잡음·회전 이봉성·간헐 미검출·거짓 양성 때문에 지도는 "지금 본 것"이 아니라 "지금까지 본 것의 종합"이어야 한다. | 초안 |
| **P37** | Adapter, keyframe-anchored frame store, incremental feed, rebuild on kf_update | 검출 0건 프레임까지 보관했다가 키프레임 자세가 바뀌면 전체를 보정된 좌표로 다시 재생해 메모리를 새로 만든다(308프레임 75 ms). | 초안 |
| **P38** | Association, position-only gate 0.30 m, greedy 1:1, spawn-on-miss | 같은 클래스 안에서 위치 거리로만 짝짓고(회전 게이트는 180° 뒤집힘 때문에 끔), 매칭 실패는 거부가 아니라 새 인스턴스 생성으로 처리한다. | 초안 |
| **P39** | Pose fusion, running-mean translation with weight floor, rotation mode voting, symmetry alignment | 위치는 러닝 평균으로 누르고 회전은 평균 대신 표가 가장 많은 자세를 쓰며(지도 배회 −51 %, 유령 자세 11° → 1°), 대칭 선언된 물체는 섞기 전에 동치 자세로 맞춘다. | 초안 |
| **P40** | Bernoulli existence, expected P_D from visibility, lifecycle state machine | 각 물체에 존재확률을 두고 볼 수 있었는데 못 봤을 때만 깎아 tentative → active → lost → remembered → deleted의 수명을 관리한다. | 초안 |
| **P41** | Depth consistency check, free/occluded/surface, window confirmation, speed gate | 그 자리를 depth가 꿰뚫어 보는지로 "없어짐"을 직접 판정하되, 창 확인·속도 게이트·3프레임 확인으로 실물 오삭제를 막는다. | 초안 |

---

## Part 6 — Results (P42–P49)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P42** | Evaluation design, shared detections across lanes, metric definitions, same-condition rule | 같은 검출을 여러 레인에 먹이는 구조로 인식 오차를 공통 인자로 제거한 뒤 SLAM만 비교하며, 비교는 같은 데이터·같은 RT에서만 한다. | 초안 |
| **P43** | System in operation, viewer video, overlay + 3D map + status | 실제 화면에서 영상 박스와 3D 지도, 상태 카드가 함께 갱신되는 모습을 보여 파이프라인이 도는 것을 먼저 확인시킨다. | 초안 |
| **P44** | Live-sensor runs (2026-09-17), pose rate, latency, throughput | 노트북 센서를 직접 연결한 라이브 실행에서 포즈 25 Hz·도착 지연 65 ms·SAM-6D 0.9 Hz로 동작했고, 29분 정지 조건에서 관측 산포 0.4–1.3 cm를 유지했다. | 초안 |
| **P45** | Recorded-session replay, identical code path, 8 objects placed | 녹화본을 같은 경로로 재생해 91초 동안 8종을 모두 배치했고(포즈 2,719 전부 OK, 결과 나이 중앙 0.06 s), 재생과 라이브가 같은 로직임을 확인했다. | 초안 |
| **P46** | Per-input summary, condition × metric table | 정지·이동·장거리·실시간 등 입력 조건별로 포즈·처리율·객체 수·산포를 한 줄씩만 요약해 조건의 영향을 얕고 넓게 보인다. | 초안 |
| **P47** | Backend comparison ①, runtime facts from one multi-lane run | 한 실행 안에서 ORB-SLAM3와 hdl_graph_slam이 각각 포즈 2,731/888개, 도착 지연 107/23 ms, 키프레임 0/59로 동작한 런타임 차이를 보인다. | 초안 |
| **P48** | Backend comparison ②, object-level metrics, shared detections | 같은 검출을 쓴 두 레인의 관측 산포가 6.04 cm 대 1.52 cm로 갈리고 같은 물체의 위치가 중앙 7.7 cm 차이 난다는 것을, 인식 오차가 제거된 조건에서 보인다. | 초안 |
| **P49** | Interpretation and failure cases, keyframe effect, instance split, alignment degeneracy | 키프레임을 제공한 레인만 메모리가 8종 = 8개로 깨끗하게 남은 사실을 근거로 앵커링의 효과를 해석하고, 인스턴스 분열·정렬 퇴화처럼 믿으면 안 되는 사례를 명시한다. | 초안 |

---

## Part 7 — Public Dataset Evaluation (P50–P53) · **예정**

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P50** | Why public datasets, YCB-Video / BOP, single-camera simplification (X = I), role split | GT가 있는 데이터셋으로 절대 정확도를 재되, 카메라가 한 대라 리그 보정·시계 동기가 빠지고 인식 성능과 절대 오차만 남는다는 점을 먼저 명시한다. | 예정 |
| **P51** | Porting procedure, CAD → 42-view templates → caches, prompts, dataset reader adapter | 대상 객체의 자산을 새로 굽고 프롬프트와 리더 어댑터만 추가하면 나머지 파이프라인은 그대로 돈다. | 예정 |
| **P52** | Metrics, BOP AR (VSD/MSSD/MSPD), ADD(-S), map-level object error | 프레임 단위는 BOP 표준 지표로, 시스템 단위는 GT 카메라 자세로 환산한 지도 좌표계 객체 위치 오차로 측정한다. | 예정 |
| **P53** | System-level results, per-frame vs. fused-with-memory | 프레임 단위 추정과 융합·메모리를 거친 결과를 GT 기준으로 비교해 시간적 종합이 절대 오차를 얼마나 줄이는지 보인다. | 예정 |

---

## Part 8 — Operation & Visualization (P54–P57)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P54** | Viewer architecture, SSE payload, three.js map, per-lane panels | 브라우저는 SSE 한 줄기로 초기 정보와 프레임 payload를 받아 레인마다 영상 패널과 상태 카드를 만들고, 3D 지도 하나에 모든 레인을 색으로 겹쳐 그린다. | 초안 |
| **P55** | Launcher, one backend at a time, dataset tabs, run scripts | 브라우저에서 데이터셋과 백엔드를 고르면 런처가 기존 실행을 정리하고 새로 띄우며, 같은 조합을 명령행 스크립트로도 재현할 수 있다. | 초안 |
| **P56** | Artifacts and reproducibility, per-run jsonl + summary.json, post-hoc analysis | 실행마다 원시 로그 6종과 요약 1종을 남겨, 나중에 로그만으로 파이프라인을 mm 단위로 재현해 원인을 규명할 수 있게 했다. | 초안 |
| **P57** | Operational pitfalls, environment/lifecycle issues and their fixes | ssh와 conda 라이브러리 충돌, GIL로 인한 재생 붕괴, 낡은 공유메모리 부착, 죽은 카메라의 토픽 잔존처럼 실제로 시간을 잡아먹은 문제와 대응을 정리한다. | 초안 |

---

## Part 9 — Limitations & Future Work (P58–P60)

| # | 핵심 키워드 | 한 문장 요약 | 상태 |
|---|---|---|---|
| **P58** | Methodological limits, four assumptions (static objects, planar motion, fixed rig, per-frame recognition) | 정지 물체·평면 주행·고정 리그·프레임 독립 인식이라는 네 가정이 깨지면 현재의 정확도 보장이 사라진다는 점을 분명히 한다. | 초안 |
| **P59** | Implementation and scale limits, full rebuild cost, single GPU, single hub, latency budget | 메모리 전체 재구성이 프레임 수에 선형이고 GPU를 한 장만 쓰며 허브가 한 번에 하나만 도는 등, 현재 설계가 수 분·수백 프레임 규모에 맞춰져 있음을 밝힌다. | 초안 |
| **P60** | Future work and contributions, map-quality fixes, throughput, moving objects, contribution summary | 지도 갈라짐 탐지·다중 GPU·움직이는 물체 지원을 다음 과제로 제시하고, 본 연구의 기여(분해형 리그 보정, 키프레임 앵커링, 회전 모드 융합, depth 소멸 판정, 백엔드 교체형 평가 틀)를 한 장으로 정리한다. | 초안 |

---

## 확장하면서 일부러 넣지 않은 것 (백업/부록으로 남길 것)

- 이미지 메시지 바이트 레이아웃, CDR 정렬 규칙, sqlite 조회 방식 — 구현 세부
- 환경 구축 함정(conda `LD_LIBRARY_PATH`, ld64, ROS 도메인 격리) — 운영 노하우
- 파라미터 기본값 전체 표 — 부록
- 기각한 실험의 상세 수치(후보 게이트 ch1/ch2, 루프 전 관측 가중치) — 질문 대응용

이들까지 넣으면 70~80장이 되지만, 그 지점부터는 발표 본문이 아니라 문서로 읽힐 내용이다.

---

## 참고

| 파일 | 내용 |
|---|---|
| `objpose/docs/paper_flow_outline.md` | **35장 본안** (기본으로 유지) |
| `objpose/docs/ppt_260918_objpose_pipeline.md` | 페이지별 상세(본문·그림 제안·발표 노트) 108장 |
| `objpose/docs/figs/` | 결과 그림·영상 자료 |
