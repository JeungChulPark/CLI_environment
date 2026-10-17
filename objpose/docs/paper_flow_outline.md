# Paper Flow Outline — objpose: Real-Time Object 6D Pose Mapping via SLAM–SAM-6D Fusion

**목적:** 논문(및 발표)의 **전체 흐름도**. 슬라이드 한 장 = 논문 한 단락의 단위로, `페이지 / 핵심 키워드 / 한 문장 요약 / 논문 섹션 / 상태`만 관리한다.
슬라이드 본문·그림·발표 노트 같은 상세는 `ppt_260918_objpose_pipeline.md`에 있고, 이 파일은 **뼈대만** 유지한다.

**갱신 규칙:** 페이지를 하나 확정할 때마다 아래 표에 행을 추가하거나 상태를 올린다(`초안 → 검토 → 확정`).
행 순서 = 발표 순서 = 논문 서술 순서. 부록(A*)은 발표 본문에 넣지 않고 질문 대응용으로 둔다.

**상태 범례:** `확정` 내용·수치 합의됨 / `검토` 초안은 있으나 확인 필요 / `초안` 키워드만 잡힘 / `보류` 근거 확인 대기

**최종 갱신:** 2026-09-18 — 본문 35장(P01–P35) 확정/초안, 부록 A1–A3 보류

---

## Part 0 — System Overview (본 발표 P01–P02)

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P01** | Hardware rig, dual RGB-D (D455f ×2), VLP-16 LiDAR, two workstations (SLAM host / inference host) | 카트 위의 RGB-D 카메라 2대와 VLP-16 라이다, 그리고 SLAM을 담당하는 호스트와 인식·융합·시각화를 담당하는 워크스테이션으로 이루어진 전체 하드웨어 구성을 보여 준다. | 3.1 System Overview | 확정 |
| **P02** | Time synchronization, D455f 9-pin external sync (Pin5 SYNC / Pin9 GND, master–slave), host clock alignment, timestamp domain, residual offset τ | 두 카메라의 **노출 순간**은 9핀 외부 동기로 맞추고, 그것만으로는 맞지 않는 **타임스탬프**는 도메인 통일·호스트 시계차 차감·잔여 오프셋 τ 보정의 3단계로 하나의 시간축에 올린다(LiDAR는 트리거 입력이 없어 하드웨어 동기 없이 시각 정렬만 적용). | 3.2 Temporal Alignment | 검토 |

**P02 미확정 항목(보류):** 실제 녹화에서 `inter_cam_sync_mode`를 켰는지, master/slave 배정이 어떻게 됐는지 — 저장소에는 계획 문서와 주석 처리된 파라미터만 있음.

---

## Part 1 — Offline Preparation (본 발표 P03–P06)

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P03** | CAD models, colored PLY (mm, object-centred), 8,192 surface points, bounding-box extents, symmetry declaration | 인식 대상 9종을 컬러 PLY(mm 단위, 물체 중심 원점, 정점 16만~59만)로 확보하고, 여기서 PEM 입력용 8,192점 샘플·오버레이용 바운딩박스 치수·회전 대칭 선언(Sikhye·Sauce·Mugcup 3종)을 뽑아 둔다. | 3.3 Object Assets | 초안 |
| **P04** | Template rendering, 42 viewpoints, rgb/mask/xyz per view, DINOv2 feature cache, HSV reference, PEM tensors (tp/tf/tc) | 각 CAD를 42개 시점에서 렌더해 뷰마다 컬러·마스크·좌표 3종과 포즈 행렬을 만들고, 이를 한 번만 DINOv2·PEM에 통과시켜 ISM용 의미/외형 특징과 HSV 기준값, PEM용 점·특징·색 텐서로 캐시해 둔다. | 3.3 Object Assets | 초안 |
| **P05** | Offline SLAM, reference trajectory, real-time vs optimized pose (loop closure + GBA), no prior-map reuse | 두 카메라 세션을 각각 ORB-SLAM3로 오프라인 실행해 실시간 궤적과 루프 클로저·GBA가 반영된 최종 궤적을 남기고, 이 최종 궤적을 리그 보정의 입력으로 삼는다(사전 맵을 저장해 재사용하는 방식은 지도 변형을 상속해 기각). | 3.4 Rig Calibration (input) | 초안 |
| **P06** | Rig extrinsic X = T_slam_sam ∈ SE(3), planar hand-eye (AX = XB), gravity from trajectory plane, floor-plane height, joint τ estimation, rejected alternatives | 두 궤적의 상대 움직임만으로 리그 외부 파라미터 `X = T_slam_sam`와 잔여 시간 오프셋 τ를 함께 추정하고, 평면 운동이 관측하지 못하는 높이 차만 depth 바닥면에서 따로 측정해 합성한다(잔차 중앙 0.41 cm, 불확실도 약 2 mm / 0.05°). | 3.4 Rig Calibration | 검토 |

**P06 본문에 포함된 기각 사유 한 줄(A안, 2026-09-18 채택):**
> 두 지도 간 AX = ZB는 카트가 요(yaw)만 회전해 조건 불량이고, SAM 카메라를 SLAM 지도에 재측위하는 방식은 지도의 국소 변형을 그대로 상속하므로(잔차 p90 12 cm), 지도 전체를 가로지르는 비교를 버리고 **짧은 구간의 상대 움직임**으로 전환했다.

---

## Part 2 — SLAM Backends (본 발표 P07–P12)

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P07** | SLAM input data, RGB-D 640×480@30, depth-to-colour alignment, VLP-16 10 Hz scans, per-point time, shared time base | 같은 주행에서 RGB-D 카메라(640×480 30 fps, depth를 컬러 격자에 정렬)와 VLP-16(10 Hz 스캔, 점별 시각 포함)이 동시에 녹화되며, 두 입력은 같은 시간축 위에 올라가 서로 다른 SLAM 백엔드에 병렬로 공급된다. | 3.5 SLAM Backends | 초안 |
| **P08** | ORB-SLAM3 (RGB-D), feature-based, reference keyframe, loop closure, keyframe pose broadcast | 특징점 기반 RGB-D SLAM을 ROS 없이 직접 구동해 프레임마다 자세와 **참조 키프레임**을 함께 내보내며, 루프 클로저가 일어나면 전체 키프레임 자세를 갱신해 과거를 고쳐 쓸 수 있게 한다(추적 10.5–16.6 ms, 260826 7,074 포즈 무손실). | 3.5 SLAM Backends | 초안 |
| **P09** | RTAB-Map (RGB-D), graph SLAM, frequent map correction, real-time vs final pose gap | 같은 RGB-D 입력에 그래프 기반 SLAM을 붙인 대조군으로, 지도 보정이 매우 잦아(165회, 최대 17.2 cm) 실시간 자세와 최종 자세의 차이가 크다는 특성을 보인다(오도메트리 18.8–21.2 ms). | 3.5 SLAM Backends | 초안 |
| **P10** | KISS-ICP (VLP-16), LiDAR odometry, no loop closure, no keyframes, lowest latency | ROS 없이 venv에서 도는 최소 구성의 라이다 오도메트리로, 루프 클로저와 키프레임이 없는 대신 가장 매끄럽고 지연이 작다(정합 11–18 ms, 도착 지연 11–24 ms). | 3.5 SLAM Backends | 초안 |
| **P11** | hdl_graph_slam (VLP-16), ROS 2 graph SLAM, keyframes + loop closure on LiDAR | ROS 2 그래프 SLAM을 붙여 라이다에서도 키프레임과 루프 클로저(12회, 최대 보정 2.7 cm)를 제공하며, 카메라 백엔드와 동일한 방식으로 지도 보정을 파이프라인에 전달한다. | 3.5 SLAM Backends | 초안 |
| **P12** | Backend comparison (SLAM-side only), timing, arrival latency, loop closure, keyframe-anchor support, same-condition pairs | 네 백엔드를 센서·처리 시간·도착 지연·루프 클로저·키프레임 앵커 지원·드리프트 특성으로 정리하고, 어느 쌍이 동일 조건에서 비교된 것인지 명시한다(객체 위치 기준 비교는 결과 절로 미룬다). | 3.5 / 5.x | 초안 |

**P08–P11 공통 서술 템플릿(5칸):** ① 입력·전제 ② 알고리즘 한 줄 ③ 이 시스템에 붙인 방식(스트리머 파일·실행 호스트·키프레임 제공 여부) ④ 실측 성능 ⑤ 이 시스템 관점의 장단점.

**P12 주의:** 4종이 한 데이터셋에서 동시에 비교된 적은 없다(ORB↔RTAB = 260826, ORB↔KISS↔hdl = 260910). 260910 두 레인 동시 실행 수치는 표시 정렬이 퇴화해 사용 금지.

---

## Part 3 — Improved SAM-6D (본 발표 P13–P19)

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P13** | Input/output contract, RGB + aligned depth + K, per-frame independence, CAD/template assets | 이 블록은 RGB + 정렬 depth + 내부 파라미터와 물체별 CAD·템플릿 자산만 받아서 **프레임 독립적으로** 카메라 기준 6D 자세(R, t, score)를 돌려주며, 카메라가 어디 있는지도 이전 프레임에 무엇이 있었는지도 알지 못한다. | 3.6 Object Pose Estimation | 초안 |
| **P14** | Original SAM-6D, SAM everything-mode proposals, DINOv2 template matching, PEM coarse+fine, real-time limitations | 원본 SAM-6D는 SAM으로 이미지 전체에서 인스턴스 후보를 만든 뒤 DINOv2 템플릿 매칭으로 CAD를 고르고 PEM으로 자세를 푸는 구조인데, 이 방식은 실시간 속도·특정 물체만 찾는 목적·실내 유사 형상 오인식 세 가지에서 그대로 쓸 수 없었다. | 3.6 / 2. Related Work | 초안 |
| **P15** | Modified ISM ①, YOLO-World text prompts, DINOv2 semantic gate, MobileSAM box-prompted segmentation, masked appearance gate | 전 영역 분할 대신 **텍스트 프롬프트 기반 YOLO-World**가 물체별 후보 박스를 내고, DINOv2 cls 의미 게이트로 승자 박스를 고른 뒤 **MobileSAM**이 그 박스만 분할하며, 마스크 안의 patch 특징으로 외형 게이트를 통과시킨다(원본 ISM 코드는 저장소에 남아 있으나 호출되지 않는다). | 3.6.1 | 초안 |
| **P16** | Modified ISM ②, HSV hard gate, multi-label proposals, relative assignment, cross-object NMS, 959-cell human-verified GT | 색(HSV 하드게이트), 박스 하나에 여러 라벨을 허용하는 multi-label, 절대 임계 대신 후보끼리 비교하는 상대 배정과 cross-object NMS를 넣어, 사람이 직접 라벨링한 959셀 기준으로 F1 0.676 → 0.809, 오검출 334 → 28건으로 줄였다. | 3.6.1 | 초안 |
| **P17** | Modified PEM, Stage V verification (mask IoU → texture → cluster convergence), symmetry declaration, template colour channel, rejected candidate gate | PEM 본체(coarse 6000→300 + fine)는 원본을 유지하되, 기하 점수 argmax로 승자를 정하던 것을 **마스크 IoU → 텍스처 → 클러스터 수렴 순의 검증 단계(Stage V)**로 바꾸고 대칭 선언과 템플릿 색 채널을 추가했으며, 추가로 시도한 후보 사전 필터는 실측에서 순손실이라 진단용으로만 남겼다. | 3.6.2 | 초안 |
| **P18** | Real-time architecture, ROS-free inference process, shared-memory seqlock, newest-frame-only policy, warm-up | 인식을 ROS 없는 별도 프로세스로 분리하고 공유메모리(seqlock)로 최신 한 장만 주고받으며, 처리 중 들어온 프레임은 큐에 쌓지 않고 버리도록 해 **뒤처짐이 누적되지 않게** 만들었다(설계상 드롭이 기능이다). | 3.6.3 | 초안 |
| **P19** | Verification of parity + throughput/latency, per-stage budget | 실시간 경로의 판정이 오프라인 정본과 동일함을 먼저 확인하고(61프레임 95검출 완전 일치), 그 위에서 처리량과 지연을 보고한다(객체 0개 27 ms ~ 5개 1.14 s, 단계 비중 YOLO 6 % · ISM 37 % · PEM 57 %, 결과 나이 중앙 0.8 s). | 3.6 / 5.x | 초안 |

**P16 주의:** F1 0.809 · FP 28은 특정 bag들에서 **사람이 직접 판정한 959셀** 기준이므로, 수치를 인용할 때 평가 집합을 함께 명시한다.
**P17 주의:** 모델 가중치와 coarse/fine 구조는 원본, **승자 선택 로직(Stage V)은 본 연구가 추가**한 것 — 경계를 분명히 그을 것.

---

## Part 4 — Fusion (본 발표 P20–P22)

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P20** | Fusion equation, coordinate chain `T_w_obj = T_w_slam(s)·X·T_sam_obj`, world-fixed object | 융합식과 좌표 체인을 세우고, 필요한 것이 결국 "SAM 프레임 시각 s에서의 SLAM 자세" 하나임을 보인다. | 3.7 Fusion | 초안 |
| **P21** | SE(3) interpolation, Slerp + linear translation, max-gap 0.25 s, interpolation error | SLAM 30 Hz와 SAM-6D 1 Hz는 같은 순간을 찍지 않으므로 앞뒤 자세를 SE(3) 보간해 그 시각의 값을 만들고, 간격이 0.25초를 넘으면 포기한다(보간 오차 ≤3 s에서 중앙 0.5 cm). | 3.7 Fusion | 초안 |
| **P22** | Keyframe anchoring, `T_kf_obj`, kf_update propagation, loop-closure correction | 물체를 월드가 아니라 참조 키프레임 기준으로 저장해, 루프 클로저가 지도를 고칠 때 과거의 모든 물체가 함께 따라오게 한다(루프 전후 위치 차 5.4 → 0.8 cm). | 3.7 Fusion | 초안 |

---

## Part 5 — Object Memory (본 발표 P23–P27)

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P23** | Why memory, per-frame limits (noise, bimodal rotation, intermittent misses, false positives), low update rate | 프레임 독립 추정의 네 가지 한계(잡음, 회전이 180° 뒤집히는 이봉성, 간헐적 미검출, 거짓 양성)와 낮은 갱신률 때문에, 지도는 "지금 본 것"이 아니라 "지금까지 본 것의 종합"이어야 한다. | 3.8 Object Memory | 초안 |
| **P24** | Adapter structure, keyframe-anchored frame store, incremental feed, full rebuild on kf_update | 검출이 0건인 프레임까지 키프레임 기준 카메라 자세와 함께 보관하고, 평상시에는 증분으로 먹이다가 키프레임 자세가 바뀌면 전체를 보정된 좌표로 다시 재생해 메모리를 새로 만든다(308프레임 75 ms). | 3.8 Object Memory | 초안 |
| **P25** | Association gate (position-only, 0.30 m), greedy 1:1, running-mean translation, rotation mode voting, symmetry | 같은 클래스 안에서 위치 거리로만 짝짓고(회전 게이트는 180° 뒤집힘 때문에 끔), 위치는 러닝 평균, 회전은 평균이 아니라 **표가 가장 많은 자세(모드 다수결)**로 정한다. | 3.8 Object Memory | 초안 |
| **P26** | Bernoulli existence probability, expected detection probability P_D, lifecycle state machine | 각 물체에 베르누이 존재확률 r을 두고 **볼 수 있었는데 못 봤을 때만** 깎아, tentative → active → lost → remembered → deleted의 수명을 관리한다. | 3.8 Object Memory | 초안 |
| **P27** | Depth consistency check, free/occluded/surface verdicts, window confirmation, speed gate | 존재확률만으로는 "안 보이는 것"과 "없어진 것"을 구분할 수 없으므로 그 자리를 depth가 꿰뚫어 보는지로 직접 판정해 옛 사본을 지운다(창 확인·속도 게이트·3프레임 확인으로 실물 오삭제를 막는다). | 3.8 Object Memory | 초안 |

**Part 4·5 공통 원칙:** 성과 수치(8개 = 8종, 유령 삭제, 흩어짐 0.8 cm 등)는 결과 절로 미루고, P27 끝에 한 줄만 예고한다.

---

## Part 6 — Results (본 발표 P28–P32)

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P28** | Evaluation design, shared detections across lanes, metrics (observation spread, interpolation error, inter-lane object distance), same-condition rule | 같은 SAM-6D 검출을 여러 SLAM 레인에 동시에 먹이는 구조 덕분에 인식 오차를 공통 인자로 제거한 채 SLAM만 비교할 수 있으며, 비교는 반드시 같은 데이터·같은 RT 조건에서만 한다. | 4. Experimental Setup | 초안 |
| **P29** | System in operation, viewer video, live sensors and replay | 실센서 라이브와 녹화본 재생 모두에서 파이프라인이 정상 동작함을 화면으로 확인하고(박스·지도·상태가 함께 갱신), 정량 비교는 다음 장으로 넘긴다. | 5.1 System Operation | 초안 |
| **P30** | Per-input summary, session × condition table, shallow reporting | 입력(정지·이동·실내 장거리·실시간)별로 포즈 수·처리율·배치된 객체 수·관측 산포를 한 줄씩만 요약해, 조건이 결과에 어떤 영향을 주는지 얕고 넓게 보여 준다. | 5.1 System Operation | 초안 |
| **P31** | Backend comparison on identical input, object-level metrics | 같은 데이터에서 ORB-SLAM3·RTAB-Map·KISS-ICP·hdl_graph_slam이 만든 **객체 위치**를 비교해, 궤적 오차가 아니라 하위 작업 기준으로 어느 SLAM이 유리한지 보인다. | 5.2 Backend Comparison | 초안 |
| **P32** | Interpretation and failure cases, keyframe anchoring effect, instance split, alignment degeneracy | 라이다가 짧은 실내 주행에서 유리한 이유와 키프레임 앵커링의 효과(루프 전후 5.4 → 0.8 cm)를 해석하고, 인스턴스 분열·정렬 퇴화처럼 수치를 그대로 믿으면 안 되는 사례를 함께 명시한다. | 5.3 Analysis | 초안 |

**Part 6 원칙:** 라이브 런들은 장면·시간대가 달라 **백엔드 비교의 근거가 될 수 없다**(동작 확인 용도). 비교는 같은 데이터·같은 RT, 가능하면 **한 실행 안의 다중 레인**(같은 검출 공유)으로만 한다.

---

## Part 7 — Public Dataset Evaluation (P33–P35) · **예정**

GT가 있는 공개 데이터셋(YCB-Video / BOP)으로 **절대 정확도**를 재는 절. 아직 실행하지 않았고, 구성만 잡아 둔 상태다.

| # | 핵심 키워드 (keywords) | 한 문장 요약 | 논문 섹션 | 상태 |
|---|---|---|---|---|
| **P33** | Public dataset evaluation, YCB-Video / BOP, single-camera simplification (X = I), role split | 자체 데이터로는 절대 정확도를 잴 수 없으므로 GT가 있는 공개 데이터셋을 도입하되, 카메라가 한 대라 리그 보정·시계 동기가 빠지고 **인식 성능과 절대 오차**만 남는다는 점을 먼저 명시한다. | 6.1 | 예정 |
| **P34** | Metrics, BOP AR (VSD/MSSD/MSPD), ADD(-S), map-level object error | 프레임 단위 정확도는 BOP 표준 지표(AR)와 ADD(-S)로, 시스템 단위 정확도는 GT 카메라 자세로 환산한 **지도 좌표계 객체 위치 오차**로 각각 측정한다. | 6.2 | 예정 |
| **P35** | System-level results, per-frame vs. fused-with-memory, absolute error reduction | 프레임 단위 추정과 SLAM 융합 + 객체 메모리를 거친 결과를 GT 기준으로 비교해, 시간적 종합이 절대 오차를 얼마나 줄이는지 보인다. | 6.3 | 예정 |

**역할 분담 원칙:** 공개 데이터셋 = 인식 성능 + 절대 정확도 / 자체 데이터 = 시스템 검증(융합·메모리·실시간). 공개 데이터셋은 카메라 이동 범위가 작아 루프 클로저·키프레임 앵커링 효과가 거의 드러나지 않으므로, 그 기여를 여기서 주장하지 않는다.

**선행 작업(미완):**
1. 대상 객체 CAD → **42뷰 템플릿 렌더 + ISM/PEM 특징 캐시** 생성 — 렌더 파이프라인이 이 저장소에 없어 원본 SAM-6D Render 모듈을 다시 세워야 한다
2. 데이터셋 리더 어댑터(`rs_session`/`conv_session`과 같은 인터페이스)
3. BOP toolkit 연동 평가 스크립트
4. **임계값 처리 방침** — HSV 게이트 0.1214, appe 게이트 0.605는 자체 9객체에 캘리브레이션된 값이다. 고정한 채 보고하고 재캘리브레이션 결과를 병기한다

*(원안의 P34 "이식 절차"와 P36 "원본 SAM-6D와의 인식 단독 비교"는 준비 비용이 커서 이번 범위에서 제외. 여유가 생기면 이 절에 다시 넣는다.)*

---

## Appendix — 현재 **제외(보류)**

아래 백업 3장은 이번 발표 구성에서 **뺀다.** 필요해지면 되살릴 수 있도록 기록만 남긴다.

| # | 핵심 키워드 | 한 문장 요약 | 대응 질문 | 상태 |
|---|---|---|---|---|
| A1 | Template feature cache anatomy | 42뷰 렌더에서 뽑은 ISM 특징(cls·patch·HSV)과 PEM 텐서(tp·tf·tc)가 각각 어느 게이트/단계에서 소비되는지 보여 준다. | "템플릿으로 정확히 무엇을 하나?" | 보류 |
| A2 | Three-part decomposition of X | 중력(궤적 평면) → 수평 오프셋·요(상대 움직임 hand-eye) → 높이(depth 바닥면)로 X를 쪼개 푸는 이유와 단계별 잔차를 보여 준다. | "왜 한 번에 6-DoF로 풀지 않나?" | 보류 |
| A3 | Rejected calibration methods | 두 지도 AX = ZB(요만 회전이라 조건 불량)와 공유 지도 재측위(잔차 p90 12 cm)를 왜 버렸는지 수치로 보여 준다. | "다른 보정 방법은 안 되나?" | 보류 |

> A3의 결론 한 줄(= 기각 사유)은 이미 **P06 본문에 흡수**되어 있으므로, 부록을 빼도 논지는 유지된다.

---

## 다음에 채울 페이지 (예정, 키워드만)

| 예정 위치 | 키워드 | 비고 |
|---|---|---|
| P07~ | Runtime pipeline overview, per-lane architecture | 전체 데이터 흐름 한 장 |
| | SLAM streaming protocol, keyframe anchoring | `pose` / `kf_update` |
| | SAM-6D inference (ISM + PEM), shared-memory split | GIL 회피, 처리량 |
| | Fusion: SE(3) interpolation, keyframe-anchored objects | `T_w_obj = T_w_slam·X·T_sam_obj` |
| | Object memory: association, Bernoulli existence, rotation-mode fusion, depth check | |
| | Visualization & launcher | |
| | Results, ablations, limitations | |

---

## 결과 자료 (objpose/docs/figs/, 2026-09-18 생성)

| 파일 | 내용 | 출처 |
|---|---|---|
| `demo_viewer_260915_inlong_middle_orb_f3000.mp4` (96 s) | **뷰어 화면 녹화** — 영상 패널의 박스·3축 + 3D 지도 + 상태 카드가 함께 도는 실제 화면 | 260915_inlong_middle 세션을 라이브와 같은 경로로 재생(ORB-SLAM3는 노트북에서 ssh 실행) |
| `demo_viewer_highlight_30s.mp4` | 위 영상의 30초 발췌(슬라이드용) | 〃 |
| `demo2_offline_orb_f3000_260918_175925_overview.png` | 4패널 정량 그림(궤적·흩어짐·추론 비용·지연) | 같은 실행의 로그 |
| `demo2_offline_orb_f3000_260918_175925_mapbuild.mp4` (18 s) | 지도가 만들어지는 과정 애니메이션 | 〃 |
| **`cmp2lane_260915_inlong_middle_260918_214736_lane_compare.png`** | **백엔드 비교(P31 주자료)** — 같은 입력·**같은 검출**을 ORB-SLAM3와 hdl_graph_slam 두 레인에 동시에 먹인 1회 실행: 궤적 중첩·객체별 레인 간 거리·레인별 관측 산포·런타임 표 | `cmp2lane_...214736` 실행 |
| `demo_twolane_260915_inlong_middle_orb_vs_hdl.mp4` (94 s) | 위 실행의 뷰어 녹화 — 두 레인의 영상 패널과 하나의 3D 지도 | 〃 |
| `one_live_*_overview.png` / `_mapbuild.mp4` (각 7건) | 2026-09-17 **실센서 라이브** 런들의 정량 그림·애니메이션 | orb f2000/f4000, rtab, kiss, hdl |
| `live_backend_compare.png` | 라이브 백엔드 4종 비교(포즈율·도착 지연·처리율·결과 나이) | 〃 |
| `output/one_live_orb_260917_135455/viewer_screenshot_1402.png`, `output/one_live_rtab_260917_144215/viewer_screenshot_rtab.png` | 실센서 라이브 뷰어 스크린샷 | 2026-09-17 |

생성 도구: `objpose/tools/live_result_figures.py`(로그 → 그림·애니메이션), `objpose/tools/record_viewer.py`(뷰어 페이지 → mp4, 헤드리스 크롬 사용으로 데스크톱 화면을 쓰지 않음).

---

## 참고 문서

| 파일 | 내용 |
|---|---|
| `objpose/docs/ppt_260918_objpose_pipeline.md` | 페이지별 상세(본문·그림 제안·발표 노트), 108페이지 |
| `objpose/README.md` | 실행 방법과 실험 결과 원본 |
| `objpose/docs/rig_rt_260915.md` | 리그 RT 재추정(260915) |
| `objpose/docs/sam6d_inference_speed_260914.md` | SAM-6D 처리량·지연 실측 |
| `objpose/docs/loop_split_analysis_260914.md` | 루프 클로저 후 객체 분열 분석(F1·F2) |
| `_bmad_output_for_slam/planning-artifacts/research/technical-ros2-multilaptop-sensor-timestamp-sync-research-2026-07-07.md` | 시간 동기화 설계 근거(9핀·PTP·LiDAR 결정) |
