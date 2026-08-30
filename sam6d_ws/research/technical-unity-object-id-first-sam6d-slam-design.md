# Unity 3D Map 중복 방지를 위한 Object ID / Object Memory 우선 설계 (Technical Research)

> **작성일**: 2026-06-25
> **연구자**: jucpark
> **목적**: SAM-6D 6D pose + SLAM pose를 결합해 Unity 3D Map 위에 객체를 **중복 없이** 표시하기 위한 persistent object id / object memory 구조를 우선 조사하고, "Multi-object가 먼저인가, Object ID/Memory가 먼저인가"를 냉정하게 재판단한다.
> **선행 보고서**: `research/technical-slam-aware-multi-object-sam6d-priority-roadmap.md` (기존 결론: Multi-object Wrapper 1순위 / Object Memory 2순위)
> **제약**: 코드 미수정, 조사·설계만, 사용자 가설 무비판 수용 금지, PEM 내부 수정 제외, 한국어.

---

## 1. Executive Summary

### 1.1 기존 보고서의 Multi-object 1순위 판단 재검토

기존 보고서가 Multi-object Wrapper를 1순위로 둔 것은 **"SAM-6D 자체의 능력 확장"** 관점에서는 타당했다. 그러나 그 판단의 전제는 *deliverable이 "멀티객체 pose 벤치마크"* 였다. **이번에 deliverable이 "Unity 3D Map에 중복 없는 persistent 객체 표시"로 바뀌었다.** 이 변경은 시스템의 **binding constraint(병목 제약)**를 바꾼다.

- 기존 병목: "한 프레임에서 객체를 몇 개나 동시에 추정하는가" (공간적 확장).
- 변경된 병목: "같은 객체를 프레임을 넘어 **하나의 동일 객체로 유지**하는가" (시간적 동일성/identity).

Unity 표시 목표에서는 후자가 **선결 제약**이다. 따라서 **기존 1순위 판단은 새 목표 하에서 수정되어야 한다.**

### 1.2 Unity 요구사항 반영 후 최종 우선순위

```
1순위: SLAM 좌표계 기반 Object ID / Object Memory  (1~2개 객체로 시작)
2순위: Unity Object Map Publish / ROS2–Unity Interface  (1순위와 한 묶음 MVP)
3순위: Multi-object SAM-6D Wrapper (N객체 scaling — config 확장)
4순위: Pose Jitter Reduction (memory에 흡수, 독립 단계 아님)
5순위: Object Landmark Graph (schema/시각화/post-hoc, full opt는 future work)
```

> **핵심**: 1순위와 2순위는 **분리 가능한 두 단계가 아니라 하나의 MVP**다. "중복 없는 Unity 표시"는 memory node(id 부여)와 unity bridge(id 기반 렌더)가 **동시에 있어야** 실증되기 때문이다.

### 1.3 사용자 가설의 타당성 판단

> 사용자 가설: "1순위는 Multi-object가 아니라 Object ID / Object Memory다. Unity에 표시하려면 동일 객체를 같은 id로 유지해야 하고, 아니면 매 프레임 중복 생성된다."

**판단: 대체로 옳다(★ 채택). 단 3가지 보완이 필요하다.**

- ✅ **옳은 핵심**: Unity 중복 생성의 근본 원인은 "프레임 간 stable identity 부재"이며, 이는 multi-object 추정과 **직교(orthogonal)**하는 문제다. 객체가 1개여도 매 프레임 새로 만들면 중복이 쌓인다. → identity가 multi-object보다 선행해야 한다는 결론은 정당하다.
- ⚠️ **보완 1 (drift)**: 중복의 원인은 "id 부재"만이 아니다. **SLAM drift로 같은 객체의 `T_map_object`가 떠다니면, id 로직이 있어도 false new / false merge가 발생**한다. memory는 drift에 강건해야 한다(가설이 과소평가).
- ⚠️ **보완 2 (렌더 dedup은 이미 공짜)**: 중복 방지의 *렌더링* 측면은 `visualization_msgs/MarkerArray`의 `(ns,id)` + ADD/MODIFY/DELETE/lifetime 의미론으로 이미 해결된다. 진짜 어려운 부분은 **id "부여(association)"**이지 "id 기반 렌더"가 아니다. memory node의 가치는 association에 있다.
- ⚠️ **보완 3 (단일 객체 memory의 약함)**: 단일 객체 + "한 pose 저장"은 기술적으로 trivial하다. 논문적으로 의미를 가지려면 **재방문(revisit) 시나리오에서 re-identification(association)**이 반드시 MVP에 포함되어야 한다. 그래야 "중복 없음"이 실증된다.

결론: **사용자 가설을 채택해 우선순위를 재배열하되, "단일 pose 저장"이 아니라 "재관측 association + drift 강건성 + Unity id-기반 렌더"를 묶은 MVP로 구체화**한다.

---

## 2. Problem Re-definition

### 2.1 기존 문제
- **"SAM-6D를 멀티객체로 확장한다."** 한 프레임에서 N개 객체의 6D pose를 얻는 것이 목표. 시간축·표시 일관성은 범위 밖이었다.

### 2.2 변경된 문제
- **"Unity 3D Map 위에 객체를 중복 없이, 일관된 identity로 persistent하게 표시한다."** 입력은 SAM-6D pose + SLAM pose, 출력은 Unity가 소비하는 **object map state(객체별 id·pose·state)**다.

### 2.3 왜 object id가 핵심 문제가 되는가
Unity는 메시지를 받아 GameObject를 그린다. Unity의 표준 패턴은 **`Dictionary<id, GameObject>`**: id가 새로우면 Instantiate, 있으면 Transform update, 사라지면 Destroy. **만약 ROS2가 매 프레임 "raw detection"을 보내면서 안정적 id가 없으면**, Unity는 매 프레임 같은 객체를 **새 GameObject로 Instantiate** → 같은 위치에 동일 객체가 수십 개 중첩된다. 즉 **stable id가 시스템의 정확성을 결정하는 1차 변수**다. 이 id는 **누군가 stateful하게 부여**해야 하며, 그 책임을 Unity에 떠넘기면 안 된다(이유는 §3, §6).

---

## 3. Critical Analysis of User Hypothesis

### 3.1 가설이 맞는 부분
1. **Identity는 multi-object와 직교한다.** 객체 수와 무관하게 "프레임 간 동일성"이 없으면 Unity 중복이 발생한다. 따라서 identity를 먼저 푸는 것은 논리적으로 정당하다.
2. **memory node는 ROS2 쪽에 있어야 한다.** association은 stateful 로직(누적 관측, world 좌표, 거리 게이트)이며, 이를 Unity(렌더 클라이언트)에 두면 (a) SLAM/depth/CAD 정보 접근이 어렵고 (b) 재사용·디버깅·평가가 불가능하다. "object_memory_node가 필요하다"는 가설은 옳다.
3. **Unity에는 raw detection이 아니라 fused object map state를 보내야 한다.** (§5 RQ 답변과 일치)

### 3.2 틀릴 수 있는 부분 / 과장 위험
1. **"id만 유지하면 중복이 사라진다"는 부분적 진실.** SLAM drift, depth 노이즈, association 임계값 오설정 시 **id가 있어도** false new(중복) / false merge(서로 다른 객체를 하나로)로 깨진다. id는 필요조건이지 충분조건이 아니다.
2. **"단일 객체 memory를 먼저"가 그 자체로 논문 기여라고 보면 약하다.** 단일 객체 1-pose 저장은 trivial. 재관측 association을 포함하지 않으면 "중복 방지"를 실증할 수 없다.
3. **렌더 측 dedup은 신규 개발이 아니다.** `MarkerArray (ns,id)`로 이미 해결되는 부분을 "구현해야 할 핵심"으로 오해하면 노력이 잘못 배분된다.

### 3.3 보완해야 할 부분
- MVP에 **revisit(이동 후 재관측) 시나리오**를 반드시 포함 → association/re-ID를 실제로 작동·평가.
- **drift 강건성** 설계: keyframe 시점 TF 사용, association은 절대 거리뿐 아니라 상대/최근성 가중, drift 보정 후 재정렬(추후).
- **id 부여(association)와 id 렌더(MarkerArray)**를 개념적으로 분리해 노력 배분.

### 3.4 최종 판단
**사용자 가설 채택.** "Object ID / SLAM-based Object Memory"를 1순위로 재배열하는 것은 **새 목표(Unity persistent 표시) 하에서 명백히 옳다.** 단, MVP의 정의를 "단일 pose 저장"이 아니라 **"재관측 association + drift-aware world mapping + Unity id-기반 렌더"**로 격상해야 방어 가능한 contribution이 된다. Multi-object wrapper는 memory 파이프라인이 검증된 뒤 **config 확장(N=1→N)**으로 흡수하는 것이 위험·노력 면에서 합리적이다.

---

## 4. Revised Priority Roadmap

| Priority | Feature | Goal | Why First/Next | Minimum Output | Stop 가능 여부 |
|---|---|---|---|---|---|
| **1** | SLAM 기반 Object ID / Object Memory | 1~2객체의 `T_map_object` 저장 + 재관측 association + persistent id | Unity 중복의 **근본 제약(identity)**; multi-object와 직교; 모든 후속의 입력 | object map state(id, class, T_map_object, conf, count, last_seen) + association 로그 | ✅ (단 Unity와 묶어야 실증) |
| **2** | Unity Object Map Publish / ROS2–Unity Interface | object map state를 Unity로 publish, id 기반 create/update/delete | 1순위의 효과(중복 없음)를 **실증하는 화면**; 묶음 MVP | Unity에 중복 없이 표시되는 데모 + 메시지 schema | ✅ (1과 함께 = MVP) |
| **3** | Multi-object SAM-6D Wrapper | per-mask PEM으로 N(≤5)객체 동시 처리 | memory 파이프라인 검증 후 **config 확장**(저위험 scaling) | N객체 object map + runtime 표 | ✅ |
| **4** | Pose Jitter Reduction | 누적 관측의 confidence-weighted SE(3) fusion | memory의 **자연스러운 부산물**; 독립 단계 불필요 | jitter(static std/velocity) before/after 곡선 | ✅ (3에 흡수) |
| **5** | Object Landmark Graph | keyframe+object node graph **schema/저장/시각화/post-hoc** | Unity 표시에 full opt 불필요; future work 토대 | g2o/GTSAM 호환 schema + 시각화 + consistency 분석 | ✅ (schema 수준 stop) |

> **두 우선순위 비교 결론**: 기존(Multi-object 1순위)은 "벤치마크" 목표에 최적, 신규(Object ID 1순위)는 "Unity persistent 표시" 목표에 최적. **현재 목표가 후자이므로 신규 우선순위가 타당하다.** 단 신규안의 1·2순위는 한 MVP로 묶고, 3순위(multi-object)는 "이미 거의 완성된 자산"이므로 메모리 검증 직후 빠르게 흡수한다.

---

## 5. Recommended MVP (1개월 내)

**목표 한 줄**: *"1~2개 객체를, 로봇이 이동했다가 재방문해도 Unity 3D Map에 중복 없이 같은 자리·같은 id로 표시한다."*

포함 여부 검토 결과:

| 항목 | 포함? | 비고 |
|---|---|---|
| 단일/소수 객체 pose input | ✅ 필수 | 기존 ISM→PEM 브릿지 재사용, N=1~2 |
| SLAM pose 수신 | ✅ 필수 | ORB-SLAM3 TF `map→camera` 구독 |
| `T_map_object` 변환 | ✅ 필수 | `T_map_object = T_map_camera · T_camera_object` |
| object memory 저장 | ✅ 필수 | in-memory dict + JSON/SQLite 스냅샷 |
| object_id 부여 | ✅ 필수 | association 성공 시 기존 id, 실패 시 신규 |
| 재관측 시 update | ✅ 필수 (핵심) | **revisit 시나리오로 실증** — MVP의 정수 |
| Unity publish용 map state 생성 | ✅ 필수 | object map state 메시지(또는 MarkerArray) |
| RViz/Unity mock 시각화 | ✅ 필수 | **RViz MarkerArray로 먼저 실증**(공짜 dedup), Unity는 병행/후속 |
| confidence-weighted pose fusion | ◐ 선택 | 시간 남으면 jitter 절 흡수 |
| appearance feature association | ✗ 제외 | 초기엔 class+거리+크기로 충분(§8) |

**MVP 권장 전략**: Unity 통합 전에 **RViz `MarkerArray`로 "중복 없음"을 먼저 검증**하라. `(ns,id)` + DELETE/lifetime이 Unity와 동일한 dedup 의미를 공짜로 제공하므로, memory/association 로직을 Unity 의존 없이 빠르게 디버깅할 수 있다. 그 다음 동일 메시지를 ROS-TCP-Connector로 Unity에 흘린다.

---

## 6. ROS2 Architecture

```
RGB-D / ros2 bag ──┬──► slam (ORB-SLAM3)  ── TF: map→camera, /slam/keyframes
                   │
                   └──► ism_node (YOLO-World+MobileSAM) ─ /ism/proposals (mask,bbox,class,score)
                              │
                              ▼
                        sam6d_pose_node (PEM, per-mask)
                              │  /objects/detections (Detection: T_camera_object, class, score, stamp)
                              ▼
                        object_memory_node  ◄── TF map→camera (관측 시점)
                              │  · world 변환 · association · id · fusion
                              │  /objects/map_state (ObjectStateArray)
                              ├───────────────► (RViz) /objects/markers (MarkerArray)
                              ▼
                        unity_bridge_node (ROS-TCP-Endpoint)
                              │  TCP
                              ▼
                           Unity (ROS-TCP-Connector, Dictionary<id,GameObject>)
```

| Node | Subscribe | Publish / 비고 |
|---|---|---|
| `slam_node` (기존) | `/camera/rgb`, `/camera/depth` | TF `map→camera`, `/slam/keyframes` |
| `sam6d_pose_node` | `/ism/proposals`, `/camera/depth`, `/camera/info` | `/objects/detections` (per-frame raw, `T_camera_object`+class+score+stamp) |
| `object_memory_node` ★핵심 | `/objects/detections`, TF `map→camera` | `/objects/map_state` (fused, id 부여), `/objects/markers` (MarkerArray); service `reset_map`, `query_object` |
| `unity_bridge_node` | `/objects/map_state` | ROS-TCP-Endpoint 경유 Unity로 전달 (별도 코드 거의 없음; Endpoint 실행 + 메시지 등록) |
| `multi_object_wrapper_node` (optional, P3) | `/ism/proposals` | `sam6d_pose_node`를 N-mask 루프로 일반화 (별도 노드 대신 pose_node 내부 확장 권장) |
| `graph_node` (optional, P5) | `/objects/map_state`, `/slam/keyframes` | graph export, `/objects/graph_viz` |

설계 포인트:
- **무거운 추론(ISM/PEM)은 keyframe-triggered 또는 rate-limited.** SAM-6D 지연(객체당 ~1.5s) 때문에 매 프레임 불가.
- **관측 시점 TF로 world 변환**(지연 보정). `message_filters` ApproximateTime 또는 keyframe stamp 정렬.
- **단위 일관성**: CAD mm ↔ ROS m, ROS FLU ↔ Unity RUF(좌표/쿼터니언 변환은 bridge에서). 변환 지점 단일화.

---

## 7. ROS2–Unity Message Schema

### 7.1 권장: 2단계 전략
- **Phase A(디버그/실증)**: `visualization_msgs/MarkerArray` 재사용. `marker.ns="objects"`, `marker.id=object_id`, `action=ADD(0)/DELETE(2)`, `lifetime`으로 LOST 자동 만료, `pose`=`T_map_object`. → **RViz·Unity 양쪽에서 (ns,id) 기반 dedup이 공짜.** class/conf는 `text` marker 병행으로 임시 표시.
- **Phase B(의미 정보 필요 시)**: 커스텀 `ObjectState.msg` / `ObjectStateArray.msg` 도입(아래). class·confidence·observation_count·covariance 등 thesis 지표를 담는다. ROS-TCP-Connector의 MessageGeneration으로 Unity C# 클래스 생성(주의: Humble branch의 common_interfaces로 생성).

### 7.2 커스텀 메시지 (제안)
```
# ObjectState.msg
int32              object_id
string             class_name
geometry_msgs/Pose pose_map_frame      # T_map_object (ROS FLU, m)
geometry_msgs/Vector3 extent           # scale (m)
float32            confidence
uint8              state               # 0 NEW, 1 UPDATE, 2 LOST(미관측·가려짐 UNKNOWN), 3 DELETE(제거확정 ABSENT)
builtin_interfaces/Time last_seen_time
uint32             observation_count
uint32             negative_count       # 기대 가시성 충족했으나 미검출된 누적 횟수 (removal 판정용)
float32            presence_belief      # 0~1, 존재 확신도 (positive/negative 증거 누적)

# ObjectStateArray.msg
std_msgs/Header header                 # frame_id = "map"
ObjectState[] objects
```

### 7.3 Unity 쪽 동작
- 수신 시 `Dictionary<int, GameObject>` 갱신:
  - **id 없음 + state=NEW** → Instantiate(prefab by class), `transform.position = pose.position.From<FLU>()`, `transform.rotation = pose.orientation.From<FLU>()` (FLU→RUF, 쿼터니언 `-w` 주의).
  - **id 있음 + state=UPDATE** → 기존 GameObject Transform만 갱신(중복 생성 X).
  - **state=LOST (UNKNOWN: 미관측·가려짐)** → 반투명/회색 등 시각적 표시(파괴는 보류). "확신 못 함"을 의미하며 timeout만으로 지우지 않는다.
  - **state=DELETE (ABSENT: 제거 확정)** 또는 lifetime 만료 → Destroy + dict 제거. **이 신호는 negative observation 누적(§8.5)으로만 발생**해야 ghost object를 막는다.
- **전송 방식**: 매 publish마다 *현재 살아있는 객체 전체 배열*(snapshot) 권장. Unity가 자신의 dict와 diff하여 이번에 없는 id를 timeout 기반 reap → 패킷 유실에 강건. (incremental event 방식은 재접속 시 DELETEALL 필요.)

---

## 8. Object Association Design

`object_memory_node`가 새 detection `d`(world 변환 후 `T_map_object_d`, class, extent, conf)를 받았을 때:

1. **Class gate (필수, 1차 차단)**: `class(d)`와 같은 class의 기존 객체만 후보. 다르면 즉시 신규 후보.
2. **3D distance gate (핵심)**: 후보 객체 중 `‖t_map_object_d − t_map_object_i‖ < τ_d` 인 것. `τ_d`는 객체 크기·SLAM 정확도 기반(예: 0.1~0.3 m). 가장 가까운 것 선택.
3. **Extent / CAD size consistency**: extent 비율이 임계 내(예: 0.7~1.4×)일 때만 매칭 허용. depth 노이즈로 인한 오매칭 차단.
4. **Orientation consistency (보조)**: geodesic 각도 차가 임계 내(대칭 객체는 ADD-S식 완화). 초기엔 weak gate.
5. **Confidence update**: 매칭 시 `conf ← max` 또는 running mean, `observation_count++`, `last_seen ← now`.
6. **Merge rule**: 두 기존 객체가 누적 후 서로 거리 게이트 내로 수렴하면 병합(작은 id 유지, 관측 합산). drift 보정 후 발생 가능.
7. **New object rule**: 어떤 게이트도 통과 못 하면 **신규 id 생성**. 단 **단발성 오검출 억제**를 위해 `n_consecutive ≥ k`(예 2~3회) 관측 후에만 NEW로 승격(잠정 버퍼).
8. **Lost object rule**: `now − last_seen > τ_lost` → state=LOST(표시 유지, 매칭 후보에선 제외 또는 가중 하락).
9. **Delete rule**: `now − last_seen > τ_delete`(τ_delete ≫ τ_lost) 또는 관측이 지속 모순(반복 false) → DELETE.

**appearance feature 없이 가능한가? → 가능(초기 권장).** 정적 씬·소수 객체·서로 다른 class 환경에서는 **class + 3D distance + size**만으로 충분하다. appearance는 (a) 같은 class 다중 인스턴스가 근접하거나 (b) drift가 클 때만 필요 → Phase C 이후 옵션.

### 8.5 Object Removal & Negative Observation (객체 부재 처리) ★보강

**문제**: §8.8/8.9의 LOST/DELETE는 `last_seen` **timeout 기반**이라 "객체가 실제로 치워졌다(ABSENT)"와 "그냥 그쪽을 안 봤다·가려졌다·검출 실패(UNKNOWN)"를 **구분하지 못한다.** timeout만 쓰면 (a) 치워진 객체를 한참 Unity에 남기는 **ghost object**, 또는 (b) 멀쩡한 객체를 성급히 삭제하는 오류가 생긴다. 본 절은 이를 **negative observation(없음을 적극 확인)**으로 해결한다.

**8.5.1 객체 상태 3값화 (2값 → 3값)**
- `PRESENT`: 최근 positive 관측, 존재 확신.
- `UNKNOWN`(=Unity LOST): 미관측·가려짐·시야 밖 → "모름". **삭제하지 않음.**
- `ABSENT`(=Unity DELETE): negative evidence 누적으로 **제거 확정.**

**8.5.2 기대 가시성(expected visibility) 추론 — 핵심 신규 컴포넌트**
새 프레임에서 기존 객체 `i`가 detection되지 않았을 때, 그것이 *유의미한 negative*인지 판정:
1. **Frustum check**: `T_map_object_i`가 현재 카메라 시야(frustum) 안인가?
2. **Occlusion check**: depth/맵 raycast로 그 위치가 **가려지지 않았는가**? (앞에 다른 표면이 없는가)
3. **Reliability check**: 거리·각도가 검출 신뢰 구간 내인가(너무 멀거나 비스듬하면 negative로 안 셈)?
→ 세 조건 모두 충족 + 미검출 = **negative evidence 1표** (`negative_count++`). 하나라도 불충족이면 `UNKNOWN`(증거 아님).

**8.5.3 ⚠️ Recall 보정 — 단발 negative 금지 (이 시스템의 결정적 caveat)**
본 ISM의 **recall ≈ 0.62**(9-bag audit) → "안 보임"의 ~38%는 객체 부재가 아니라 **검출 실패**다. 따라서:
> **단 한 번의 non-detection으로 ABSENT를 선언하면 안 된다.** 기대 가시성을 충족한 **N회 연속(예 3~5) negative**가 누적되어야 `ABSENT`로 승격.

`presence_belief`를 Bayesian-style로 갱신(positive 관측 시 ↑, 검증된 negative 시 ↓, detector recall을 likelihood로 반영)하고 임계 이하에서 ABSENT 판정하는 것이 timeout보다 원리적이다. (Fusion++의 *existence probability* 선례)

**8.5.4 갱신된 규칙**
- **Removal rule**: `negative_count ≥ N` (기대 가시성 충족분) **AND** `presence_belief < θ` → `ABSENT` → Unity DELETE.
- **Re-appearance**: ABSENT 직전 객체가 같은 위치·class로 재검출되면 → 같은 id 복원(즉시 삭제하지 않고 잠정 보존했다면) 또는 신규 id(완전 삭제 후).
- **Moved object**: 옛 위치 negative 누적(→ABSENT) + 인근 신규 위치 positive → "이동"으로 해석할지(같은 id 이전) "삭제+생성"으로 볼지는 정책 선택(§S8 시나리오로 평가).
- **timeout은 fallback로만**: 시야에 한 번도 안 들어온 채 매우 오래된 객체의 `τ_delete` archival(메모리 정리용)로 격하, 표시 삭제의 1차 근거에서 제외.

**8.5.5 영향 범위 (방향성은 안 바뀜)**
노드 그래프·Unity schema는 거의 그대로(상태값·negative_count 추가뿐). 유일한 신규 비용은 **frustum+occlusion 가시성 추론**. 단 이 보강으로 contribution이 *persistent map* → **object-level semantic map maintenance / change detection**으로 강화된다(§10 Phase B).

---

## 9. Evaluation Plan

| 지표 | 정의 / 측정 | 적용 |
|---|---|---|
| **Unity duplicate object count** | 동일 실제 객체에 대해 Unity에 생성된 GameObject 수(이상=1) | MVP 핵심 KPI |
| **Object id switch count** | 한 실제 객체의 id가 바뀐 횟수(MOTA류) | MVP |
| **False new object count** | 기존 객체를 신규로 잘못 생성한 수 | MVP |
| **False merge count** | 서로 다른 객체를 하나로 합친 수 | MVP |
| **Re-identification accuracy** | 재방문 시 올바른 id로 association한 비율 | MVP (revisit 시나리오) |
| **Pose jitter (before/after fusion)** | static 구간 pose std, velocity/accel 크기 | P4 |
| **Pose accuracy 유지** | ADD/ADD-S, fusion이 정확도 악화 안 시킴 | P4 |
| **Runtime** | detection→map_state→Unity end-to-end latency, 객체 수별 | 전 단계 |
| **SLAM drift sensitivity** | drift 크기(또는 trajectory 노이즈 주입) vs id switch/duplicate 곡선 | P1 강건성 |
| **Ghost object count** ★ | 실제 제거된 객체가 Unity에 남아있는 수(이상=0) | removal(§8.5), S7 |
| **Removal detection precision/recall** ★ | ABSENT 판정의 정확도(검출실패를 제거로 오판=FP, 진짜 제거 놓침=FN) | removal, S7 |
| **Removal detection latency** ★ | 객체 제거 시점 ~ ABSENT 판정까지의 시간/프레임 수 | removal, S7 |
| **Moved-object handling** ★ | 재배치 시 옛 위치 삭제 + 새 위치 일관 id 유지 여부 | S8 |
| **Ablation** | (no memory, raw→Unity) vs (memory) vs (memory+fusion) vs (memory+removal) — duplicate/id switch/ghost | 전 단계 |

> **가장 강한 정량 메시지**: "raw detection을 직접 Unity에 보내면 N프레임당 ~N개 중복이 생기지만, object_memory_node를 거치면 duplicate=0, id switch≈0을 유지한다" + "drift X cm까지 association이 견딘다" + "**객체 제거 시 ghost 없이 negative evidence N회 내에 ABSENT로 정리하며, recall 0.62 검출기에서도 false removal을 억제한다**".

---

## 10. Thesis Contribution Framing

### Phase A — Object ID + Unity-safe Object Memory (MVP)
- **Contribution 문장**: "SAM-6D의 frame-wise 6D pose를 SLAM 궤적으로 map 좌표계에 누적하고, class+geometry 기반 association으로 persistent object id를 부여하여, ROS2–Unity 인터페이스에서 동일 객체의 중복 생성 없이 일관된 3D Map 표시를 실현하는 시스템을 제안·실증했다."
- **과장 금지**: "object-level SLAM을 풀었다", "tracking을 해결했다", "새 pose estimator".
- **안전한 주장**: "persistent object memory 기반 **중복 없는** Unity 표시 통합·실증", "단일/소수 객체 재관측 re-ID".

### Phase B — Multi-object Object Memory + Map Maintenance (객체 제거 포함)
- **Contribution 문장**: "위 구조를 per-mask SAM-6D orchestration으로 확장하여 최대 5개 객체의 동시 persistent mapping을 달성하고, **기대 가시성 기반 negative observation으로 객체 등장·유지·제거를 추적하는 동적 맵 유지(map maintenance / change detection)**를 구현했다 — 특히 recall 0.62 검출기에서도 단발 미검출을 제거로 오판하지 않도록 evidence 누적으로 robust하게 처리했다."
- **과장 금지**: "실시간 멀티객체 추적", "완전한 동적 환경 SLAM".
- **안전한 주장**: "멀티객체 persistent map + runtime/정확도 trade-off 정량화", "**객체 제거에 robust한 ghost-free 맵 유지**(low-recall 검출기 조건)".

### Phase C — Jitter Reduction
- **Contribution 문장**: "object memory의 누적 관측을 confidence-weighted SE(3) fusion으로 결합하여, 단일 프레임 대비 pose jitter를 줄이면서 정확도를 유지했다."
- **과장 금지**: "필터링이 핵심 novelty"(단독은 incremental).
- **안전한 주장**: "memory-conditioned 안정화가 jitter–accuracy trade-off를 개선", "naive smoothing의 lag 함정을 실측·회피".

### Phase D — Object Landmark Graph
- **Contribution 문장**: "object memory를 camera–object factor-graph schema로 구조화하고 멀티뷰 consistency를 분석하여, full pose-graph optimization으로의 확장 토대를 제공했다."
- **과장 금지**: "joint optimization으로 drift를 줄였다"(미구현 시).
- **안전한 주장**: "graph schema·export·consistency 분석 제공, optimization은 future work".

---

## 11. Final Recommendation

### 11.1 지금 당장 구현해야 할 1순위
**`object_memory_node`(SLAM 기반 Object ID/Memory) + `unity_bridge`를 하나의 MVP로 구현.** 1~2개 객체로 시작하되, **반드시 "이동 후 재방문" 시나리오에서 duplicate=0 / 일관된 id를 실증**한다. multi-object는 그 다음 config 확장.

### 11.2 기존 보고서에서 수정해야 할 결론
- **수정**: "Multi-object Wrapper가 1순위 MVP"라는 결론은 **deliverable이 벤치마크일 때만 유효**. **Unity persistent 표시가 목표인 현재는 Object ID/Memory가 1순위**로 교체된다.
- **유지**: Pose Jitter는 memory 흡수, Object Landmark Graph는 schema 수준 후순위·future work라는 기존 판단은 그대로 유효.
- **추가**: 중복 방지의 *렌더 dedup*은 `MarkerArray (ns,id)`로 공짜이며, 노력은 *id 부여(association)*에 집중해야 한다는 점 명시.

### 11.3 뒤로 미뤄야 할 기능
- Multi-object N≥3 scaling(메모리 검증 후), appearance-feature association, jitter용 명시적 필터, Object Landmark Graph full optimization, PEM 내부 수정(범위 외).

### 11.4 1~2개월 내 stop-point
- **~4주**: Phase A(Object ID + Unity-safe memory, 1~2객체, revisit 실증) — **최소 방어선이자 권장 정지선**.
- **~6주**: Phase B(multi-object까지) — contribution 강화.
- **~8주+**: Phase C(jitter fusion) / Phase D(graph schema) — 여유 시.

### 11.5 다음 `/bmad-architect` 또는 `/bmad-prd`로 넘길 요구사항
1. **`object_memory_node`**: detection 구독 → 관측 시점 TF로 world 변환 → §8 계층적 association(class→distance→extent→orientation) → persistent id(잠정 버퍼·merge·lost·delete 규칙) + **§8.5 negative observation(기대 가시성 추론 + presence_belief 기반 removal)** → `/objects/map_state` + `/objects/markers` 발행, service `reset_map`/`query_object`. (Phase A=removal 단순 timeout, Phase B=가시성 기반 정식 removal)
2. **메시지 계약**: Phase A=`visualization_msgs/MarkerArray (ns,id,action,lifetime)`, Phase B=커스텀 `ObjectState`/`ObjectStateArray`(§7.2). Unity MessageGeneration은 Humble branch 사용.
3. **`unity_bridge`**: ROS-TCP-Endpoint 셋업, FLU↔RUF 좌표/쿼터니언 변환(−w 주의), Unity `Dictionary<id,GameObject>` create/update/lost/delete 로직.
4. **시간 동기화 정책**: keyframe-triggered vs ApproximateTime 결정(아키텍처 핵심).
5. **평가 하네스**: §9 지표 자동 산출(duplicate count, id switch, false new/merge, re-ID, drift sensitivity, ablation no-memory vs memory).
6. **비기능**: CAD mm/ROS m/Unity RUF 단위·좌표 변환 단일화, 정적 씬·소수 객체(≤5) scope, ORB-SLAM3 우선.

---

## 12. Research Questions 직접 답변

1. **Unity까지 고려하면 Object ID/Memory가 먼저인가?** → **예.** identity는 multi-object와 직교하며, Unity 중복의 binding constraint다.
2. **persistent id 없이 raw pose를 Unity에 publish하면?** → Unity가 매 프레임 같은 객체를 새 GameObject로 Instantiate → 동일 위치 중복 누적, pose jitter가 시각적으로 증폭, 객체 수·메모리 폭증, 의미 없는 표시.
3. **단일 객체라도 memory를 먼저 구현하는 게 의미 있나?** → **있다(단 조건부).** 단순 1-pose 저장은 trivial하지만, **재방문 association을 포함**하면 시스템·논문적으로 유의미("중복 없는 persistent 표시"의 최소 실증).
4. **Multi-object는 memory 이후 확장해도 되나?** → **된다.** memory 파이프라인은 객체 수와 직교하며, multi-object는 per-mask 루프(config N) 확장으로 저위험 흡수.
5. **Unity에 raw vs fused map state?** → **fused object map state.** raw는 중복·jitter를 Unity로 전가. association·fusion은 ROS2 memory node 책임.
6. **최소 메시지 schema?** → §7.2. 시작은 `MarkerArray(ns,id,action,pose,lifetime)`로 충분, 의미 정보 필요 시 `ObjectState(id,class,pose_map,extent,conf,state,last_seen,obs_count)`.
7. **object_id 생성/유지/병합/삭제 기준?** → §8: 게이트 실패+연속관측 k회 시 **생성**, association 성공 시 **유지/update**, 누적 후 거리 수렴 시 **병합**. **삭제는 timeout이 아니라 §8.5 negative observation 기반** — 기대 가시성(frustum+occlusion) 충족분 N회 연속 미검출 + presence_belief<θ일 때만 ABSENT→delete(recall 0.62 보정).
8. **SLAM drift의 영향?** → drift가 `T_map_object`를 이동시켜 **false new(중복)·false merge·id switch** 유발. 완화: 관측 시점 TF, 상대/최근성 가중 association, 큰 τ_d 회피(과병합 위험)와의 균형, 추후 drift 보정·graph 재정렬. **drift sensitivity를 평가 지표로 명시**.
9. **memory 기반 jitter reduction이 자연스러운가?** → **예.** 같은 id의 누적 관측을 confidence-weighted SE(3) fusion하면 별도 필터 없이 jitter 감소. 독립 1순위 불필요.
10. **논문 contribution으로 방어 가능한가?** → **예(시스템 통합 contribution).** "open-set 검출(YOLO-World/MobileSAM)+zero-shot 6D(SAM-6D)+SLAM 궤적을 persistent object memory로 묶어 ROS2–Unity에서 중복 없는 3D Map 표시를 실증". 알고리즘 발명이 아닌 **통합·실증·평가**로 정직하게 포지셔닝하면 석사 논문/보고서급으로 충분.

---

## 13. 참고문헌 (검증 출처)

- SAM-6D: Lin et al., CVPR 2024. [arXiv:2311.15707](https://arxiv.org/abs/2311.15707) · [code](https://github.com/JiehongLin/SAM-6D)
- Unity Robotics Hub: [github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- ROS-TCP-Connector: [github.com/Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) · 좌표 변환 [ROSGeometry.md](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/ROSGeometry.md) · Humble 메시지 이슈 [#319](https://github.com/Unity-Technologies/ROS-TCP-Connector/issues/319)
- visualization_msgs/Marker (ADD/MODIFY/DELETE/DELETEALL, lifetime): [docs](https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html) · [Marker.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/Marker.msg)
- 대안 브리지: [ros2-for-unity](https://discourse.openrobotics.org/t/ros2-for-unity/22022) · [rosbridge_suite + ROS2](https://foxglove.dev/blog/using-rosbridge-with-ros2) · [Foxglove ROS2](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2)
- 성능 비교: [Comparing Performance of ROS-for-Unity Implementations](https://openreview.net/pdf?id=WH3yhsbBjj)
- Object SLAM 계보(맥락): SLAM++ (CVPR 2013), Fusion++ ([arXiv:1808.08378](https://arxiv.org/abs/1808.08378)), QuadricSLAM ([arXiv:1804.04011](https://arxiv.org/abs/1804.04011))
- One Euro filter (jitter): Casiez et al., CHI 2012. [PDF](https://gery.casiez.net/publications/CHI2012-casiez.pdf)
- 선행 보고서: `research/technical-slam-aware-multi-object-sam6d-priority-roadmap.md`

---

## 14. 다음 액션 제안

본 리서치는 조사·설계만 수행했고 코드는 수정하지 않았습니다. 선택해 주세요:

1. **`/bmad-prd`로 진행** — Phase A MVP(object_memory_node + unity_bridge)를 PRD로 구체화(§11.5의 6개 항목을 epic/story로).
2. **`/bmad-architect`로 진행** — §6 노드 그래프·§7 메시지 계약·§8 association·시간 동기화 정책을 정식 solution design으로 확정.
3. **이 보고서 보강** — association 임계값 튜닝 전략, drift sensitivity 실험 설계, Unity prefab/렌더 상세, MarkerArray→커스텀 msg 이행 계획 중 택일 심화.
4. **메모리에 방향 전환 기록** — "Unity 표시 목표로 우선순위가 Object ID/Memory 1순위로 전환됨"을 프로젝트 메모리에 저장.

**질문**: 1순위를 **Object ID/Memory + Unity bridge 묶음 MVP**로 확정하고, **MVP에 "이동 후 재방문 → duplicate=0" 실증을 반드시 포함**하는 데 동의하시나요? 동의하시면 바로 `/bmad-prd`로 Phase A 요구사항을 구체화하겠습니다.
