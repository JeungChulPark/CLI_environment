# SLAM + SAM(-6D) + objectmemory 전체 체인 병목 시간 분석

작성일: 2026-07-06 · 대상: SAM_* 4 bag의 오프라인 전체 체인 (이식된 opt2 원본 기준)
체인: `SLAM(ORB-SLAM3 traj) ∥ SAM[ISM 생성(build_ism_inputs_imu) → PEM(run_pem.sh)] → objectmemory 융합(run_object_memory.py)`

## 단계별 실측 단가

| 단계 | 실측값 | 측정 방법 | 구분 |
|---|---|---|---|
| SLAM (ORB-SLAM3) | ≈ bag 재생 시간 (27~82s/bag) | bag duration 실측; SLAM은 센서 속도로 온라인 동작 | **특성 기반 추정** (SLAM 자체 재실행은 안 함) |
| ISM 생성 startup | 36.6s (1회/bag) | 1-frame 런 실측 | 확인됨 — YOLO 모델 10개(프롬프트별) + DINOv2 + MobileSAM + template feature 로딩 |
| ISM 생성 per-frame | **0.12s/frame** | 61-frame vs 1-frame 차분 | 확인됨 (SAM_occlusion 기준; 검출 많은 bag은 번들 IO로 다소 증가) |
| **PEM per-detection** | **10.1s/detection** | 3회 단독 실행 실측 (10.2/9.9/10.2s) | 확인됨 |
| objectmemory 융합 | **≤0.1s/bag** (355 detections 포함) | 4 bag 전체 실행 실측 | 확인됨 |

## bag별 전체 체인 시간 (stride-10, 기존 detection 수 기준)

| bag | SLAM(온라인) | ISM | PEM | objectmemory | 합계 | **PEM 비중** |
|---|---|---|---|---|---|---|
| SAM_circle (133f/107det) | 45s | 53s | 1,081s | 0.1s | ≈20분 | **91.7%** |
| SAM_loop1 (244f/355det) | 82s | 66s | 3,586s | 0.1s | ≈62분 | **96.0%** |
| SAM_loop2 (197f/328det) | 66s | 60s | 3,313s | 0.1s | ≈57분 | **96.3%** |
| SAM_occlusion (80f/103det) | 27s | 46s | 1,040s | 0.1s | ≈19분 | **93.4%** |

## 결론: 전체 체인의 병목은 압도적으로 PEM 배치 구조 (92~96%)

**병목의 정체 (확인됨)**: `run_pem.sh`가 **detection 1건마다 python 프로세스를 새로 기동**하는 구조라서, 10.1s/detection의 대부분이 추론이 아니라 **반복 고정비**다:

- python+torch import: **1.6s** (실측)
- 1.3GB PEM checkpoint 로드: **0.8s** (실측)
- 나머지 ~7s: CUDA context 초기화 + 모델 GPU 전송 + **template 42장 feature 추출**(객체당 동일 계산을 detection마다 반복) + 실제 추론 (분해 미측정, 추정)

비교 근거: 실시간 ROS 노드(`sam6d_multiobject_node.py`)는 같은 PEM을 **startup 1회 로드 + 객체별 template feature 사전계산**으로 상주시키고 detection마다 추론만 수행한다 — 즉 이 고정비는 구조적으로 제거 가능함이 이미 코드베이스 안에서 입증돼 있다.

**부차 확인 사항:**
- 이번에 최적화한 ISM(인식) 단계는 체인에서 ~2~5% 수준 — frame당 0.12s로 이미 SLAM 온라인 속도(30fps 입력의 stride-10 = 3fps 처리 요구)보다 빠름.
- objectmemory 융합은 355 detections 기준 0.1s — 병목 아님(확정).
- SLAM은 온라인 특성상 bag 길이가 하한 — 오프라인 체인에서는 ISM/objectmemory와 병렬 수행 가능.

## 개선 방향 (다음 단계 후보, 미구현)

1. **PEM persistent worker** (최우선): run_pem.sh를 "모델 1회 로드 + manifest 순회" 단일 프로세스로 재작성 (ROS 노드의 `_pem_pose` 패턴 재사용). 기대: 10.1s → 추론+IO만(~1s 내외 추정, 측정 필요) → **체인 전체 ~10배 단축 잠재** (SAM_loop1 62분 → ~7분 수준).
2. PEM template feature 캐시: 객체당 1회 추출해 디스크/메모리 캐시 (ISM template 캐시와 동일 패턴).
3. detection 수 자체 감축은 정확도 트레이드오프라 이번 범위 밖.

## ⚠️ 정정 (2026-07-06 추가 조사): persistent worker는 이미 존재하며, 위 표는 legacy 경로 기준이었다

추가 조사에서 **`tools/run_pem_batch.py`(2026-06-18 작성)가 이미 A안(모델 1회 로드) + B안(template feature 디스크 캐시 `_tem_feat_cache/`)을 모두 구현**하고 있고, 실전 스크립트(`run_imu_*.sh`, `run_e2e.sh`)가 전부 이것을 사용함을 확인했다. 즉 위 표의 PEM 10.1s/detection은 **legacy `run_pem.sh`를 쓸 때만 해당**하며, 기존 rgbd_imu_sdk_bag pem 출력들도 이미 빠른 경로로 생성된 것이다.

**run_pem_batch 실측 (동일 9-bundle manifest):**
- 전체 wall **8.8s** (run_pem.sh 대비 **10.3배**): 모델 로드 ~7s(1회) + inference **0.11s/bundle** + vis/IO
- template feature: 디스크 캐시 히트로 0.0s (B안이 이미 동작 중)
- 실패 1건(frame_000318 Bear, depth 부재)도 동일 재현 — 도구 무관 데이터 edge

**보정된 전체 체인 (실전 경로 기준, SAM_loop1):**

| 단계 | legacy(run_pem.sh) 가정 | **실전(run_pem_batch)** |
|---|---|---|
| SLAM (온라인) | 82s | 82s |
| ISM 생성 | 66s | 66s |
| PEM | 3,586s | **~78s** (로드 7s + 355×~0.2s) |
| objectmemory | 0.1s | 0.1s |
| **체인 합계** | ≈62분 | **≈2.4분** |
| 병목 구조 | PEM 96% | **PEM ~50% ≈ ISM ~45%** (양강), objectmemory 0% |

**보정된 결론**: 실전 도구 기준으로 오프라인 체인은 bag당 ~2\~3분이며, 병목은 PEM 단독이 아니라 **ISM 생성과 PEM이 비슷한 양강 구조**다. legacy `run_pem.sh`는 10배 느린 경로이므로 사용하지 않는 것이 유일한 액션 아이템(문서/스크립트에서 deprecated 표기 권장). ISM 생성의 startup 36.6s(모델 로딩)도 여러 bag 연속 처리 시 프로세스 상주화로 줄일 수 있는 다음 후보.

## [최종] 신규 bag 입력 시나리오 전 구간 실측 (2026-07-06)

"새 bag이 주어졌다"를 가정하고 4개 SAM_* bag을 **bag 파일에서부터 끝까지** 실제 재처리했다
(ISM은 추출 프레임 캐시 없이 bag 직접 디코드, stride 10 전체 frame, 실전 도구 체인:
`build_ism_inputs_imu → run_pem_batch → reorg_imu_outputs → run_object_memory`).
객체 단위 캐시(ISM/PEM template feature, CAD pts)는 새 bag과 무관하게 유효하므로 warm 상태로 측정 — 현실적 조건.

### bag별 단계 실측 (wall time)

| bag | frames | detections | SLAM(온라인)* | **ISM 생성** | **PEM** | reorg | objectmemory | **SAM 체인 합** |
|---|---|---|---|---|---|---|---|---|
| SAM_circle | 133 | 107 | 45s | 36.6s | 18.7s | 0.0s | 0.0s | **55.3s** |
| SAM_loop1 | 244 | 355 (+19 depth-fail) | 82s | 53.6s | 48.2s | 0.1s | 0.1s | **102.0s** |
| SAM_loop2 | 197 | 328 (+27) | 66s | 49.3s | 45.5s | 0.1s | 0.1s | **95.0s** |
| SAM_occlusion | 80 | 103 (+8) | 27s | 29.0s | 17.6s | 0.0s | 0.0s | **46.6s** |

\* SLAM은 온라인 특성상 bag 재생 시간을 하한으로 병기 (이번에 재실행하지 않음). bag은 이미 녹화 파일이므로 SLAM과 SAM 체인은 병렬 실행 가능.

### 결론

1. **새 bag 1개의 총 처리 시간 ≈ bag 길이의 1.2~1.7배** (SAM 체인 47~102s). SLAM과 병렬이면 전체 소요 = max(SLAM, SAM 체인) ≈ **1~2분/bag**.
2. **병목 1위 = ISM 생성 (체인의 52~62%)** — 그중 지배 요인은 frame 처리(0.1s/frame)가 아니라 **bag당 고정 startup ~30초**(YOLO 모델 10개 + DINOv2 + MobileSAM + template 로딩). 여러 bag 연속 처리 시 프로세스 상주화로 bag당 ~30s 절감 가능(=ISM의 60~70%).
3. **병목 2위 = PEM (36~46%)** — 분해: 모델 로드 ~7s(bag당 1회) + 순수 추론 **0.06~0.07s/detection** + vis/JSON IO ~0.05s/detection. wall의 ~1/3이 vis 렌더링·IO — `--no-vis` 옵션이 없어서 항상 지불 중.
4. **reorg·objectmemory ≈ 0%** (각 ≤0.1s) — 융합은 병목이 아님을 재확인.
5. **재현성 검증 덤**: 이식된 파이프라인의 detection 수가 6월 산출과 정확히 일치(107/355/328/103) — 새 코드로 재생성해도 인식 결과 규모 동일. PEM 실패분(depth-빈 mask edge)도 동일 패턴. objectmemory 객체 수는 일부 상이(loop1 14→10 등) — PEM의 pointcloud 샘플링 비결정성(np RNG 미시드, 원본부터 존재)에 따른 pose 미세 차이가 association에 전파된 것으로, 이식과 무관.

### 남은 개선 카드 (우선순위순, 미구현)

| 카드 | 절감 (bag당) | 난이도 |
|---|---|---|
| ISM 다중 bag 상주화 (bags 목록 순회) | ~30s (ISM의 60%) | 중 (build_ism_inputs_imu 루프화) |
| PEM `--no-vis` 옵션 | ~5~17s (PEM의 ~35%) | 하 (몇 줄) |
| run_pem.sh deprecated 표기 | 실수 방지 | 하 (1줄) |

## 다음 액션 제안

1. **PEM `--no-vis` + run_pem.sh deprecated 표기** — 저난이도 2건 일괄 처리.
2. **ISM 다중 bag 상주화** — 여러 bag 배치가 잦다면 가장 큰 남은 절감.
3. **여기서 종료** — 새 bag당 1~2분이면 실용상 충분.

어느 방향으로 진행할까요?
