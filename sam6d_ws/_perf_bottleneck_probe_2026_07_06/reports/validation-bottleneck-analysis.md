# 병목 확정 분석 — Validation V1~V8 실측 결과

작성일: 2026-07-06 · 데이터: `outputs/v1_obj1 ~ v8` (bag `two_table_around`, 30 frames/런, warm-up 3 제외 = steady 27 frames, RTX PRO 6000 Blackwell, `sam_yolo` env)
계측 방식: stage 경계 `torch.cuda.synchronize` 포함 — 절대값은 비계측 실행보다 다소 크며, **상대 비중과 스케일링이 판단 기준**이다.

## 결론: 병목 확정 순위 (10객체 기준, steady 27f 평균)

| 순위 | stage | mean ms | total 대비 | 단가 (실측 유도) |
|---|---|---|---|---|
| **1** | **obj:mask (MobileSAM)** | **59.3** | **35.9%** | ~15.6 ms/호출 (3.8회/frame) |
| **2** | **obj:dinov2_cls** | **43.1** | **26.1%** | ~4.6 ms/forward (~9.3회/frame) |
| 3 | obj:dinov2_patch (중복 forward) | 14.2 | 8.6% | ~3.7 ms/forward (3.8회/frame) |
| 4 | yolo_world | 12.9 | 7.8% | frame당 1회 상수 |
| 5 | obj:appearance_score | 9.6 | 5.8% | CPU matmul |
| 6 | obj:normalize_rgb (중복) | 6.2 | 3.8% | 객체당 ~0.6ms × 10 |
| 7 | output_packaging (Disk IO) | 6.0 | 3.6% | V5로 검증 |
| — | total_frame | 165.0 | 100% | ≈ 6.1 FPS (계측 포함) |

**research 보고서의 "측정 전 추정"과의 차이**: 추정 1위였던 DINOv2 반복 forward(cls+patch 합산 57.3ms)와 실측 1위 MobileSAM(59.3ms)이 사실상 동률 1위 그룹. YOLO-World는 예상대로 상수·소액(7.8%)이었고, template 선택 자체(semantic matmul 1.4ms, 0.8%)는 병목이 아님이 확정됨.

## 조건별 실측 (mean ms, steady 27 frames)

| stage | obj1 | obj5 | obj10 | topk1 | imgsz640 | nosave | 판정 |
|---|---|---|---|---|---|---|---|
| total_frame | 37.4 | 122.8 | 165.0 | 149.5 | 167.3 | 151.3 | 객체 수 선형 (~14ms/객체) |
| obj:mask | 8.0 | 42.8 | 59.3 | 57.4 | 62.3 | 60.8 | semantic 통과 객체 수 선형; top_k 무관 |
| obj:dinov2_cls | 5.5 | 35.8 | 43.1 | **24.8** | 45.2 | 40.5 | forward 수 선형; top_k=1이면 −42% |
| obj:dinov2_patch | 1.8 | 9.4 | 14.2 | 12.9 | 14.4 | 13.7 | 통과 객체 수 선형 |
| yolo_world | 11.7 | 11.7 | 12.9 | 12.6 | **10.0** | 11.3 | 객체 수 무관 상수 (확정) |
| output_packaging | 1.8 | 4.4 | 6.0 | 5.4 | 6.0 | **0.1** | V5: 저장 off 시 소멸 (확정) |
| dinov2_fwd/frame | 1.7 | 11.0 | 13.1 | 8.9 | 14.0 | 13.1 | 객체×proposal 스케일 (확정) |
| mask_calls/frame | 0.5 | 2.7 | 3.8 | 3.6 | 4.0 | 3.8 | — |

## 검증 항목별 판정

- **V1 (객체 스케일링)**: total 37→123→165ms. recognize 내부(mask+dinov2)가 증가분의 ~90%를 차지. **objects 10개의 지배 비용 = per-object GPU 반복 호출** — 확정.
- **V2 (top_k 1 vs 3)**: dinov2_cls 43.1→24.8ms(−42%), mask는 불변(top-1 선택 후 1회라서). proposal 수가 cls forward 비용의 직접 변수임을 확정.
- **V3 (imgsz 640 vs 960)**: yolo_world 12.9→10.0ms — 픽셀 2.25× 차이 대비 미미. **이 GPU에서는 imgsz가 주요 변수 아님** (sidecar 640 불일치의 시간 영향도 ~3ms 수준으로 추정됨).
- **V4 (cache cold)**: `--rebuild-features`가 probe 내부 `outputs/template_features/`에 20개 파일 생성(원본 캐시 불변 — 격리 계약 검증). startup 재빌드 오버헤드는 wall +7s 수준(21s vs 14s, frame 수 차이 감안 시 근사치). frame 시간에는 영향 없음(가설 확인, 단 n=3).
- **V5 (저장 off)**: output_packaging 6.0→0.1ms, total −13.7ms(−8.3%). Disk IO는 minor 병목으로 확정 (research 후보 4는 하위 순위).
- **V6 (warm-up)**: 첫 frame ~1.5s(cuBLAS/커널 초기화) vs steady 134~225ms — warm-up 제외 필수 재확인.
- **V7 (ROS)**: 미실행 — ROS 노드 계측은 이번 범위 밖.
- **V8 (계측 오버헤드)**: perf-off wall 13s vs perf-on 14s (동일 30f) — 오버헤드 대략 ≤1s/30frames(≈7%, 로딩 시간 변동 포함 근사). 상대 비중 해석에는 영향 없음.

## 개선 여지 정량화 (실측 기반, 구현은 별도 단계)

| 개선안 | 근거 stage | 예상 절감 (frame당) |
|---|---|---|
| ① dinov2_patch 중복 제거 (cls forward에서 patch 동시 반환) | obj:dinov2_patch 14.2ms | **~14ms (−8.6%)** — forward는 이미 patch를 계산 후 폐기 중 |
| ② proposal cls를 box 단위 1회 계산 + 객체 간 재사용 / 배치화 | obj:dinov2_cls 43.1ms | 최대 ~20-30ms (proposal 중복·batch=1 launch 절감) |
| ③ MobileSAM frame당 1회 호출 (bboxes 리스트로 일괄) | obj:mask 59.3ms | 최대 ~44ms (image encoding 재사용 시 15.6ms×(3.8−1)) |
| ④ normalize_rgb frame당 1회 hoisting | obj:normalize_rgb 6.2ms | ~5.6ms |
| ⑤ 저장 off / 비동기화 | output_packaging 6.0ms | ~6ms |

①+④는 저위험 즉효(합 ~20ms, −12%), ②③은 구조 변경 필요하나 잠재 최대 −45%.

## 한계

- 단일 bag(two_table_around)·단일 GPU·27 steady frames — bag 간 분산 미검증.
- synchronize 계측 포함 절대값. mask/dinov2의 "GPU 연산 vs launch 오버헤드" 분해는 미수행(torch profiler 필요).
- V4는 n=3, V8은 wall time 근사.

## [추가] 개선안 ①+④ before/after 실측 — SAM_* 4 bag (2026-07-06)

**데이터**: `outputs/rgbd_imu_sdk_bag/{SAM_circle, SAM_loop1, SAM_loop2, SAM_occlusion}/input/frame_*/rgb.png`
(640×480, 기 추출 stride-10 프레임을 probe 내부 심링크 farm으로 연결, 원본 무변경).
런당 33 frames(warm-up 3 제외 = steady 30), 10객체 원본 config. before = `copied/yolo_ism_object_n_baseline.py`(개선 전 보존본), after = 개선 적용본.

| bag | total before→after | 개선율 | mask | dinov2_cls | dinov2_patch | normalize_rgb(obj) |
|---|---|---|---|---|---|---|
| SAM_circle | 83.7 → 77.1 ms | **−7.9%** | 19.2→18.6 | 25.0→25.3 | 4.6→**0** | 7.2→**0** |
| SAM_loop1 | 127.7 → 92.1 ms | **−27.9%** | 35.1→29.3 | 34.2→29.6 | 7.6→**0** | 10.9→**0** |
| SAM_loop2 | 134.5 → 134.4 ms | −0.1% | 47.2→46.1 | 40.1→44.1 | 10.6→**0** | 4.5→**0** |
| SAM_occlusion | 70.0 → 68.2 ms | −2.6% | 18.5→17.9 | 19.7→20.3 | 4.2→**0** | 2.8→**0** |
| **4-bag 평균** | **104.0 → 92.9 ms** | **−10.6%** | | | | |

**정확도/판정 동일성 (핵심 검증)**: 4 bag × 330 object-rows = **1,320 rows 전수 비교 —
decision·accepted·selected_template_id 차이 0건, |Δbest_sem|·|Δmasked_appe| 최대 0.0000.**
개선이 알고리즘 출력을 전혀 바꾸지 않음이 실측으로 입증됨.

**세부 해석**:
- `dinov2_patch`(중복 forward)와 `obj:normalize_rgb`(객체별 재계산)는 전 bag에서 **완전 소멸**(설계 그대로).
- normalize 1회분은 `preprocess`로 이동 (0.17 → ~2.0ms/frame): 객체 10회분 → 1회분.
- **SAM_loop2 상쇄 원인**: 이 bag은 proposal이 많아(fwd 12.9회/frame) `want_patch=True`가 proposal마다
  patch 정규화+GPU→CPU 전송(~[256,384])을 추가하는 비용이 cls stage에 얹혀(40.1→44.1ms)
  patch-forward 절감분을 상쇄. **개선 ①은 "proposal당 전송 추가 vs 통과객체당 forward 제거" trade-off**로,
  proposal 대비 semantic-통과 비율이 높을수록 이득(loop1 −28%)이고 낮으면 중립(loop2 −0.1%).
- **추가 개선 여지**: proposal patch를 GPU에 둔 채 best 승자만 마지막에 1회 `.cpu()` 전송하면
  loop2류의 상쇄를 제거 가능 (dinov2_forward 래퍼 수정 필요, 미구현).
- 병목 순위의 bag 불변성 확인: 4 bag 전부 mask·dinov2_cls가 1·2위 (two_table_around와 동일 구조).

## [추가 2] 개선 전체 단계 실측 — ①보강(Step A) + opt2 2-pass 배치(Step B) (2026-07-06)

동일 조건(SAM_* 4 bag, steady 30 frames, 10객체 원본 config)에서 4단계 비교.

| bag | before | ①④ | ①보강(Step A) | **opt2(Step B)** | opt2 dinov2_batch | opt2 mask_batch |
|---|---|---|---|---|---|---|
| SAM_circle | 83.7 | 77.1 | 71.0 | **55.5** | 8.4 | 12.1 |
| SAM_loop1 | 127.7 | 92.1 | 101.9 | **54.8** | 8.2 | 11.0 |
| SAM_loop2 | 134.5 | 134.4 | 129.7 | **59.2** | 9.3 | 16.4 |
| SAM_occlusion | 70.0 | 68.2 | 59.9 | **46.4** | 4.5 | 9.0 |
| **4-bag 평균 (ms)** | **104.0** | 92.9 | 90.6 | **54.0** | | |

**최종 결과: before 대비 −48.1% (104.0 → 54.0 ms, 9.6 → 18.5 FPS 상당)**

**Step A (① 보강, patch GPU 유지·승자만 전송)**: 평균 92.9→90.6ms. 승자 전송 비용 0.1~0.4ms로 미미, SAM_loop2 상쇄 일부 해소. 판정 동일성 Δ=0 유지.

**Step B (opt2: 2-pass 재구성)** — `copied/yolo_ism_object_n_opt2.py`:
- **DINOv2 배치화**: frame 내 전 객체·전 proposal crop을 `torch.stack`으로 묶어 **forward 1회** (`dinov2_forward_batch`, copied/yolo_ism.py). per-object 반복 43+14ms → **frame당 8~9ms** (batch=1 launch 오버헤드 제거 입증).
- **MobileSAM 일괄 호출**: semantic 통과 bbox 전부를 `bboxes=[...]`로 **호출 1회** (`segment_boxes`). 객체별 반복 47~59ms → **frame당 9~16ms** (image encoding 1회 재사용 입증).
- **mask 동일성 사전 검증**: 실제 3 frames × 12 boxes에서 개별 vs 일괄 호출 **IoU=1.0000, 픽셀 면적까지 동일**.
- **판정 동일성 (opt2 vs before)**: 4 bag × 330 = 1,320 rows 전수 — decision·accepted·selected_template_id 차이 0, |Δbest_sem|·|Δmasked_appe| 최대 **0.000000** (배치 forward의 수치까지 결정적으로 동일; masked_appe는 mask의 함수이므로 mask 동일성의 추가 증거).

**opt2 이후의 잔여 구조 (SAM_loop2 기준)**: total 59.2ms = yolo_world 12.8 + dinov2_batch 9.3 + mask_batch 16.4 + 나머지(입출력/CPU) ~20. **per-object 반복 병목이 제거되어 YOLO-World와 mask 디코딩이 새로운 상위 비용**이 됨 — 객체 수 증가에 대한 민감도가 구조적으로 크게 낮아짐 (배치 크기만 증가).

**opt2 계측 참고**: 배치 비용은 per-object로 귀속 불가라 frame stage `dinov2_batch`/`mask_batch`로 기록되고, per-object row의 timing 필드는 0 + note=`opt2_batched` (counts/score/decision은 그대로 기록되어 identity 검증에 사용).

## [추가 3] opt2 원본 이식 + objectmemory/SLAM 통합 검증 (2026-07-06, 사용자 승인 하에 원본 수정)

### 이식 내용

- **`yolo_ism.py`**: `dinov2_forward`에 `patch_stays_on_device` kwarg 추가(기본값 = 기존 동작), `dinov2_forward_batch()`·`segment_boxes()` 신규 — **기존 호출부 동작 불변인 추가만**.
- **`yolo_ism_object_n.py`**: `recognize()`는 시그니처 하위호환 유지(ROS 노드 `sam6d_multiobject_node.py:217`·`build_ism_inputs_imu.py:252`가 위치 인자로 호출) + 내부에 개선 ①(patch 재사용·GPU 유지) 적용, `norm_full=None` kwarg 추가; `recognize_frame()`(opt2 2-pass) 신규; `main()`은 recognize_frame 사용으로 전환 + normalize hoist.
- 이식 전 원본은 scratchpad snapshot으로 보존, probe의 `copied/yolo_ism_object_n_baseline.py`가 이식 전 로직의 영구 사본.

### 검증 3종 (전부 통과)

| Test | 내용 | 결과 |
|---|---|---|
| **A. 파이프라인 동일성** | 이식 원본 vs 이식 전 로직(baseline), SAM_loop1 33 frames × 10객체, 네이티브 results CSV 전 필드 비교 | **330 rows 필드 차이 0, multiobject_summary byte 동일** |
| **B. objectmemory 상류 입력 동일성** | `build_ism_inputs_imu.py`(내부에서 이식된 `o_n.recognize` 호출)를 pre/post-port로 실행, SAM_occlusion 6 frames | **detection JSON(RLE seg)·camera.json·manifest 전부 byte 동일** (9 bundles) |
| **C. objectmemory+SLAM 통합** | 이식 원본 산출 detection → PEM(8/9 성공) → `run_object_memory.py`를 SLAM `CameraTrajectory.txt`·bag과 함께 실행 | **정상 동작: objects 2(Bear, milk), frames_fused 4/4, detections 8 융합, T_map_obj 산출** |

- PEM 1건 실패(frame_000318 Bear)는 mask 영역 유효 depth 부재로 인한 **PEM 자체의 기존 edge case** — 입력 detection이 pre/post 동일이므로 이식과 무관.
- objectmemory는 yolo_ism 계열을 직접 실행하지 않으므로(하류 소비자), Test B의 입력 byte-동일성 + Test C의 체인 완주가 "이식 후에도 변경 없이 동작"의 완결 증거.

## 다음 액션 제안

1. ~~병목 확정~~ · ~~개선 3단계(−48.1%)~~ · ~~원본 이식 + objectmemory/SLAM 검증~~ → **전부 완료**.
2. **[권장] sam6d_ws repo에 이식 커밋** — `yolo_ism.py`/`yolo_ism_object_n.py` 변경분을 커밋해 고정 (기존 사용자 WIP와 분리 커밋 권장).
3. **ROS 실시간 노드(`sam6d_multiobject_node.py`)에 recognize_frame 적용** — 콜백도 2-pass 배치화 가능 (현재는 ① 개선만 자동 적용됨 — recognize 내부 개선이라 노드 재시작만으로 반영).
4. **probe 폴더 정리** — 이식이 완료됐으므로 `_perf_bottleneck_probe_2026_07_06/`은 보고서·CSV 보존 목적 외 삭제 가능.

어느 방향으로 진행할까요?
