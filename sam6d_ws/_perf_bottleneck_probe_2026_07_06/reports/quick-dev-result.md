# quick-dev 결과 — 병목 계측 격리 환경 (_perf_bottleneck_probe_2026_07_06)

작성일: 2026-07-06 · 근거: `_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-pipeline-timing-bottleneck-research-2026-07-06.md`

## 1. 구현 요약

원본 파일을 일절 수정하지 않고, `yolo_ism.py`/`yolo_ism_object_n.py` 복사본에만
timing instrumentation을 추가한 격리 계측 환경. `--perf-out` 미지정 시 원본과
동일 동작(계측 완전 OFF, CSV 미생성)임을 실행으로 검증했다.

## 2. 격리 폴더 구조

```
_perf_bottleneck_probe_2026_07_06/
  README.md                     # 사용법·해석 주의
  copied/yolo_ism.py            # 복사본 (REPO_ROOT 경로 상수만 수정)
  copied/yolo_ism_object_n.py   # 복사본 (PERF 훅 + --perf-* CLI)
  perf/__init__.py
  perf/stage_timer.py           # PerfCollector (신규)
  outputs/<run_id>/             # perf CSV/JSON (smoke_test, smoke_test2 포함)
  outputs/algo_out/             # 복사본의 overlay/CSV (원본 outputs/ 오염 방지)
  reports/quick-dev-result.md   # 본 문서
```

## 3~5. 파일 목록

- **복사한 원본**: `yolo_ism.py`, `yolo_ism_object_n.py` → `copied/`
- **수정한 복사본**: `copied/yolo_ism.py`(REPO_ROOT 3단계 상향 1곳),
  `copied/yolo_ism_object_n.py`(경로/import 가드, PERF 훅, CLI 옵션, 캐시 쓰기 격리)
- **신규 계측 파일**: `perf/stage_timer.py`, `perf/__init__.py`

## 6. 추가된 CLI 옵션

`--perf-out` (계측 활성+출력 루트) · `--perf-warmup-frames` (기본 3) · `--perf-run-id` (기본 `<bag>_<ts>`)
그 외 복사본 한정 변경: `--output-root` 기본값이 probe 내부 `outputs/algo_out`
(원본 `outputs/yolo_ism_object_n/` 결과 덮어쓰기 방지).

## 7. perf output 구조

`outputs/<run_id>/{run_meta.json, per_frame_timing.csv, per_object_timing.csv, stage_summary.csv}`

## 8. CSV schema 요약

- **per_frame** (21열): run_id, frame_index, stamp, is_warmup, image_width/height,
  object_prompt_count, detection_count, proposal_count_total, template_count_total,
  stage_{frame_input,preprocess,yolo_world,box_routing,recognize_total,output_packaging}_ms,
  total_until_template_selection_ms, total_frame_ms, gpu_memory_{allocated,reserved}_mb, notes
- **per_object** (24열): …object_index/name, detection/proposal/template count,
  normalize_rgb/crop/dinov2_cls/semantic_score/dinov2_patch/mask/appearance_score/recognize_total ms,
  dinov2_forward_count, selected_template_id, best_sem, masked_appe, decision, accepted, notes
- **stage_summary** (12열): warm-up 제외, mean/median/p90/p95/max/min/std,
  percent_of_total_mean, bottleneck_rank (`total_frame`·`recognize_total`은 이중계상 방지로 랭킹 제외)
- decision 값: `no_proposal / below_semantic / mask_failed / below_appearance / detected / error`
  (mask None은 알고리즘상 진행되므로 notes=`mask_none`으로 기록)

## 9. Smoke test command

```bash
cd /home/ldh9501/temp_ws/CLI_environment/sam6d_ws
/home/ldh9501/miniconda3/envs/sam_yolo/bin/python \
  _perf_bottleneck_probe_2026_07_06/copied/yolo_ism_object_n.py \
  --bag data/ros2_bag/two_table_around --max-frames 6 \
  --perf-out _perf_bottleneck_probe_2026_07_06/outputs \
  --perf-warmup-frames 3 --perf-run-id smoke_test2
```

## 10. Smoke test 결과 (통과)

- 6 frames / 10 objects / 60 object rows, 4개 산출 파일 전부 생성, header가 요구 schema와 일치
- 알고리즘 결과 원본 의미 보존: accepted 23 obj-frames (계측 ON/OFF 및 패치 전후 동일)
- warm-up 동작 확인: frame0 total 1509ms(warm-up) vs steady 134~225ms; summary는 3 frame(steady)만 집계
- perf OFF 런: perf 디렉터리/CSV 미생성 확인(디렉터리 diff empty)
- **참고(스모크 수치, n=3, 확정 아님)**: obj:mask(MobileSAM) 43.9% > obj:dinov2_cls 16.1%
  > obj:appearance_score 10.8% > obj:dinov2_patch 10.0% > yolo_world 6.9% — 병목 확정은 V1~V8 실행 후

## 11. 실제 import된 yolo_ism.__file__

`/home/ldh9501/temp_ws/CLI_environment/sam6d_ws/_perf_bottleneck_probe_2026_07_06/copied/yolo_ism.py`
(startup 로그 + run_meta.json `actual_imported_yolo_ism_file` + assert 하드 가드로 3중 확인)

## 12. 원본 비수정 검증

작업 전/후 md5 checksum 비교 — 6개 보호 파일 전부 OK:
`yolo_ism.py`, `yolo_ism_object_n.py`, `sam6d_multiobject_node.py`,
`yoloworld_sidecar.py`, `configs/yolo_ism_objects.yaml`, `detector.py`
(git status의 `M yolo_ism_object_n.py`/`M configs/yolo_ism_objects.yaml`은 본 작업 이전부터
존재하던 사용자 변경분 — 작업 전 스냅샷 checksum과 동일함으로 입증)

## Adversarial review 결과

17건 발견 → **patch 12건 적용**(캐시 쓰기 probe 내부 격리, atexit finalize(크래시 생존),
import assert 가드, 조기 run_meta, stderr 로그, rank 이중계상 제거, 무귀속 구간 편입,
env 버전 메타, 상수 hoisting 등) / reject 3건(스키마 필수 컬럼, 도달 불가 경로) /
문서화 2건(README "해석 시 주의" 절).

## 13. 아직 하지 않은 것

ROS2 node 계측 · sidecar 내부 계측 · PEM 계측 · Validation Plan V1~V8 실행 · 병목 확정 분석

## 14. 다음 추천 단계

research 보고서의 Validation Plan(V1 객체 1/5/10 스윕 등)을 이 probe로 실행한 뒤,
생성된 CSV를 근거로 `/bmad-performance-review`(또는 분석 요청)로 병목을 확정한다.
분석 종료 후 `_perf_bottleneck_probe_2026_07_06/` 폴더 삭제만으로 원상 복구된다.
