# _perf_bottleneck_probe_2026_07_06 — 일회성 병목 계측 환경

YOLO-World → semantic → MobileSAM mask → appearance → template 선택 pipeline의
시간 병목 실측용 **격리 폴더**. 원본 프로젝트 파일은 일절 수정하지 않았으며,
**이 폴더만 삭제하면 원상 복구**된다.

근거 보고서: `_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-pipeline-timing-bottleneck-research-2026-07-06.md`

## 구조

```
copied/yolo_ism.py            # 원본 복사 + REPO_ROOT만 실제 sam6d_ws 루트로 고정
copied/yolo_ism_object_n.py   # 원본 복사 + PERF 계측 훅 + --perf-* CLI 옵션
perf/stage_timer.py           # PerfCollector (기본 OFF, --perf-out 지정 시에만 동작)
outputs/<run_id>/             # run_meta.json / per_frame / per_object / stage_summary CSV
outputs/algo_out/             # 복사본의 overlay·결과 CSV (원본 outputs/ 오염 방지용 격리)
reports/quick-dev-result.md   # 구현 결과 보고서
```

## 실행 (sam_yolo env)

```bash
cd /home/ldh9501/temp_ws/CLI_environment/sam6d_ws
/home/ldh9501/miniconda3/envs/sam_yolo/bin/python \
  _perf_bottleneck_probe_2026_07_06/copied/yolo_ism_object_n.py \
  --bag data/ros2_bag/two_table_around \
  --perf-out _perf_bottleneck_probe_2026_07_06/outputs \
  --perf-warmup-frames 3
```

- `--perf-out` 미지정 시 원본 스크립트와 동일 동작(계측 완전 OFF).
- `--perf-run-id` 미지정 시 `<bag>_<YYYYmmdd-HHMMSS>` 자동 생성.
- 시작 로그의 `[perf-probe] imported yolo_ism from: ...` 가 반드시
  `_perf_bottleneck_probe_2026_07_06/copied/yolo_ism.py` 여야 한다.

## 원본과 다른 점 (의도된 차이, 알고리즘 무변경)

1. `copied/yolo_ism.py`: `REPO_ROOT`가 실제 sam6d_ws 루트를 가리키도록 3단계 상위로 수정 (경로 해석만).
2. `copied/yolo_ism_object_n.py`: `--output-root` 기본값이 probe 내부 `outputs/algo_out`
   (원본 `outputs/yolo_ism_object_n/` 결과 덮어쓰기 방지). template feature 캐시는
   원본 캐시(`outputs/yolo_ism_object_n/template_features/`)를 **읽기 재사용**.
3. PERF 훅: threshold/top_k/imgsz/게이트 로직·판정 결과는 전부 원본과 동일.

## CUDA timing 주의

stage 경계마다 `torch.cuda.synchronize()` 수행 → 계측 ON 런의 총 시간은
비계측 런보다 느릴 수 있다(파이프라이닝 제거). stage 간 상대 비중 비교가 목적.
첫 `--perf-warmup-frames`(기본 3) frame은 `is_warmup=1`로 표시되고
`stage_summary.csv` 통계에서 제외된다.

## stage_summary의 stage_name 규칙

- `frame_input/preprocess/yolo_world/box_routing/recognize_total/output_packaging/total_frame`: frame 단위
- `obj:<substage>` (예: `obj:dinov2_cls`): 해당 frame의 **전 객체 합산값**을 frame 단위로 집계
- `percent_of_total_mean` = 해당 stage 평균 / `total_frame` 평균 × 100
- `bottleneck_rank`: `total_frame`·`recognize_total`(obj:* 상위 합집합이라 이중계상) 제외 내림차순 순위

## 해석 시 주의

- `std_ms`는 모집단 표준편차(`statistics.pstdev`).
- `proposal_count_total`은 prompt를 공유하는 객체끼리 동일 박스를 중복 합산할 수 있음.
- 캐시 정책: 원본 template feature 캐시는 **읽기 전용 재사용**. cache miss 또는
  `--rebuild-features` 시 probe 내부 `outputs/template_features/`에만 기록(원본 불변).
- 크래시 시에도 `atexit`으로 `stage_summary.csv`가 기록되고, per_frame/per_object는
  행 단위 flush라 부분 데이터가 생존한다.
