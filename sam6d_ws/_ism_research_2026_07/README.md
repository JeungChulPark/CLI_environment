# `_ism_research_2026_07` — ISM 조사용 격리 폴더 (2026-07-20)

SAM-6D ISM 파이프라인 조사(지연시간 / 정확도 / 6데이터 전수관찰)에서 생성한
**분석 전용 산출물**을 한곳에 모아둔 폴더다.

## 원상복구

```bash
rm -rf sam6d_ws/_ism_research_2026_07
```

이 폴더 삭제만으로 완전히 원상복구된다.

- 운영 코드(`yolo_ism.py`, `yolo_ism_object_n.py`, `tools/`, `src/sam6d_ros/`, `configs/`)를
  **수정한 적 없다.**
- 운영 코드/launch/config 어느 것도 이 폴더를 참조하지 않는다 (2026-07-20 grep 확인).
- git untracked 상태다.

## 구성

| 하위 폴더 | 크기 | 조사 | 내용 |
|---|---:|---|---|
| `ism_methodology_analysis/` | 124K | 지연시간 방법론 | 벤치마크 스크립트 4개 + 타이밍 JSON/CSV |
| `ism_accuracy_analysis/` | 32M | FP/FN 근본원인 | probes/experiments, 라벨 338건, DINOv2 특징 캐시, crop |
| `ism_accuracy_observation/` | 204M | 6데이터 2,596프레임 전수관찰 | 박스 22,436·쌍 224,360 덤프, HSV, contact sheet, 라벨 504건 |

## 삭제 전 반드시 보존할 것 (재생성 불가)

아래는 **재실행해도 동일하게 복구되지 않는다.** contact sheet를 보고 붙인 라벨이기 때문이다.

```
ism_accuracy_observation/labels/box_labels_merged.csv     # 504건
ism_accuracy_observation/labels/human_review_queue.csv    #  79건 (사람 검수 미완)
ism_accuracy_analysis/labels/box_labels.csv               # 338건 (다른 데이터셋 계열)
```

이 라벨들이 HSV / class margin / Top-3 결론 전부의 근거다.
후속 실험(class competition, HSV fusion)을 계속할 계획이면 최소한 이 3개는 남긴다.

디스크만 회수하려면 무거운 중간 산출물만 지운다 (약 198M, 프로브 재실행으로 재생성 가능):

```bash
rm -rf ism_accuracy_observation/{frame_dumps,hsv_features,contact_sheets} \
       ism_accuracy_analysis/{features,datasets}
```

## 보고서 (이 폴더 밖, 삭제해도 남음)

```
_bmad_output_for_slam/planning-artifacts/research/
  technical-sam6d-ism-discriminability-latency-methodology-research-2026-07-20.md
  technical-sam6d-ism-accuracy-fp-fn-color-discriminability-research-2026-07-20.md
  technical-sam6d-six-dataset-top3-hsv-yolo-proposal-observation-research-2026-07-20.md
```

## 경로 규칙

스크립트는 `sam6d_ws`를 다음처럼 계산한다 (이 폴더 아래로 옮기면서 한 단계 추가됨).

```python
REPO = os.path.dirname(os.path.dirname(ROOT))     # ROOT = 각 조사 폴더
```

폴더 깊이를 다시 바꾸면 `probes/`·`scripts/` 상단의 `REPO` 정의도 함께 고쳐야 한다.
