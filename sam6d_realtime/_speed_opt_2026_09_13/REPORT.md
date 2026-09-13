# SAM-6D 속도 최적화 — 실측 보고서

측정 2026-09-13 · **RTX 3080 Ti Laptop GPU** (58 SM, 16 GB, cc 8.6) · i9-12900HK · WSL2
torch 2.7.1 / CUDA 12.9 · conda env `sam6d` (이번에 구축)

표기: **[M]** 이 기계 실측 · **[M-5090]** 다른 기계(RTX 5090 Laptop) 기존 실측 · **[E]** 추정

---

## 0. 요약

| | 값 |
|---|---|
| 기준선 | 353 ms · **2.83 FPS** [M-5090] |
| 안전한 파라미터 튜닝만 | ~202 ms · **4.9 FPS** [M 조합] |
| 구조 변경까지 | **~86 ms · 11.6 FPS** [E, 부분 실측 조합] |
| 통합 실행 검증 | ❌ **불가** — CAD 비공개로 15종 템플릿을 만들 수 없음 |

**10 FPS는 파라미터 튜닝으로 도달할 수 없고, 구조 변경(마스크 재사용 + PEM 예산제)으로만 간다.**

---

## 1. 측정 가능 범위

이 노트북에 **CAD(.ply)가 없다**(`ASSETS_REQUIRED.md`: "비공개 · 원본 PC 복사"). CAD 없이는
15종 객체의 렌더 템플릿·PEM 템플릿 캐시를 만들 수 없어 **파이프라인 전체를 실제 영상으로
돌릴 수 없다.** 입력 bag(`Dataset/260826_etri_eightcircle_dark/SAM/`, 13 GB)은 있다.

그래서 두 가지로 나눠 측정했다.

1. **합성 벤치** — PEM/ISM 의 비용은 텐서 shape 에 지배되고 객체 정체성과 무관하다.
   실제 config·실제 입력 shape 으로 모델을 세우고 합성 텐서를 흘려 **모듈별 지연**을 쟀다.
2. **정확도** — 공개 저장소 `JeungChulPark/sam6d-object-memory` 에서 **Milk 객체의
   42-view 템플릿**을 확보해, 알려진 R·t 로 합성 관측을 만들어 PEM 이 그 포즈를
   복원하는지 쟀다(self-consistency).

교차 검증: 합성 PEM B=4 FP32 가 **254.6 ms [M]**, 기존 실측 PEM 이 **277 ms [M-5090]** —
다른 기계인데 근접한다. 합성 벤치가 실제 워크로드를 잘 재현한다는 방증이다.

### 로컬에 템플릿이 없던 진짜 이유

Milk 템플릿은 **로컬에도 절반이 있었다**. `rgb_*.png` 42장과 `mask_*.png` 42장은 있는데
`xyz_*.npy` 42개(66 MB)만 없었다. 원인은 **우리 `.gitignore` 17행의 `*.npy`** — 렌더
결과물을 제외하려던 규칙이 템플릿의 3D 좌표 맵까지 삼켰다. 저장소에서 받아 복구했다.

---

## 2. 파이프라인과 기준선

`realtime/sam6d_core.py::Sam6DCore.process()` 실제 경로:

```
RGB + aligned Depth + K
 ├─ YOLO-World (yolov8m-worldv2)                           7 ms  [M-5090]
 ├─ ISM  MobileSAM(프레임당 1회) + DINOv2 ViT-S/14@224(프레임당 1회)
 │        + 템플릿 매칭(42장, GPU 캐시) + HSV/NMS 게이트     87 ms  [M-5090]
 └─ PEM  ViT-Base + geo_embed + coarse(6000→300) + fine(2048)
                                                          277 ms  [M-5090]
                                                    총 353 ms = 2.83 FPS
```

`config/base.yaml`: `coarse_npoint 196`, `fine_npoint 2048`, `vit_type vit_base`,
`nproposal1 6000`, `nproposal2 300`, `img_size 224`, `n_template_view 42`.

### 이미 되어 있던 것 (중복 작업 방지)

| 요청서 항목 | 상태 |
|---|---|
| §11 `torch.inference_mode()` | ✅ `sam6d_core.py:296` |
| §7-C 템플릿 GPU 캐시 | ✅ 기동 시 1회 로드 |
| §12 검출 배치 처리 | ✅ 프레임당 PEM 1회, 전 검출 concat |
| §2 CUDA sync 타이밍 | ✅ `_sync()` 로 stage 계측 |
| §4 프레임 스킵 | ✅ 큐 깊이 1 + busy 플래그 |
| **ISM 배치화** | ✅ `yolo_ism_object_n.py:579,580` 프레임당 1회 |

> 정정: 처음에 "`segment_boxes`(배치)가 호출되지 않는다"고 보고했으나 **틀렸다.**
> 실시간 경로 `recognize_frame_auto` → `assign_frame_relative` 가 쓴다. 박스별
> `segment_box` 를 쓰는 `recognize_multi` 는 실시간이 타지 않는 다른 경로다.

---

## 3. PEM 내부 분해 [M] — 요청서 전제 2개가 뒤집힘

B=4 검출(실제 프레임 상황):

| 모듈 | FP32 | FP16 | 비중 |
|---|---|---|---|
| feature_extraction (**ViT-Base**) | 21.83 | 12.05 | **7~9%** |
| geo_embedding | 20.13 | 17.75 | 8~10% |
| **coarse_point_matching** | **95.05** | **67.05** | **38%** |
| **fine_point_matching** | **86.16** | **60.18** | **35%** |
| unattributed | 23.29 | 19.91 | 10% |
| **TOTAL** | **246.5** | **177.0** | |

**① §8 "backbone이 병목" → 아니다.** ViT-Base 는 7~9%. 0 ms 로 만들어도 12 ms 절감이다.
backbone 교체(FastViT/EfficientViT)도, backbone 우선 TensorRT(§17)도 **하지 말 것.**

**② probe README 의 "후보 300개가 병목" → 아니다.** `nproposal2` 를 300→32 로 **9배
줄여도 4%**(잡음 수준). coarse 가 줄어드는 만큼 fine 이 늘어 상쇄된다.

---

## 4. 파라미터 스윕 [M] (B=4)

| 파라미터 | 변경 | TOTAL | 속도 효과 |
|---|---|---|---|
| 정밀도 | FP32 → BF16 | 246 → 177 | −28% |
| `coarse_npoint` | 196 → 96 | 153 → 114 | −26% |
| `fine_npoint` | 2048 → 1024 | 156 → 144 | −8% |
| `nproposal2` | 300 → 64 | 157 → 150 | −4% (잡음) |

### 정확도까지 본 트레이드오프 [M] — 속도만 보면 틀린다

Milk 객체, 20 trial, 관측 뷰를 템플릿에서 제외(`--leave-out`):

| 설정 | PEM B=1 | PEM B=4 | rot median | **rot p90** | **rot<5°** | tr<20mm |
|---|---|---|---|---|---|---|
| FP32 baseline | 75.9 | 254.6 | 0.24° | 0.62° | 19/20 | 20/20 |
| BF16 | 63.6 | 158.8 | 1.31° | 3.59° | 19/20 | 20/20 |
| BF16 + **cn=96** | 56.3 | 112.4 | 2.32° | **93.17°** | **17/20** | 19/20 ⚠ |
| **BF16 + fn=1024** | **55.1** | **144.4** | **0.88°** | **2.55°** | **20/20** | **20/20** ★ |
| BF16 + cn96 + fn1024 | 46.7 | 87.2 | 1.18° | **91.43°** | **17/20** | 19/20 ⚠ |
| BF16 + cn64 + fn1024 | 45.8 | 85.0 | 1.73° | **90.42°** | **17/20** | 18/20 ⚠ |

🔴 **`coarse_npoint` 축소는 포즈를 깨뜨린다.** 줄인 세 설정이 **모두 17/20** 으로 떨어지고
p90 회전 오차가 **90° 이상**으로 폭발한다. 세 설정에서 일관되게 재현되므로 우연이 아니다.
`coarse_npoint` 는 초기 포즈 후보를 뽑는 기하 매칭의 점 개수라, 줄이면 엉뚱한 대칭 해를
고르기 시작한다.

> 이는 **1차 보고의 권고를 뒤집는다.** 당시 `coarse_npoint=96` 을 "가장 효과적(−26%)"으로
> 제시했으나 속도만 보고 내린 판단이었다. 과거 `tools/run_pem_coarse_sweep.sh` 도
> `196 256 392 … 4096` 으로 **위로만** 훑어 196 아래는 평가된 적이 없었다.

★ **`fine_npoint` 축소는 안전하다** — 20/20 으로 baseline(19/20)보다 성공률이 높고
p90 도 2.55° 다.

**BF16 은 공짜가 아니다** — 회전 median 0.24° → 1.31°(5.5배). 절대값은 작고 성공률은
유지되지만, "수치 표현만 바뀌니 안전" 이라던 초기 발언은 과했다.

### 대칭 실패는 원래 있던 문제

**FP32 baseline 에서도 20회 중 1회가 89.79°** 로 틀린다. Milk 는 74×36×194 mm 우유팩이라
세로축 180° 회전이 형태로 구분되지 않는다(요청서 §19). **속도 최적화와 무관한 기존
실패 모드**다. 이 때문에 **mean 은 지표로 쓸 수 없고**(median 0.24 vs mean 4.76)
**median 과 성공률**로 봐야 한다.

---

## 5. ISM 분해 [M]

| 박스 N | MobileSAM | DINOv2 | ISM 신경망 합 |
|---|---|---|---|
| 1 | 35.3 | 9.6 | 44.9 ms |
| 2 | 36.2 | 9.5 | 45.7 ms |
| 4 | **38.8** | 11.7 | **50.5 ms** |
| 8 | 46.5 | 19.5 | 66.0 ms |

🔴 **MobileSAM 이 ISM 의 77%**(B=4 기준 38.8/50.5). 그리고 **박스 1개에도 35.3 ms** —
박스가 8배로 늘어도 11 ms 밖에 안 는다. 거의 전부가 **박스 수와 무관한 고정비**
(이미지 인코더)다.

그래서 **"박스를 줄이는" 최적화는 듣지 않고 "프레임을 건너뛰는" 최적화만 듣는다.**

배치화의 가치도 수치로 확인했다 — 되돌리면 B=4 에서 38.8 → 142.3 ms(3.7배),
B=8 에서 46.5 → 302.2 ms(6.5배). 이미 적용돼 있지만 `recognize_multi` 경로(709행)는
여전히 박스별이라 그 경로를 타는 오프라인 도구는 이 손해를 지불한다.

---

## 6. 구현한 것

### 6-1. FP16/BF16 — "적용"이 아니라 "수리"였다 [M]

`autocast` 를 그냥 씌우면 **즉시 크래시**한다. 세 단계로 막혔다:

| # | 오류 | 위치 | 원인 |
|---|---|---|---|
| 1 | `svd_cuda_gesvdjBatched not implemented for 'Half'` | `utils/model_utils.py` `weighted_procrustes` | `torch.svd` 에 half 커널 없음 |
| 2 | `new_xyz must be a float tensor` | `pointnet2_utils.py` `ball_query` | CUDA 확장이 `CHECK_IS_FLOAT` |
| 3 | `points must be a float tensor` | 〃 `group_points` | 동일 |

`_ext_src/src/*.cpp` 의 **모든 커널이 `CHECK_IS_FLOAT`** 이라 진입점 5곳
(`ball_query`/`furthest_point_sample`/`gather_points`/`group_points`/`three_nn`·
`three_interpolate`)에 fp32 캐스팅을 넣었다. SVD 는 3×3 이라 fp32 로 푸는 게 비용도
없고 수치적으로도 맞다.

요청서 §10 이 요구한 "FP16 가능한 곳 / FP32 유지할 곳 구분" 의 답은
**수치 안정성이 아니라 커널이 존재하지 않는 것**이었다.

### 6-2. MobileSAM 마스크 재사용 — `realtime/mask_cache.py`

박스 단위로 재사용을 판단한다(프레임 단위가 아니다). 안전장치 3개: `iou_thresh`(겹침
부족 시 재계산), `max_age`(낡은 마스크 폐기), `interval`(주기적 강제 갱신).

실측 (4박스, 30프레임):

| 물체 속도 | 재사용률 | ms/frame | 절감 | IoU mean | IoU min | IoU<0.9 |
|---|---|---|---|---|---|---|
| 0 px/f | 83.3% | 19.5 | **−49.4%** | 0.999 | 0.996 | 0 |
| 4 px/f | 50.0% | 19.4 | **−49.8%** | 0.999 | 0.996 | 0 |
| 12 px/f | 0% | 37.8 | −0.5% | 1.000 | 1.000 | 0 |
| 30 px/f | 42.3% | 27.7 | −27.0% | **0.898** | **0.643** | **22** ⚠ |

정지·저속에서 **약 50% 절감에 IoU 0.999**. 12 px/f 에서는 문턱에 걸려 전부 재계산되고
절감이 0 이 된다 — **안전장치가 의도대로 동작**한다.

> **측정이 설계 버그를 잡아냈다.** 1차 실행에서 speed=30 의 재사용률이 speed=12 보다
> 높게 나왔다(비단조). 캐시가 박스 IoU 만으로 조회해 **물체가 직전 프레임에 다른 물체가
> 있던 자리로 이동하면 그 물체의 마스크를 받아갔다.** 합성 박스가 전부 같은 모양이라
> IoU 지표에 숨어 있었다. 박스↔항목 **일대일 매칭**으로 고치고, 테스트 박스를 서로
> 구별되게 바꿔 speed=30 의 IoU 0.898/min 0.643 이 정직하게 드러나게 했다.

### 6-3. PEM 객체 round-robin — `realtime/pem_scheduler.py`

PEM 은 **검출 1개에도 48 ms**, 4개면 87.2 ms [M] — 검출당 증가분(+13 ms)보다 고정비가
크다. 그래서 프레임당 푸는 객체 수를 줄이는 것이 가장 값싼 카드다.

우선순위: ① 한 번도 못 푼 객체 → ② 오래 안 푼 객체 → ③ 직전 점수가 낮은 객체.
`max_age` 를 넘기면 예산을 무시하고 강제로 푼다.

검증 (객체 4개, `budget=1`, `max_age=4`, 12프레임):

```
A: SScccScccScc      S = 이번 프레임에 품
B: ScScccScccSc      c = 직전 포즈 물려받음
C: SccScccScccS
D: ScccScccSccc
frames=12 dets=48 solved=15 (31.2%) carried=33 forced=12
```

모든 객체가 `max_age` 이내에 갱신된다. 물려받은 행에는 `pose_source:"carried"` 와
`carried_age` 가 실려 소비자가 구분할 수 있다.

### 6-4. 연결 — `realtime/sam6d_core.py`

`Sam6DCore` 인자로만 켜지고 **기본은 전부 off**(§26):

```python
Sam6DCore(..., precision="bf16",
          mask_cache={"enabled": True, "iou_thresh": 0.90, "max_age": 5, "interval": 30},
          pem_schedule={"enabled": True, "budget": 1, "max_age": 4})
```

`yolo_ism*.py` 는 한 줄도 건드리지 않았다(`README.md` 의 "원본 그대로 복사(수정 금지)"
준수) — `segment_boxes` 를 감싼 것으로 바꿔 끼우고, 끄면 원본이 복원된다.

---

## 7. 전망

| 단계 | 기준선 | 파라미터만 | +구조 변경 |
|---|---|---|---|
| yolo | 7 | 7 | 7 |
| ism | 87 | 50.5 [M] | **~31** (MobileSAM 38.8→~19) |
| pem | 277 | 144.4 [M] | **~48** (B=4→B=1) |
| **합** | **353 ms** | **~202 ms** | **~86 ms** |
| **FPS** | **2.83** | **4.9** | **11.6** [E] |

**파라미터 튜닝의 한계는 4.9 FPS 다. 10 FPS 는 구조 변경으로만 간다.**
그리고 구조 변경의 1순위는 PEM 이 아니라 **MobileSAM 호출 빈도**다 — 박스 1개에도
35 ms 를 내는 고정비라 "가끔만 실행" 이 가장 잘 듣는다.

---

## 8. 검증하지 못한 것 (중요)

1. **통합 실행 불가.** `Sam6DCore` 를 띄우려면 15종 객체의 ISM 템플릿 특징 캐시와
   PEM 템플릿(`assets/pem_templates/*.pt`)이 필요한데 CAD 가 비공개라 만들 수 없다.
   Milk 하나로는 운영 config 가 뜨지 않는다. **위 "~86 ms" 는 부분 실측을 조합한
   예상치이지 통합 측정이 아니다.**

2. **carried 포즈의 실제 오차 미측정.** `budget=1`, 객체 4개면 각 포즈는 4프레임
   (133 ms) 동안 낡는다. 8자 코스처럼 카메라가 계속 도는 시퀀스에서는 무시 못 할 수
   있다. `slam_pose_memory.py` 로 보정하는 경로가 이미 있어 결합하면 완화되겠지만
   그것도 실측이 필요하다.

3. **마스크 캐시 이득은 장면 의존적.** 12 px/frame 이상 움직이면 재사용률 0% 였다.
   카메라가 움직이는 시퀀스에서는 이득이 거의 없을 수 있다.

4. **정확도 검증은 객체 1종·합성 관측·20 trial.** 15종을 대표하지 않고, 센서
   노이즈·부분 가림·조명 변화가 없으며, 실제 ADD 가 아니라 self-consistency 다.
   17/20 vs 19/20 차이는 표본이 작다(다만 cn 축소 3개 설정이 모두 17/20 인 일관성은
   유의미하다).

5. **실행 간 변동 [M].** 같은 baseline(B=4, BF16)이 143~177 ms 로 흔들린다(노트북 GPU
   발열/클럭). **4% 수준 차이는 잡음과 구분되지 않는다** — `nproposal2` 의 "−4%" 를
   이득으로 취급하면 안 되는 이유다.

---

## 9. 권장 설정

```yaml
# 안전 — 정확도 검증됨 (Milk, 20 trial)
precision: bf16
fine_npoint: 1024          # 2048 → 1024, 20/20 성공
coarse_npoint: 196         # 건드리지 말 것 — 96 으로 줄이면 17/20
nproposal2: 300            # 건드리지 말 것 — 효과 4%, 잡음 수준

# 구조 — 이득은 크지만 장면 의존적, 실 영상 검증 필요
mask_cache: {enabled: true, iou_thresh: 0.90, max_age: 5, interval: 30}
pem_schedule: {enabled: true, budget: 1, max_age: 4}
```

**하지 말 것**: backbone 교체(ViT 는 7~9%), backbone 우선 TensorRT, `nproposal2` 축소,
`coarse_npoint` 축소.

---

## 10. 만든 것

| 파일 | 용도 |
|---|---|
| `tools/bench_pem_modules.py` | PEM 모듈별 합성 벤치 (`--batch/--precision/--sweep-*/--set`) |
| `tools/bench_pem_accuracy.py` | 알려진 R·t 로 포즈 정확도 측정 |
| `tools/bench_ism_modules.py` | MobileSAM/DINOv2 분해, 배치 vs 직렬 |
| `tools/bench_mask_cache.py` | 마스크 재사용의 속도 이득 + IoU 비용 |
| `realtime/mask_cache.py` | 마스크 프레임 간 재사용 |
| `realtime/pem_scheduler.py` | PEM 객체 round-robin |
| `realtime/sam6d_core.py` (수정) | 위 둘 + BF16 연결 |
| `utils/model_utils.py` (수정) | SVD 를 AMP-safe 하게 |
| `model/pointnet2/pointnet2_utils.py` (수정) | 커널 진입점 5곳 fp32 캐스팅 |

원본 백업: `/tmp/model_utils.py.bak`, `/tmp/pointnet2_utils.py.bak`, `/tmp/sam6d_core.py.bak`

### 환경 (`sam6d`, 11 GB)

`_env_specs/sam6d.requested_cmds.txt` 의 ROS Jazzy 전체(17 GB) 대신 PEM/ISM 측정에
필요한 부분만 세웠다. 원본 레시피에 없어 추가로 필요했던 것:
**`six`, `opencv`, `imageio`, `trimesh`, `scikit-image`, `pycocotools`, `gdown`**
(전부 `gorilla-core` 와 PEM 의 `data_utils`/`run_inference_custom` 의존성).

`TORCH_CUDA_ARCH_LIST` 는 이 기계에 맞춰 **`8.6` 하나만** 구웠다(원본은
`8.6;8.9;9.0;12.0+PTX` — 이식 대상 GPU 를 몰라 여럿 구운 것).

받은 가중치: `sam-6d-pem-base.pth`(1.3 GB), `mae_pretrain_vit_base.pth`(327 MB),
`dinov2_vits14_pretrain.pth`(85 MB), `mobile_sam.pt`(39 MB).

---

## 11. 다음 단계

| # | 작업 | 왜 |
|---|---|---|
| 1 | **CAD 확보** | 이것 없이는 통합 검증도 실 정확도 측정도 불가. 최대 병목 |
| 2 | 실 영상으로 통합 벤치 | "~86 ms" 가 예상치인 채로 남아 있다 |
| 3 | carried 포즈 오차 측정 | `budget`/`max_age` 를 실제로 정하려면 필요 |
| 4 | config/preset 체계 (§23·24) | 지금은 `Sam6DCore` 인자. accuracy/balanced/fast |
| 5 | §19 대칭 처리 | baseline 에도 있는 실패 모드. temporal consistency 로 완화 |
