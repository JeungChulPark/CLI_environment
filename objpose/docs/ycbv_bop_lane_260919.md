# YCB-V (BOP) 레인 — 공개 데이터셋을 이 파이프라인으로 돌리기 (2026-09-19)

논문용 공개 벤치마크를 기존 objpose 파이프라인(SLAM 포즈 + SAM-6D + 객체 메모리 + 뷰어)에
그대로 태우기 위해 만든 경로. 코드는 새로 만든 4개 + 기존 파일 2곳 수정이 전부다.

```
노트북 192.168.240.1 (etri-rog)            이 PC 192.168.240.2
fake_slam_stream.py                        hub.py --sam-session <BOP scene>
  YCB-V GT 카메라 포즈 (TUM, 30 fps)         ├─ BopSession: rgb/·depth/ 폴더 1x 재생
  ──TCP 17001 (hello/start/pose)──────────► ├─ SAM-6D (YCB 21객체 자산)
                                            ├─ 융합 + 객체 메모리
                                            └─ http://192.168.240.2:8765  → viewer.mp4
```

## 왜 이렇게 했나

`Datasets/paper_benchmarks/README.md` 가 적어 둔 3가지 부족분을 그대로 메운 것이다.

1. **폴더형 세션 어댑터** — `pc/bop_session.py`. `ConvSession` 과 같은 인터페이스
   (`K` / `__len__` / `read(i, with_depth)` / `align`). BOP 영상은 시계가 없으므로
   `t0 + index/fps` 로 스탬프를 합성하고(YCB-V 는 30 fps), **포즈 스트림도 같은 시계**로
   만든다(`tools/bop_to_tum.py`). 그래서 τ = 0 이고 두 재생이 프레임 단위로 맞는다.
2. **depth 스케일** — YCB-V 는 `depth_scale: 0.1`(0.1 mm 단위)이라 `read()` 에서 mm 로
   환산한다. 파이프라인 나머지는 전부 mm 가정이다.
3. **SAM-6D 객체 자산** — 21종 전부 새로 구웠다(아래).

SLAM 은 **장면의 GT 카메라 포즈**로 대체했다. 목적이 SLAM 정확도가 아니라 *객체 포즈 + 객체
메모리* 평가이므로, SLAM 오차를 0 으로 고정해 두 가지를 분리한다. 포즈는 노트북이 LAN 으로
보내므로 2호스트 경로(터널 없이 TCP 직결)는 실제로 그대로 탄다.

## 만든 자산 (21종, 약 3.5 GB)

| 산출물 | 도구 | 비고 |
|---|---|---|
| 42뷰 템플릿 렌더 | `tools/render_bop_templates.sh` | Blender 3.3.1 + blenderproc, CPU, 객체당 약 55 s |
| `assets/model_points/<name>.npy` | `tools/build_bop_assets.py` | 표면 8192점, mm |
| `assets/pem_templates/<name>.pt` | 〃 | `tp(1,2048,3)` `tf(1,2048,256)` |
| 점별 색 `tc` | `sam6d_realtime/tools/add_template_colors.py` | 21종 모두 색 커버리지 100% |
| ISM cls/appe 캐시 | `tools/prewarm_ycbv_ism.py` | 미리 구워 두지 않으면 허브 기동이 몇 분 멈춘다 |
| 객체표 · 프롬프트 | `rt/bop/ycbv_objects.json` | obj_id ↔ 자산 이름 ↔ YOLO-World 프롬프트 |
| 인식기 설정 | `tools/make_bop_ism_config.py` → `configs/yolo_ism_ycbv.yaml` | |

렌더에 필요했던 `Instance_Segmentation_Model/utils/poses/predefined_poses/cam_poses_level0.npy`
는 저장소에 없어서 SAM-6D 원본 저장소에서 받아 넣었다(42뷰, 기존 lab 템플릿과 같은 뷰 집합).

### 실험실 설정에서 끈 것

`configs/yolo_ism_ycbv.yaml` 의 **HSV 색 게이트는 off**. 임계값 0.1214 는 lab 10객체의
렌더에 맞춰 적합한 값이고 YCB-V 에 대해서는 아무것도 적합시키지 않았다. 나머지 동작점
(appe-v2 blocks 2+9 @0.605, cross-object NMS, similarity 0.35)은 lab 과 **동일**하게 두어
두 데이터셋이 같은 인식기로 돌아가게 했다.

### 대상 객체 제한

BOP 프로토콜은 장면마다 추정 대상 객체 목록을 준다(`test_targets_bop19.json`). 그래서
`run_bop.sh` 는 `scene_gt.json` 에서 그 장면의 객체만 뽑아 `--ism-objects` 로 넘긴다.
21종 전부를 open-set 으로 돌리려면 `ALL_OBJECTS=1`.

## 실행

```bash
# 한 장면: 실시간 재생 + 뷰어 mp4 녹화 (포즈는 노트북에서 LAN 으로)
bash objpose/run_bop.sh                                  # 기본 test/000048
bash objpose/run_bop.sh --scene <...>/test/000052 --duration-s 60
SLAM_HOST=local bash objpose/run_bop.sh                  # 포즈 스트리머도 이 PC 에서

# 12개 테스트 장면 전부 + 채점표
bash objpose/tools/run_bop_scenes.sh

# 채점만
python objpose/tools/eval_bop_run.py --run <run dir> --scene <scene dir>
```

## 채점이 재는 것

`tools/eval_bop_run.py` 는 성격이 다른 두 가지를 따로 잰다.

* **프레임별 포즈** — `sam6d_estimates.jsonl` 의 모든 추정을 `scene_gt.json` 과 카메라
  좌표계에서 비교. ADD / ADD-S 와 `0.1 × 지름` 재현율(YCB-Video 관례).
  회전대칭 객체(캔 등)는 ADD 가 원리적으로 낮게 나오므로 **ADD-S 가 맞는 지표**다.
* **객체 메모리** — `summary.json` 의 융합 랜드마크를 객체의 GT 월드 포즈와 비교.
  YCB-V 는 장면 안에서 객체가 움직이지 않으므로 장면당 정답 월드 포즈가 하나다.
  프레임별 지표가 보여주지 못하는 "수렴했는가"를 이것이 본다.

## 인식기 동작점: lab 설정에서 바꾼 것 두 가지

1. **HSV 색 게이트 off.** 임계값 0.1214 는 lab 10객체 렌더에 적합시킨 값이고 YCB-V 에
   대해서는 아무것도 적합시키지 않았다.
2. **per-box owner election(`relative_assignment_enabled`) off.** 이게 켜져 있으면 프레임의
   박스마다 DINOv2 semantic 점수 1등 객체 **하나**가 박스를 가져간다. `tools/diag_bop_ism.py`
   로 장면 000048·000057 각 30프레임을 재어 보니 그 때문에 **tuna_fish_can 과
   extra_large_clamp 는 박스를 한 번도 못 받았다**(12장면 배치에서 추정 0건). 끄면
   tuna_fish_can 이 30/30 프레임에서 잡힌다. LM-O 에서도 똑같은 현상이 나왔다(cat·duck·
   holepuncher 0건 → 전부 검출). lab 10객체에 맞춘 동작점이 서로 닮은 객체가 섞인 벤치마크로
   옮겨지지 않는다는 뜻이다. 이유는 `rt/bop/ycbv_objects.json` 의 `ism_overrides_why` 에 있다.

프롬프트는 **손으로 쓴 서술형 그대로** 두었다. `tools/probe_bop_prompts.py` 로 21종 전부
측정해 봤지만 검출률 최대화는 `box` · `toy can` · `black object` 같은 범용어를 고르고,
`diag_bop_ism.py` 로 재면 장면에 없는 객체(pudding_box, power_drill)를 30/30 프레임에서
검출하는 오검출을 만든다. 검출기 재현율은 목적이 아니다. (LM-O 는 반대로 클래스 이름이
실물을 오도해서 — `can` 이 물뿌리개다 — 측정이 도움이 됐다. `docs/lmo_bop_lane_260919.md` 참고.)

## 결과 — 12개 테스트 장면 전부, 실시간 1x, 노트북이 LAN 으로 포즈 송신

| 장면 | 객체 | 검출된 객체 | 추정 수 | ADD<0.1d | ADD-S<0.1d | 메모리 오차(중앙) |
|---|---|---|---|---|---|---|
| 000048 | 5 | 4 | 485 | 67.4% | 95.9% | 10.8 mm |
| 000049 | 4 | 2 | 444 | 100% | 100% | 12.8 mm |
| 000050 | 5 | 5 | 419 | 97.6% | 100% | 8.2 mm |
| 000051 | 5 | 5 | 398 | 58.8% | 97.2% | 6.9 mm |
| 000052 | 3 | 3 | 195 | 100% | 100% | 4.5 mm |
| 000053 | 3 | 2 | 105 | 98.1% | 100% | 9.8 mm |
| 000054 | 5 | 4 | 429 | 99.8% | 99.8% | 11.0 mm |
| 000055 | 6 | 6 | 434 | 92.6% | 100% | 8.6 mm |
| 000056 | 4 | 4 | 268 | 89.9% | 100% | 6.3 mm |
| 000057 | 5 | 4 | 312 | 99.0% | 100% | 15.6 mm |
| 000058 | 4 | 2 | 307 | 100% | 100% | 4.2 mm |
| 000059 | 6 | 5 | 324 | 99.4% | 100% | 8.6 mm |
| **전체** | **55** | **46** | **4120** | **90.3%** | **99.2%** | **4.2–15.6 mm** |

**인스턴스 재현율 4088 / 6494 = 63.0%** — SAM-6D 가 처리한 프레임에 보이는 GT 인스턴스 중
0.1d 안의 포즈를 받은 비율. 위의 ADD/ADD-S 는 *추정이 나온 경우*에만 걸리는 조건부 값이므로
둘을 같이 봐야 한다. 장면별로는 44.1%(000053) ~ 95.6%(000052).

프레임 드롭 0(실시간 유지), SAM-6D 프레임당 300~500 ms, 공급 10 Hz.

### 어디가 약한지는 객체별로 보면 분명하다

| 객체 | 등장 장면 | 검출된 장면 | 추정 수 | ADD-S<0.1d |
|---|---|---|---|---|
| tuna_fish_can | 4 | 4 | 564 | 100% |
| tomato_soup_can | 6 | 6 | 536 | 100% |
| sugar_box | 5 | 5 | 513 | 99.8% |
| bleach_cleanser | 4 | 4 | 468 | 100% |
| master_chef_can | 4 | 4 | 430 | 97.7% |
| … | | | | |
| bowl | 2 | 1 | 2 | 100% |
| **large_clamp** | 2 | **0** | 0 | — |
| **potted_meat_can** | 3 | **0** | 0 | — |
| **foam_brick** | 1 | **0** | 0 | — |
| **pudding_box** | 1 | **0** | 0 | — |
| **gelatin_box** | 1 | **0** | 0 | — |

**객체 커버리지 46/55 (83.6%).** 포즈가 틀리는 게 아니라 — 잡히기만 하면 ADD-S 는 99.2% —
**아예 안 잡히는 객체가 남는다**는 것이 이 시스템의 YCB-V 실패 양상이다. 전체 평균만 보면
이게 감춰지므로 `tools/summarize_bop_results.py` 가 커버리지를 함께 낸다.

못 잡는 5종의 이유는 서로 다르다. `large_clamp` 는 `extra_large_clamp` 와 거의 같은 물체라
둘 중 하나만 살아남는다(같은 장면에서 extra 쪽이 114건). `potted_meat_can`·`pudding_box`·
`gelatin_box` 는 작고 평평한 상자/캔이라 open-vocabulary 검출기가 배경 상자와 구별하지
못한다. `foam_brick` 은 한 장면에만 나오고 작다.

## 이전 실행과의 차이 (2026-09-19 오전, 참고)

per-box owner election 이 켜져 있던 첫 12장면 배치는 ADD 86.0% / ADD-S 98.3% / 3966 추정이고
장면 000054 의 메모리 오차가 70.8 mm 였다. 끈 뒤 ADD 90.3% / ADD-S 99.2% / 4120 추정,
메모리 오차 최대 15.6 mm 로 개선됐다. 두 배치 모두 `objpose/output/ycbv_batch_*` 에 남아 있다.
