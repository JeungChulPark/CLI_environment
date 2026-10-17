# LM-O (LineMOD-Occlusion) 레인 — 두 번째 공개 벤치마크 (2026-09-19)

YCB-V 다음으로 받은 데이터셋. `bop/lmo`, 769 MB, 테스트 장면 1개(000002) · 1214 프레임 ·
주석 객체 8종(ape, can, cat, driller, duck, eggbox, glue, holepuncher). 6D 포즈 과제가
YCB-V와 같아 표가 그대로 비교되고, 가림이 심해 인식 쪽 한계가 드러난다.

## YCB-V와 다른 점 세 가지 (그리고 대응)

**1. 월드 좌표계가 없다.** `scene_camera.json` 에 `cam_R_w2c`/`cam_t_w2c` 가 아예 없다.
LineMOD 계열은 프레임마다 GT를 독립적으로 주석했고, 실제로 강체가 아니다 — 측정해 보면

| 물체 쌍 | 카메라 좌표계에서의 거리 평균 | 표준편차 |
|---|---|---|
| driller–can | 200.9 mm | **79.4 mm** |
| driller–cat | 306.1 mm | **73.4 mm** |
| duck–eggbox | 426.9 mm | **94.4 mm** |

장면이 강체라면 이 값들은 상수여야 한다. 즉 **카메라 궤적도, 객체의 전역 위치도 정의되지
않는다**. 그래서 LM-O 에서는

* 포즈 스트림 = 고정 항등 포즈(월드 = 카메라). `tools/bop_to_tum.py` 가 `cam_R_w2c` 부재를
  감지해 자동으로 이렇게 만든다.
* 객체 메모리 **끄고**(`--no-memory`) 돌린다. 월드가 없으면 융합할 대상이 없다.
* 영상 오버레이의 "interp" 보간 박스도 끈다(`--overlay-max-age 0.6`). 카메라 포즈가 고정이라
  보간이 옛 박스를 화면에 쌓기만 한다.
* 채점은 프레임별 6D 포즈만. `tools/eval_bop_run.py` 가 이 경우 메모리 절을 건너뛴다.

이건 우회가 아니라 **BOP 가 LM-O 에 요구하는 것 그대로**다. 객체 메모리를 공개 데이터로
검증하려면 연속 시퀀스 + 프레임별 extrinsic 이 있는 HOPE-Video 가 필요하다(아직 미다운로드).

**2. 모델에 UV 텍스처가 없다.** LM 계열 `.ply` 는 정점색(`red/green/blue` 속성)이다.
blenderproc 의 정점색 경로로 렌더되며 결과는 정상이다(42뷰 × 8객체, 객체당 약 25 s).

**3. 클래스 이름이 실물과 다르다.** `can` 은 음료캔이 아니라 **물뿌리개**, `driller` 는 짙은
녹색 무선 드릴, `holepuncher` 는 남색, `ape` 는 짙은 빨강이다. 이름에서 지어낸 프롬프트로는
검출기가 거의 반응하지 않았다.

## 프롬프트는 측정해서 고른다 (`tools/probe_bop_prompts.py`)

객체별 후보(객체 이름 + 렌더 평균색에서 만든 변형 + 표에 적은 수동 후보)를 장면 프레임
24장에 돌려 검출 프레임 수로 고른다. **영상만 쓰고 포즈 GT 는 쓰지 않는다.**

| 객체 | 이름에서 지은 프롬프트 | 측정으로 고른 프롬프트 | 최대 신뢰도 |
|---|---|---|---|
| duck | yellow rubber duck (0.03) | **yellow toy duck** | 0.94 |
| holepuncher | blue hole punch (0.00) | **blue stapler** | 0.93 |
| ape | red ape figurine (0.28) | **dark red figurine** | 0.86 |
| cat | pink cat figurine (0.15) | **pink toy animal** | 0.67 |
| can | watering can (0.52) | **toy can** | 0.79 |

주의: **YCB-V 에는 이 결과를 적용하지 않았다.** 거기서는 같은 도구가 `box` · `toy can` ·
`black object` 같은 범용어를 골랐고, `diag_bop_ism.py` 로 재어 보니 장면에 없는 객체를
30/30 프레임에서 검출하는(pudding_box, power_drill) 오검출을 만들었다. 검출기 재현율은
목적이 아니고 **ISM 검증을 통과하는 수**가 목적이다. LM-O 에서 이 도구가 통한 이유는
클래스 이름이 *오해를 부르기 때문*이지 프롬프트가 *지나치게 구체적이어서*가 아니다.

## 실험실 동작점 중 하나를 껐다 — per-box owner election

`relative_assignment_enabled` 는 프레임의 박스마다 DINOv2 semantic 점수가 가장 높은 객체
**하나**에게 그 박스를 준다(lab 10객체에서 F1 .8176→.8146, 색 타이브레이크와 함께 .8408).
LM-O 에서 이것을 켜면 **cat · duck · holepuncher 가 박스를 한 번도 못 받는다** — 40프레임
진단에서 전부 `no-object(no-proposal)`, 전체 실행에서 추정 0건. 끄면 8종 모두 잡힌다.

| 설정 | 추정 수 | 검출된 객체 | ADD<0.1d | ADD-S<0.1d |
|---|---|---|---|---|
| relative assignment ON | 155 | 5 / 8 | 80.5% | 98.1% |
| **OFF (채택)** | **186** | **8 / 8** | **91.9%** | **99.5%** |

같은 현상이 YCB-V 에서도 확인됐다(tuna_fish_can · extra_large_clamp 가 0건). 두 데이터셋
모두 OFF 로 돌렸고, 이유는 각 `rt/bop/*_objects.json` 의 `ism_overrides_why` 에 적어 두었다.

## 결과 (test/000002 전체 1214 프레임, 실시간 1x, 노트북이 LAN 으로 포즈 송신)

| 객체 | GT 프레임 | 추정 | ADD<0.1d | ADD-S<0.1d | ADD-S 평균 |
|---|---|---|---|---|---|
| ape | 1158 | 21 | 81.0% | 100% | 3.9 mm |
| can | 1207 | 18 | 94.4% | 100% | 4.6 mm |
| cat | 1146 | 17 | 100% | 100% | 4.3 mm |
| driller | 1214 | 30 | 96.7% | 96.7% | 6.8 mm |
| duck | 1109 | 54 | 87.0% | 100% | 3.7 mm |
| eggbox | 1137 | 2 | 0% | 100% | 4.5 mm |
| glue | 857 | 24 | 100% | 100% | 4.7 mm |
| holepuncher | 1210 | 21 | 100% | 100% | 3.3 mm |

**전체 186 추정에서 ADD<0.1d 91.9%, ADD-S<0.1d 99.5%.** 프레임 드롭 0.

그런데 이 두 값은 *추정이 나온 경우*에만 걸린다. 처리된 91 프레임에 보이는 GT 인스턴스
671개를 분모로 두면 **인스턴스 재현율은 185/671 = 27.6%** 다(duck 66.7%, glue 38.3%,
driller 31.9%, ape 24.1%, holepuncher 23.3%, can 20.0%, cat 19.5%, eggbox 2.4%).
YCB-V 의 63.0% 와 비교된다 — LM-O 가 가림 벤치마크라는 점이 그대로 나온다.

포즈 정확도는 매우 높다 — 무언가를 잡기만 하면 거의 맞다(ADD-S 평균 3~7 mm). 남은 약점은
**검출 빈도**다. 40 s 재생 중 SAM-6D 가 처리한 프레임은 280장(10 Hz 공급)인데 객체당 추정은
2~54건이다. eggbox 는 흰색·저텍스처·거의 항상 가려져 있어 2건에 그친다. 이건 포즈 추정기가
아니라 open-vocabulary 검출 + DINOv2 검증 단계의 한계이고, LM-O 가 원래 그것을 재는
벤치마크다.

## 실행

```bash
bash objpose/run_bop.sh --scene /home/jucpark/DeepLearning/Datasets/paper_benchmarks/bop/lmo/test/000002
python objpose/tools/eval_bop_run.py --run <run> \
    --scene /.../bop/lmo/test/000002 --table objpose/rt/bop/lmo_objects.json
```

자산 재생성(이미 만들어져 있다):

```bash
TABLE=objpose/rt/bop/lmo_objects.json bash objpose/tools/render_bop_templates.sh
python objpose/tools/build_bop_assets.py --table objpose/rt/bop/lmo_objects.json
python sam6d_realtime/tools/add_template_colors.py --objects lmo_ape,lmo_can,...
python objpose/tools/prewarm_ycbv_ism.py --config configs/yolo_ism_lmo.yaml
```
