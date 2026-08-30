# Technical Research — Depth/Pose Gate Phase 0: Workspace vs Depth 효과 분리 분석

> shadow 로그(315프레임, 정식 GT 시간매핑)·이미지를 실측 분석. **코드 미수정, hard 전환 안 함, git 없음.**

## # Phase 0 Shadow 결과 재해석
- Baseline(정식 GT): TP=112 FP=120 FN=6 TN=77 **P=0.483 R=0.949**.
- **결론(데이터 입증): 개선은 거의 전부 workspace(pose z) 효과이며, depth consistency(agreement)는 오히려
  recall을 파괴한다.** "depth gate"라는 이름과 달리 실효 신호는 **pose z 타당성**이다.

## # Workspace-only vs Depth-only 비교 (published 프레임, 기본임계 err100/agree0.5)
| 조건 | TP | TN | FP | FN | Prec | Rec | F1 | ΔFP | ΔFN | Rdrop |
|---|---|---|---|---|---|---|---|---|---|---|
| **workspace only** | 97 | 160 | 37 | 21 | **0.724** | **0.822** | **0.770** | 83 | +15 | 0.127 |
| depth error only(em>100) | 99 | 129 | 68 | 19 | 0.593 | 0.839 | 0.695 | 52 | +13 | 0.110 |
| **depth agree only(ag<0.5)** | 66 | 151 | 46 | 52 | 0.589 | **0.559** | 0.574 | 74 | +46 | **0.390** |
| ws + depth error | 91 | 178 | 19 | 27 | 0.827 | 0.771 | 0.798 | 101 | +21 | 0.178 |
| ws + depth agree(=기본 gate) | 59 | 183 | 14 | 59 | 0.808 | **0.500** | 0.618 | 106 | +53 | 0.449 |

- **workspace only가 단독 최강**: FP −83, R 0.822(유일하게 ≥0.80), F1 0.770. TP 손실 15뿐.
- **depth agreement는 유해**: FP 74 제거하나 TP 46 죽임(Rdrop 0.39). workspace에 더하면 R 0.822→0.500.
- depth error(em>100)는 약함: FP 52 제거하나 68 잔존.
- → **개선의 본질 = workspace. depth-rendering 일치는 음(−)의 기여.**

## # z_max Sweep 결과 (xy_max=800 고정, depth 미적용)
| z_max | TP | FP | FN | Prec | Rec |
|---|---|---|---|---|---|
| 1000 | 93 | 21 | 25 | 0.816 | **0.788** |
| 1200 | 93 | 21 | 25 | 0.816 | **0.788** |
| **1500** | 97 | 37 | 21 | 0.724 | **0.822** |
| 1800 | 97 | 50 | 21 | 0.660 | 0.822 |
| 2000~3000/disabled | 98 | 53 | 20 | 0.649 | 0.831 |

- **z_max=1500이 recall 절벽 바로 위 knee**: 1200이면 R 0.788(<0.80 실패), 1800+는 FP만 늘고 TP 무이득.
- 즉 1500은 임의값이 아니라 데이터상 knee. **단 민감(±300에서 합격/불합격 갈림) → 과적합 위험 있음**.
  hold-out 분리 검증 없이 1500 확정은 위험.

## # Depth Metric 분리력 분석 (published: TP vs FP)
| metric | TP median | FP median | 분리력 |
|---|---|---|---|
| depth_valid_ratio | 1.00 | 1.00 | **무력**(동일) |
| depth_error_median | 39.6 | 93.3 | 약함(겹침 큼) |
| depth_error_p90 | 39.7 | 93.3 | 약함 |
| depth_agreement_ratio | 1.00 | 0.00 | median은 분리, **그러나 TP 꼬리 46개 ag<0.5 → 잘못 제거** |
| **pose_z (mm)** | **694** | **1609** | **실질 분리**: FP 70/120(58%) z>1500, TP 15/112(13%)만 |

- depth_valid_ratio는 무력(거의 항상 1.0). depth agreement는 median 분리되나 **실제 milk의 노이즈 꼬리**가
  커서 hard cut 시 recall 파괴. → **현 gate는 'depth consistency gate'가 아니라 'workspace(pose-z)
  plausibility gate'로 재정의해야 한다.** (pose_z가 유일하게 안전·유효한 분리축)

## # 제거된 TP 이미지 분석 (GT=milk & PUBLISH & z>1500, 15개)
- 구간: 001184~001217, 001571~001608 (z=2108~2753mm). 일부 pose_score 0.99 고신뢰.
- **육안(001204, z=2614, pose=0.9995)**: milk carton은 **명확히 존재**(검출 자체는 옳음) — 그러나
  pose z=2.6m는 **비현실**(milk는 ~1m 거리). 즉 **"milk 있음 + pose 틀림"**.
- 해석: frame-GT(milk 유무)는 검출만 보지만, 제거되는 TP의 pose는 사실상 **사용 불가(z 2.6m garbage)**.
  → workspace 제거의 "recall 손실"은 상당부분 **나쁜 pose를 버리는 것**이라 실질 손실이 과대평가됨.
  (단, 일부는 원경 milk가 실제 멀리 있을 가능성 → depth GT 없이 100% 확정 불가)

## # 제거된 FP 이미지 분석 (GT=no-milk & PUBLISH & z>1500, 70개)
- z=3196~3238mm(3.2m), 사무실 구간(000355 등).
- **육안(000355, z=3196)**: 의자·모니터·공기청정기·박스, **milk 없음** → 명확한 오검출. 제거 타당.
- 패턴: 모델이 사무실 원거리 흰색 물체/배경에 milk pose를 3m+ 거리로 발행. workspace z<1500이 정확히 차단.

## # only_Milk 영향 분석
- only_Milk(기존 run, 109프레임) pose z: **med=542, min=497, max=696mm — 전부 <1500**.
- → **workspace z<1500 적용 시 reject 0/109 = Recall 1.0 보존** ✓. milk가 항상 근접(~0.5m)이라 안전.
- **단 주의**: depth agreement gate를 켜면 only_Milk에서도 TP가 죽을 수 있음(nomilk처럼). only_Milk에
  gate 코드로 shadow run을 아직 안 돌렸으므로, **depth 성분 포함 시 only_Milk shadow 재실행 검증 필수**.
  workspace-only라면 z분포상 안전.

## # GT Timestamp 문제 분석
- 현재 GT(visibility_labels.csv)는 **stem(frame index) 키**. 그러나 노드는 추론 바쁨에 따라 **매 run마다
  다른 stem을 처리**(비결정적 프레임 스킵) → 정식 GT를 직접 못 씀, 세그먼트 carry-forward 매핑 사용.
- **위험**: ① 세그먼트 경계 ±몇 프레임 오차가 boundary 프레임 라벨을 흔듦 → P/R에 잡음. ② run마다 stem이
  달라 결과 재현/비교가 불안정(같은 임계도 run별로 ±수% 변동). ③ z_max=1500 같은 민감 임계의 과적합 판정이
  GT 잡음에 오염.
- **권장: timestamp 기반 GT로 전환**. bag 메시지 stamp(또는 frame stamp)에 라벨을 키잉하고, 각 run의 stem을
  stamp로 매핑하면 run 비결정성과 무관하게 동일 GT 적용 가능. (디버그 로그 timestamp는 현재 wall-clock이라
  bag stamp 컬럼 추가 필요 — 별도 로깅 보강.)

## # Hard Mode 전환 가능 여부
사용자 5조건 평가:
| 조건 | 결과 |
|---|---|
| nomilk Recall ≥ 0.80 | ✅ workspace-only R=0.822 (depth agree 포함 시 0.50 ❌) |
| Precision > baseline(0.483) | ✅ workspace-only P=0.724 |
| only_Milk Recall=1.0/손실<1% | ✅ workspace z<1500 reject 0 (단 depth성분 포함 시 미검증) |
| 제거 TP가 오라벨/비현실 pose | △ 대체로 "milk有+pose틀림"(z 2.6m) — 제거 정당하나 일부 원경 가능 |
| 제거 FP가 명확한 오검출 | ✅ 사무실 3.2m 오검출 |

- **판정: workspace-ONLY 구성은 hard 후보 자격 충족**(R 0.82, P 0.72, only_Milk 안전). 단 **depth agreement는
  반드시 제외**(recall 파괴). **그러나 아직 hard 금지** — z_max 민감(과적합)·GT 잡음·only_Milk depth 미검증
  때문에 **hold-out 검증 + only_Milk gate shadow run 후** 결정해야 함.

## # 현재 접근이 파라미터 튜닝에 치우쳤는지 판단
- **그렇다. 현재 "개선"은 사실상 workspace z_max 파라미터 튜닝**이다. depth-rendering 일치(agreement/error)는
  분리력이 약하거나(error) 유해하다(agreement). "depth consistency gate"라는 명명이 실제 효과(pose-z
  plausibility)와 불일치.
- 이는 **물리적으로 타당**(FP는 원거리 사무실 물체에 latch → z가 큼; milk는 근접 → z 작음)하나, 단일
  z_max 임계는 장면/객체 의존적이라 **일반화·과적합 리스크**가 있다.

## # 다음 PRD/Quick-Dev 권장안
1. **Gate 재정의(PRD 소폭 수정)**: "Depth/Pose Consistency Gate" → **"Pose Plausibility Gate(workspace
   중심)"**. depth agreement는 기본 off(또는 진단용만), depth error는 매우 느슨(err_max≫)하게 보조.
2. **GT timestamp화(Quick-Dev)**: 디버그 로그에 bag message stamp 컬럼 추가 → GT를 stamp 키로 전환,
   run 비결정성 제거. (분석 신뢰도 전제)
3. **검증 강화(코드 무관)**: ① z_max를 train/hold-out 분리로 산정(과적합 차단) ② only_Milk에 gate shadow
   run 실행해 reject 0 실측 ③ 제거 TP를 "pose 정확도" 기준으로 재라벨(검출 recall과 pose validity 분리).
4. **hard 전환은 위 검증 통과 후**. 현재는 shadow 유지.
5. (선택) pose-z 외 보조 신호 탐색: bbox/mask area, ism_score는 TP/FP 분리 약함(이전 분석) → z가 최선.

> 근거: outputs/validation/SLAM_with_milk_nomilk/_run/debug/sam6d_debug.csv(gate 컬럼), frames/*.png,
> visibility_labels_thisrun.csv; only_Milk/frame_results.csv. 분석 스크립트는 /tmp(임시). 코드·git 변경 없음.
