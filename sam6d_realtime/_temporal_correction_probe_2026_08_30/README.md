# _temporal_correction_probe_2026_08_30 — 시간축 포즈 보정: 전 프레임 vs 2 FPS 실측 비교

"부정확한 SAM-6D 포즈를 다음 프레임들을 보면서 보정"하는 운용 보정기
(`realtime/slam_pose_memory.ObjectAnchorManager`)에 **같은 bag 의 같은 검출 스트림**을
30fps 전체와 0.5초 간격(실시간 모사, ~2fps)으로 각각 흘려 넣어 보정 품질을 비교했다.
원본 코드는 수정하지 않았고 이 폴더만 지우면 원상 복구된다.

## 재료

| 항목 | 출처 |
|---|---|
| bag | `data/longcircle2_sam` (260804_office/longcircle2 SAM 카메라, 2130프레임 71초, manifest 생성·검증) |
| 전 프레임 검출 | `tools/capture_pem_explorer.py` 재실행 → `output/longcircle2_visualization/detections.jsonl` (1933행, 630초 소요) |
| 카메라 궤적 | `output_slam/260804_office/orb3_slam/longcircle2/CameraTrajectory.txt` × `integration/.../longcircle2/calib/X_camSLAM_camSAM.npy` |
| 보정기 | `ObjectAnchorManager` 기본값: 20관측 윈도우, 16개가 20°/50mm 클러스터면 앵커 등록, 5관측 중 4개 불일치면 해제 |
| pseudo-GT | 물체는 정적 → map 좌표계 전체 30fps 관측의 대칭 인지 dominant-cluster medoid |

평가: 출력 포즈(앵커 있으면 앵커, 없으면 원 측정)를 다음 출력까지(최대 1초 만료) 표시한다고
보고, pseudo-GT 대비 표시 시간 가중 오차와 "나쁜 표시"(>20° 또는 >50mm) 비율을 계산.

## 결과 (2026-08-30)

| | full 30fps | rt 2fps |
|---|---|---|
| 처리 프레임 | 1102 | 83 |
| 앵커 등록된 물체 | **7/9** (등록까지 0.6~18s, 대부분 4s 미만) | **3/9** (등록이 58~60s — 71s bag의 끝자락) |
| 앵커 출력 비율 | 66% (1267/1930) | 7.6% (11/145) |

물체별 나쁜표시% (full → rt): Bear 8.1→3.5, Dinosaur 0→0, Febreze 33.6→20.8,
Mugcup 21.6→11.6, Sauce 46.1→0(관측1회뿐), Sikhye 0→0, **choco 53.1→55.6**,
**milk 11.6→32.2 (오차 12.8°/28mm → 47.8°/239mm)**, saffron 30.7→29.6.

## 해석

1. **2fps 에서는 이 보정기가 사실상 작동하지 않는다.** 앵커 등록에 20관측이 필요한데
   2fps 로는 이론상 10초+, 실제로는 가시성 창이 쪼개져 3/9 물체만 run 종료 직전에 등록.
   보정된(앵커) 출력이 전체의 7.6%뿐 — 나머지는 원 측정 그대로 나간다.
2. **보정이 작동하면 flip 흡수가 극적으로 다르다** (milk): full 은 뒤집힌 측정을 앵커가
   흡수하고 다음 프레임(33ms)에 회복; rt 는 뒤집힌 포즈가 수 초씩 화면에 남는다
   (나쁜표시 11.6% vs 32.2%).
3. **반례 1 — 빠른 등록의 함정** (Mugcup): full 은 0.63s 만에 등록하는데 초기 측정이
   뒤집혀 있으면 잘못된 모드에 앵커가 잠긴다(해제 3회 반복). 관측 수 기반 등록은
   "빨리 모으면 빨리 틀릴 수도" 있다.
4. **반례 2 — 측정 자체가 쌍봉이면 어느 쪽도 못 고친다** (choco, occupancy 60.5%):
   전 구간의 절반이 ~180° 뒤집힌 측정이라 rate 와 무관하게 보정 불가. 이건 시간축
   보정이 아니라 검출/검증(텍스처) 단계의 문제다.
5. **커버리지도 잃는다**: 2fps 는 짧은 가시성 창을 통째로 놓친다 (Sauce: full 15회
   관측 vs rt 1회).

## [추가 2026-08-30] soft 보정 실측 — 2fps 의 손실 대부분을 회수한다

등록 전 구간에서 원 측정 대신 **최근 K개 map 관측의 dominant-cluster medoid** 를
출력하는 soft 보정을 리플레이에 구현했다 (`replay_compare.py soft_pose()`,
앵커 등록/해제 로직은 무변경, 등록/해제와 같은 20°/50mm 임계·같은 medoid 함수 재사용).

표시시간 가중 전체 집계 (9물체):

| stream | 나쁜표시% | 오차(deg) | 오차(mm) |
|---|---|---|---|
| full 30fps (기존 앵커) | 25.1 | 32.3 | 91.5 |
| rt 2fps raw | 23.9 | 35.4 | 139.9 |
| rt 2fps + soft K=3 | 15.5 | 27.8 | 94.0 |
| **rt 2fps + soft K=5** | **9.3** | **16.5** | **58.4** |
| rt 2fps + soft K=7 | 8.3 | 16.3 | 56.9 |
| full 30fps + soft K=5 | 22.8 | 30.9 | 86.5 |

- **rt+soft5 는 rt raw 대비 나쁜표시를 23.9→9.3%로 줄이고, full(기존 앵커)보다도 좋다.**
  milk 32.2→9.7%, saffron 29.6→11.1%, Febreze 20.8→0%, Mugcup 11.6→0%.
- full 보다 좋아지는 이유: soft medoid 는 앵커처럼 **잠기지 않는다**. full 의 나쁜표시는
  초기 flip 모드에 앵커가 잠긴 사례(Mugcup 0.63s 등록→11.5초간 뒤집힘 유지)가 지배하는데,
  soft 는 최근 다수를 따라가므로 스스로 회복한다.
- K=5 가 무릎점 (K=7 은 미미한 추가 이득, 반응 지연만 커짐). K=3 은 창 안에 flip 이
  2개 들어오면 소수파를 못 누른다.
- 한계는 동일: choco(측정 40% flip)는 32.9%로 개선되지만 못 고친다 — 검증 단계 문제.
- 주의: rt 는 표본이 적어(83프레임) 운 좋게 이상치 구간을 건너뛴 효과가 섞여 있다.
  같은 스트림 내 비교(rt raw vs rt+soft)가 순수한 soft 효과다.

**[운용 반영 완료 2026-08-30]** soft 보정을 운용 코드에 넣었다 (백업: `*.pre_soft`):

- `realtime/slam_pose_memory.py`: `ObjectAnchorManager` 에 `soft_window` 설정(기본 5, 0=끔)과
  `soft_map_pose()`/`soft_camera_pose()` 추가 — 등록과 같은 임계·같은 medoid 함수 재사용,
  관측 2개 미만이면 None(원 측정 유지), 앵커 등록 후엔 물러남.
- `realtime/sam6d_core.py` `_apply_anchors()`: collecting 상태의 수락 행을 soft 포즈로 대체.
  `pose_source: "soft_medoid"`, `anchor_state: "soft"`, 원 측정은 `raw_R`/`raw_t_mm` 보존,
  진단에 `soft_objects` 기록. anchor 를 켠 run(`anchor.enabled: true`)에서만 동작한다.
- `realtime/run_live_split.yaml` 에 `soft_window: 5` 명시.
- 검증: `test_soft_production.py` (합성 포즈 5케이스: 다수결/비활성/앵커 우선/행 대체/구동작
  보존) 전부 통과. 리플레이 하네스도 운용 코드 경로(`soft_map_pose`)를 직접 쓰도록 바꿔
  재실행 — rt_soft5 수치가 실측과 동일하게 재현됨 (나쁜표시 9.3%, 패리티 확인).

## 실시간 보정기 설계 시사점 (다음 단계 후보)

- 등록 조건을 **관측 수 → 시간창 기반**으로 (예: 2초 내 4/6 관측 합의): 2fps 에서도
  수 초 내 등록 가능. 단 Mugcup 사례처럼 이른 잠금의 오등록률을 같이 측정할 것.
- 등록 전이라도 **직전 합의(최근 K개 medoid)로 출력**하는 soft 보정 — 앵커 등록을
  기다리는 동안 원 측정을 그대로 내보내는 현재 구조가 rt 손실의 대부분.
- ISM(YOLO, 7ms)은 30fps 로 돌리고 PEM 만 2fps 인 **하이브리드**: 가시성/연속성 정보는
  전 프레임에서 얻어 앵커 유지·재획득에 쓸 수 있다.

## 재실행

```bash
conda activate sam6d
python _temporal_correction_probe_2026_08_30/replay_compare.py   # 비교 재계산
python _temporal_correction_probe_2026_08_30/plot_timelines.py   # 타임라인 플롯
```

산출물: `outputs/compare.json`(요약), `timeline_{full,rt}.json`(원자료),
`timelines.png`(물체별 회전 오차 타임라인).
