# ObChange Kitchen — 객체 변화 실험 첫 결과 (2026-10-05)

데이터: ObChange(TU Wien) Kitchen, 방문 6회(s1 은 YCB 물체 없음, s2–s6 에 물체 4–9개). 채점 대상은 템플릿이 있는 11종(프라이팬·접시 제외) → 방문 쌍 10개에서 정답 100개.
채점 기준은 데이터셋 논문(Langer 외 2022)과 같다: 그대로/옮겨짐 경계 20 cm, 검출 인정 10 cm.

## 실행
```bash
OB=~/DeepLearning/Datasets/ObChange; O=objpose/output/obchange_261005
python3 objpose/tools/obchange/make_bop.py --room Kitchen --raw $OB/raw/GH30_kitchen --step 3          # bag → BOP 장면(10 fps) + tf 자세
~/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/gt_centers.py --room Kitchen               # 정답 중심
~/anaconda3/envs/sam6d/bin/python objpose/tools/baselines/run_compare.py --method proposed --dataset obchange --mode realtime --all-objects --out $O/proposed
BENCH_ROOT=$OB/bop BENCH_TABLE=$PWD/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 \
  ~/anaconda3/envs/sam6d/bin/python objpose/tools/obchange/proposed_maps.py --run $O/proposed --out $O --tag proposed
bash objpose/tools/obchange/run_dualmap.sh kitchen $O                                                # DualMap (5 fps)
# FM-Fusion: make_ycbv_scans.py(BENCH_SLAM=$OB/bop/poses_tf) → detect_gsam_prompts.py --gap 3 --all-objects → run_ycbv.sh … fmf_obchange_small.yaml 3 → fmfusion_to_maps.py
python3 objpose/tools/obchange/change_eval.py --maps $O/maps --methods proposed proposed_online dualmap fmfusion [--bg kitchen_s1]
python3 objpose/tools/obchange/pose_analysis.py --run $O/proposed --orb $O/slam_orb --out $O/eval/pose_analysis.json
```

## 카메라 자세
- 데이터셋 자세 = 로봇(HSR)의 tf: map→odom(2D 위치 추정, 주행 중 2–7 cm 보정) → odom→base(바퀴 오도메트리) → 머리 관절 → 카메라. 시각 SLAM 이 아니다.
- ORB-SLAM3 RGB-D(3000 특징)를 같은 영상에 돌려 비교(6개 방문 평균, ORB 궤적을 tf 궤적에 SE3 로 한 번 맞춤): 궤적 차 RMSE 2.4 cm(최대 4.1), 회전 차 중앙 1.1°.
- 정답 물체 기준(제안 방법 프레임별 추정을 지도에 옮긴 위치 ↔ 정답 중심): tf 중앙 1.9 cm / ORB 2.4 cm, 같은 물체 관측의 흩어짐 중앙 tf 0.44 cm / ORB 0.20 cm.
- 본 실험은 방문 사이 좌표가 같아야 하므로 tf 자세를 썼다(ORB-SLAM3 는 방문마다 좌표계가 달라 tf 에 맞춰야 한다).

## 변화 채점 (정답 100개)
| 방법 | 검출(이름 기준) | 범주 맞음 | 잘못된 변화 | 위치 기준 검출 / 맞음 |
|---|---|---|---|---|
| 제안 · 방문별 지도 비교 | 64 | 55 | 12 | 64 / 47 |
| 제안 · 한 지도에 이어 넣기 | 65 | 41 | 16 | 66 / 46 |
| FM-Fusion · 방문별 지도 비교 | 56 | 42 | 140 | 100 / 60 |
| DualMap · 방문별 지도 비교 | 32 | 16 | 76 | 100 / 53 |
물체 없는 방문(s1)에도 있는 객체를 배경으로 빼면: 제안 60/56/10, 이어 넣기 61/42/18, FM-Fusion 56/43/114, DualMap 28/19/54.

## 해석
- 제안 방법은 이름 기준 범주 정확도가 가장 높고 잘못된 변화가 가장 적다. 못 찾는 물체가 있다(그릇 뒤집힘, 젤라틴·푸딩 상자, 설탕 상자 일부).
- FM-Fusion·DualMap 은 정답 자리마다 무언가를 올리지만(위치 기준 검출 100%) 이름이 틀려 변화 범주를 못 맞히고 잘못된 변화가 많다.
- 한 지도에 이어 넣는 방식(상태 관리)은 방문별 비교보다 나쁘다: 30 cm 연관 때문에 가까이 옮겨진 물체나 배경의 비슷한 물체와 합쳐지고(옮겨짐 → 그대로 7건), 옛 객체가 남는 경우가 있다.
- 배경 오검출: 전기 주전자를 pitcher, 은색 통을 master chef can 으로 매 방문 등록한다.

## 한계
- Kitchen 한 방. 나머지 방은 내려받는 중.
- 정답 중심은 보인 점의 상자 중심이라 수 cm 치우침이 있을 수 있다. 방향은 채점하지 않았다(.anno 모델 좌표계 미확인).
- DualMap 클래스 목록과 이름 대응표, FM-Fusion 문구는 이번에 만든 것이다. FM-Fusion 복셀은 5 mm.
- 제안 방법만 실시간 조건(10 fps 입력, 밀리면 건너뜀)으로 돌렸다. DualMap 5 fps, FM-Fusion 3.3 fps 는 모든 프레임 처리.

## 추가 (2026-10-05 낮)
- 데이터셋 위치: `~/DeepLearning/Datasets/ObChange` (옛 위치는 링크). 방 하나 처리: `bash objpose/tools/obchange/run_room.sh <Room>`, 전체 채점·그림·표: `finalize.sh`.
- DualMap 수정 1곳(충돌 방지): `utils/object_detector.py` 의 `update_bbox` — 겹침을 빼고 빈 마스크가 되면 원래 코드는 ValueError 로 죽고 프로세스가 끝나지 않는다(smallroom_s1 에서 6시간 반 멈춤). 빈 마스크면 (0,0,0,0) 을 돌려주게 했다. 논문에 DualMap 수치를 쓸 때 이 수정을 밝힌다. `run_dualmap.sh` 에는 90분 제한을 넣었다.
- 세 방 합계(배경 제외 없음): 지도 정답 49 — 제안 35 맞음·없는 물체 10, FM-Fusion 37·255, DualMap 16·38. 변화 정답 196 — 범주 맞음 125 / 108 / 41.
- 발표자료: `~/다운로드/논문발표자료_0.04v_구성도추가_261005.pptx` 57쪽 구성도(`docs/ppt/arch_261005/build_arch.py`), 58쪽부터 비교(`build_results.py --replace` 로 갱신).
