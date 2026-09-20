# `LOOP_GAP` 로그 필드 — 루프 클로저 계측 (2026-09-21)

ORB-SLAM3 의 루프 클로저가 **얼마나 큰 오차를 흡수했는지**를 한 줄로 남기는 계측 로그다.
`[METRIC#2 / method A]` 로 표시돼 있고, `CorrectLoop()` 이 실제로 맵을 고치기 **직전**에 찍힌다.

| 항목 | 위치 |
|---|---|
| 코드 | `orbslam_ws/src/ORB_SLAM3/src/LoopClosing.cc:298-330` (사본: `objpose/ORB_SLAM3_src/src/LoopClosing.cc`) |
| 빌드 | `bash objpose/src/build_from_repo_mac.sh` → `~/objpose/build` |
| 같이 보는 로그 | `GYRO_LOOP_CHECK` (`LoopClosing.cc:266-291`), 자이로 루프 거부 검사 |
| 실측 로그 | `objpose/rt/260901/bigeight/**/[base|gyro]_run*.log` |

## 1. 찍히는 위치가 의미하는 것

```
후보 검출 → Sim3 정합 → bGoodLoop 판정 → [자이로 veto] → LOOP_GAP 출력 → CorrectLoop()
```

- `if (bGoodLoop)` 블록 **안**이고 자이로 veto **뒤**다. 즉 **채택된 루프만** 기록된다.
  거부된 루프는 `GYRO_LOOP_CHECK|…|verdict=VETO` 에만 남고 `LOOP_GAP` 은 찍히지 않는다.
- `CorrectLoop()` 앞이므로 모든 값은 **보정 전 맵 좌표**다.

## 2. 필드

```
LOOP_GAP|cur_id=|matched_id=|ts_cur=|ts_matched=|cur_xyz=|matched_xyz=
        |gap_m=|sim3t=|sim3_scale=|corr_m=|resid_m=
```

| 필드 | 정의 | 읽는 법 |
|---|---|---|
| `cur_id` / `matched_id` | 현재 KF / 매칭된 KF 의 `mnId` | 차이가 클수록 오래 걸려 돌아온 루프 |
| `ts_cur` / `ts_matched` | 두 KF 의 타임스탬프 (s) | `Δt` = 드리프트가 쌓인 시간 |
| `cur_xyz` / `matched_xyz` | 두 KF 의 카메라 중심 `GetCameraCenter()` | 보정 전 맵 좌표 |
| `gap_m` | `‖C_cur − C_matched‖` | **누적 위치 드리프트의 상한** |
| `sim3t` | `mg2oLoopScw.translation()` | Sim3 의 이동 성분 (아래 주의) |
| `sim3_scale` | `mg2oLoopScw.scale()` | RGB-D 는 실제 스케일이라 1.000000 |
| `corr_m` | `‖C_corrected − C_cur‖` | **보정이 현재 KF 를 실제로 옮긴 거리** |
| `resid_m` | `‖C_corrected − C_matched‖` | 보정 후 매칭 KF 와 남는 거리 ≈ 실제 베이스라인 |

### `gap_m`

```cpp
Eigen::Vector3f cc = mpCurrentKF->GetCameraCenter();
Eigen::Vector3f mc = mpLoopMatchedKF->GetCameraCenter();
double gap = (cc - mc).norm();
```

같은 장소로 인식된 두 키프레임이 보정 전에 얼마나 벌어져 있었는가. 함정 두 가지:

1. **순수 드리프트가 아니다.** place recognition 은 "같은 장면을 보는" 키프레임을 묶지 "같은 지점에 있는"
   키프레임을 묶지 않는다. 두 시점 사이의 **실제 물리적 베이스라인**이 섞여 들어가므로 드리프트의 *상한*이다.
   얼마나 섞였는지는 `resid_m` 이 알려준다.
2. **회전 드리프트는 안 보인다.** 중심 거리만 재므로 자세 오차는 빠진다.
   그건 `GYRO_LOOP_CHECK` 의 `current_vs_gyro_deg` 가 담당한다.

### `corr_m` / `resid_m` — 2026-09-21 추가

`gap_m` + `sim3t` 만으로는 루프가 실제로 무엇을 얼마나 움직였는지 알 수 없어서 추가했다.
`Scw` 는 world→camera (`x_c = s·R·x_w + t`) 이므로 **보정된 카메라 중심은 `−Rᵀt/s`** 다:

```cpp
Eigen::Matrix3d Rcorr = mg2oLoopScw.rotation().toRotationMatrix();
Eigen::Vector3d Ccorr = -(Rcorr.transpose() * s3t) / mg2oLoopScw.scale();
double corr  = (Ccorr.cast<float>() - cc).norm();   // corr_m
double resid = (Ccorr.cast<float>() - mc).norm();   // resid_m
```

**`sim3t` 의 크기를 보정량으로 읽으면 안 된다.** 그 크기는 보정된 카메라 중심이 맵 원점에서 떨어진 거리일
뿐이라 루프가 무엇을 움직이는지가 아니라 *매칭 KF 가 어디 있는지*를 따라다닌다. `corr_m` 이 그 역할을 한다.

삼각부등식으로 `gap_m ≤ corr_m + resid_m`. 판정:

| 패턴 | 해석 |
|---|---|
| `resid_m` 작음, `gap_m ≈ corr_m` | 두 KF 가 정말 같은 지점. `gap_m` 을 드리프트로 읽어도 된다 |
| `resid_m` 큼 | `gap_m` 의 상당 부분이 베이스라인. 드리프트로 읽으면 과대평가 |
| `corr_m` 이 `gap_m` 보다 훨씬 큼 | 루프가 맵을 크게 흔든다 — 오매칭 의심 |

## 3. 실측값 — 260901 (34개 루프)

전 구간 **0.33 ~ 3.67 m**. 실행당 루프는 보통 1회, 일부 2회.

| 세션 | `gap_m` 범위 |
|---|---|
| longcircle | 0.33 – 0.46 (최소) |
| digut | 0.85 – 2.27 |
| eightcircle | 1.00 – 2.80 |
| bigeightcircle | 0.72 – 3.67 (최대) |

`mac_v3_loopfix` (루프 버그 수정 후, 3회 반복):

| | run1 | run2 | run3 |
|---|---|---|---|
| base | 0.87 | 2.15 | 1.58 |
| gyro | 0.93 | 1.04 | 0.87 |

자이로 쪽 산포가 훨씬 작다 — 드리프트가 재현성 있게 억제된다.

2회째 루프가 찍힌 런은 `gap_m` 이 유독 크다 (3.1 / 3.5 / 3.7 m). 예:

```
LOOP_GAP|cur_id=351|matched_id=243|…|cur_xyz=0.017631,0.154192,2.552087
        |matched_xyz=-0.189131,-0.021558,-0.959350|gap_m=3.521907
        |sim3t=-0.326098,-0.072480,0.392161|sim3_scale=1.000000
```

3.5 m 떨어진 두 지점을 같은 장소로 선언한 것으로 8자 교차 오매칭의 전형적 형태다.
다만 이 로그들은 `mac_*_noveto` (veto 미적용본)라 `GYRO_LOOP_CHECK` 줄이 없어 **veto 가 걸러냈다는 증거는 아니다**.
기록에 남은 `GYRO_LOOP_CHECK` 는 4줄뿐이고 전부 `verdict=ok` 다 (0.87° ~ 2.53°, 한계 3.86° ~ 5.08°).
**아직 VETO 가 발동한 로그는 없다.** `corr_m`/`resid_m` 이 들어간 지금 빌드로 다시 돌리면
저 3.5 m 가 드리프트였는지 오매칭이었는지 갈린다.

## 4. 뽑아 쓰기

```bash
# 한 런의 루프 요약
grep LOOP_GAP run.log | tr '|' '\n' | grep -E 'gap_m|corr_m|resid_m'

# 여러 런의 gap_m 분포
grep -rho 'gap_m=[0-9.]*' objpose/rt/260901/bigeight | sort -t= -k2 -g

# 거부된 루프까지 같이 보기
grep -E 'LOOP_GAP|GYRO_LOOP_CHECK' run.log
```

## 5. 남은 것

- `corr_m`/`resid_m` 이 들어간 빌드로 260901 네 세션 재실행 → 큰 `gap_m` 의 정체 확정
- `Gyro.LoopVetoDeg` 값이 두 사본에서 갈려 있다: `objpose/ORB_SLAM3_src` 5.0 (근거 주석 있음),
  `orbslam_ws/src/ORB_SLAM3` 3.0 (주석 없음). 실행 바이너리는 5.0 쪽이다. 어느 쪽으로 통일할지 결정 필요.
