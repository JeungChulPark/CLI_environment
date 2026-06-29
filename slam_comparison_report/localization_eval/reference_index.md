# Reference Index (representative / median run)

기준 trajectory = 각 dataset×system의 5개 저장 런 중 **상호 ATE 평균이 최소**인 대표(중앙값) 런.
포맷: TUM (`t x y z qx qy qz qw`). GT 아님 — run-to-run 재현성 기준.

## orb3 (`orb3_easyLC/`)

| dataset | runs found | pose counts | chosen ref | mean-ATE(m) |
|---|---|---|---|---|
| SLAM_one_lap | 5 | [1046, 1046, 1046, 1035, 1035] | **run3** | 0.1539 |
| SLAM_one_lap_back_and_forth | 5 | [2138, 2138, 2138, 2138, 2128] | **run3** | 0.0171 |
| SLAM_forward_backward_repeat | 5 | [2529, 2540, 2540, 2540, 2529] | **run2** | 0.0336 |
| SLAM_three_laps | 5 | [4184, 4195, 4195, 4195, 4195] | **run3** | 0.0218 |

## rtab (`rtab/`)

| dataset | runs found | pose counts | chosen ref | mean-ATE(m) |
|---|---|---|---|---|
| SLAM_one_lap | 5 | [6, 14, 21, 27, 21] | **run1** | 0.2729 |
| SLAM_one_lap_back_and_forth | 5 | [61, 61, 59, 61, 59] | **run4** | 0.1094 |
| SLAM_forward_backward_repeat | 5 | [78, 74, 77, 78, 75] | **run5** | 0.1557 |
| SLAM_three_laps | 5 | [87, 120, 96, 109, 112] | **run4** | 1.1001 |
