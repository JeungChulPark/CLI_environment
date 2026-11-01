# 주력 설정 (2026-11-01, PC 192.168.219.200 에서 전달)

From 2026-11-01 the user's "주력" (main config) is the PC-hub launcher button `orb-gyro-fl-dp`. The user asked the PC (192.168.219.200) session to pass this on.
Use it for every new 주력 run on this Mac unless the user says otherwise:

- binary: `ORB_BUILD=/Users/user/objpose/build_fl` (source `~/objpose/ORB_SLAM3_src`, backups `*.bak-261101*`)
- command (1030 example; 1029 uses rig_1029_gyro.json):
  `ORB_BUILD=/Users/user/objpose/build_fl ~/objpose/run_slam.sh --gyro rig_1030_gyro.json --param RGBD.DepthMapFactor=1035.0 --param Loop.FalseMinCorrM=3.0 --param Loop.FalseResidFrac=0.25 --param Loop.MaxGapM=2.0 --param Loop.PostLoopRelocM=0.5 --param Loop.EarlyUnlock=1 --slam orbslam3 --session <...>/SLAM --features 2000 --time-source frame --mode live --connect 192.168.219.200:17004`
- what each does:
  - `EarlyUnlock=1`: the tracking lock is released before OptimizeEssentialGraph. Without it tracking stalled 3–21 s after a loop and the pose collapsed 1.6–2.5 m in about 1 of 3 runs.
  - `PostLoopRelocM=0.5`: for 300 frames after a loop, relocalize against KFs ≥100 ids older every 5 frames. If the result is >0.5 m off, jump there.
  - `FalseMinCorrM=3.0` / `FalseResidFrac=0.25`: false-loop veto. Do NOT lower it to 1.0, because that also rejects the recovery loops of collapsed runs.
  - `MaxGapM=2.0`: defer far loops.
  - `DepthMapFactor=1035`: depth correction, together with SAM depth ×0.96618 and the rig `X_slam_sam_<date>_dmf1035.json` on the PC side.
- evidence (2026-11-01):
  - 1029_etri_library_all, 40 runs in a row passed: ATE 14.9–19.3 cm.
  - 1030_etri_library_all, 40 runs in a row passed: ATE 14.7–23.8 cm, 0 fails.
  - Write-up on the PC: `objpose/docs/loop_fix_261101/README.md` + `orb_slam3_261101.patch`.
- before 2026-11-01 "주력" meant orbslam_ws/src/ORB_SLAM3 + `--gyro` (ours_gyro). That build lacks the 261101 loop fixes, so its results are not the 주력 anymore.

Related: [[openloris-gyro-experiment]], [[rig-external-xsens-gyro]], [[orbslam3-correctloop-race-fix]], [[objpose-remote-hub]]
