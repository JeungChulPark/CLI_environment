# sensor_sync_ws — 단일머신 검증 실행 파일 세트 (ROS2 Jazzy)

> PC에서 설계 → 이 폴더를 노트북에 통째로 복사 → V0~V8 검증 실행.
> 설계도(런북): `../_bmad_output_for_slam/implementation-artifacts/blueprint-sensor-sync-validation-jazzy.md`
> 대상: Ubuntu 24.04 + ROS2 Jazzy, 현재 사이트 하드웨어(노트북 1 + D435i + VLP-16).

## 폴더 구조
```
sensor_sync_ws/
├── README.md                     # 이 파일
├── launch/bringup_A.launch.py    # D435i + VLP-16 통합 기동
├── config/
│   ├── realsense_A.yaml          # realsense 파라미터(global_time 등)
│   └── qos_override.yaml         # record 드롭방지 QoS
├── scripts/
│   ├── check_env.sh              # V0: OS/ROS/NIC PHC 점검
│   ├── record_A.sh               # V3/V4: MCAP 기록
│   ├── check_global_time.sh      # V1/V6: stamp vs 시스템시각
│   └── verify_drops.sh           # V4: bag info 드롭 점검
├── time_sync/                    # 2대 통합 시 사용(지금은 준비만)
│   ├── chrony_A_server.conf
│   └── chrony_B_client.conf
└── metadata_template.yaml
```

## 빠른 시작 (노트북에서)
```bash
# 0) 환경 점검 (iface는 실제 이더넷 인터페이스명)
bash scripts/check_env.sh <iface>

# 1) 센서 통합 기동 (별도 터미널, source /opt/ros/jazzy/setup.bash 후)
ros2 launch launch/bringup_A.launch.py

# 2) 토픽 확인
ros2 topic hz /cam_A/color/image_raw
ros2 topic hz /velodyne_points
bash scripts/check_global_time.sh        # global_time 도메인 확인

# 3) 기록 (5~10분)
bash scripts/record_A.sh

# 4) 드롭 점검
bash scripts/verify_drops.sh session/<생성된_bag_폴더>
```

## ⚠️ 실기 확인 필요 (파일 내 `# TODO(실기)` 표시)
- realsense 파라미터명은 realsense-ros 버전차 있음(`depth_profile` vs `profile`) → `ros2 param list /cam_A/camera`로 실제명 확인 후 `config/realsense_A.yaml` 조정.
- VLP-16 IP/서브넷(기본 192.168.1.201) 웹UI로 확인 후 노트북 IP 맞춤.
- 토픽명(`aligned_depth_to_color` 등) 실제 발행명 확인.
- 이 폴더는 빌드 필요 없는 **loose 실행 파일 세트**다(colcon 패키지 아님). launch는 apt 설치된 realsense2_camera/velodyne를 참조.
