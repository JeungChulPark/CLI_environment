# 논문 실험·비교 대상 정리 (261004)

## 1. 꼭 들어가야 하는 실험 (주장 ↔ 실험)

| # | 주장 | 실험 | 상태 |
|---|---|---|---|
| E1 | 중심·방향을 가진 객체 지도를 중복 없이 만든다 | YCB-V 12영상 객체 지도 비교 (등록 수, 중복, 이름 오류, 중심 오차, 방향 오차) vs FM-Fusion, SAM-6D | 완료 (fmf_cmp_260928) |
| E2 | 이동 중 처리 가능한 속도 | 한 장당 처리 시간, 처리 프레임 수 | 완료 (1,450 → 386 ms) |
| E3 | 지도에 저장하면 늦게 온 결과도 화면에 맞는다 | 화면 일치율: 결과 유지 vs 지도 표시 | 완료 (43.3 → 85.7% 등) |
| E4 | 시야를 벗어났다 돌아와도 같은 객체로 연결 | 재연결 건수: YCB-M 101건, 실제 주행 3회 109건 | 완료 (실제 주행은 녹화 입력 재생) |
| E5 | 키프레임 기준 저장으로 루프 폐쇄 반영 | 보정 전후 같은 객체 위치 차 | 1회만 측정 (5.4 → 0.8 cm) → 다른 주행에서 반복 필요 |
| E6 | 검증 단계가 오인식을 줄인다 | 단계별 F1·오검출 (959칸) | 완료 |
| E7 | 프레임 단위 포즈 정확도가 떨어지지 않는다 | BOP AR (YCB-V) | 완료 |
| E8 | 다른 데이터에서도 같은 경향 | YCB-M 32장면 | 완료 |
| E9 | SLAM 자세 오차의 영향 | 정답 카메라 자세 vs ORB-SLAM3 자세로 만든 지도 비교 | 문서 있음 (ycbv_slam_vs_gt_260921) → 표로 정리 필요 |

주장하려면 새로 해야 하는 실험
- E10 객체 변화 대응: 주행 중 물체를 치우거나 옮긴 뒤 삭제·재등록 성공 건수와 걸린 시간. "이동·사라짐 감지"를 기여로 쓰려면 필수.
- E11 SLAM 교체: 같은 주행을 ORB-SLAM3와 hdl_graph_slam으로 돌려 객체 지도 차이 비교. "SLAM을 바꿔도 된다"를 쓰려면 필요.
- E12 추정기 교체: FoundationPose를 같은 Object Pose Map에 연결. "특정 추정기에 묶이지 않는다"를 쓰려면 필요.

분량별
- 콜로키움 2~3쪽: E1, E2 + (E3·E4·E5를 3행 표로).
- 학위논문: E1~E9 전부 + E10(권장), E11·E12(선택).

## 2. GitHub 코드로 비교할 알고리즘

### 객체 지도 수준
| 알고리즘 | 저장소 | 비교 지점 | 난이도·주의 |
|---|---|---|---|
| FM-Fusion (RA-L 2024) | HKUST-Aerial-Robotics/FM-Fusion | 최근 객체 지도 기준선 | 완료 |
| ConceptGraphs (ICRA 2024) | concept-graphs/concept-graphs | 가장 많이 인용되는 개방형 객체 지도. 객체 점군·상자 → 중심 오차·중복·이름 오류 비교 | 중. posed RGB-D 입력이라 FM-Fusion용 변환 재사용 가능. LLM 캡션 단계는 생략 가능 |
| DualMap (RA-L 2025) | Eku127/DualMap | 최신 온라인 객체 지도, 변하는 장면 갱신 포함 → E10의 비교 상대 | 중. 데이터셋 모드·ROS 모드 지원 |
| SUO-SLAM (CVPR 2022) | rpng/suo_slam | 같은 YCB-Video에서 6D 포즈를 전역으로 유지하는 유일한 공개 비교 상대. 사전 학습 모델 제공 | 중. 물체별 학습 방식이라 "학습 필요 vs 불필요" 대비 |
| MoreFusion (CVPR 2020) | wkentaro/morefusion | 외부 SLAM + CAD 6D 포즈 지도 | 상. Ubuntu 16.04·ROS Kinetic, 2023년 보관 처리. 실행 대신 인용만 권장 |
| CosyPose 다시점 (ECCV 2020) | agimus-project/happypose | 다시점 6D 포즈 정합(오프라인 상한) | 중. YCB-V 학습 모델 |

### 포즈 추정기 수준 (같은 Object Pose Map에 연결)
| 알고리즘 | 저장소 | 비교 지점 |
|---|---|---|
| SAM-6D (CVPR 2024) | JiehongLin/SAM-6D | 완료 |
| FoundationPose (CVPR 2024) | NVlabs/FoundationPose | E12. 대표적인 학습 없는 추정기 |
| MegaPose (CoRL 2022) | agimus-project/happypose | 선택 |

### 코드가 없어 실행 비교가 어려운 것
- Fusion++: 공식 코드 없음(FM-Fusion 논문이 자체 재구현으로 비교).
- CT-Map: 공식 코드 확인 못 함.
- SLAM++: 공식 코드 없음.

## 3. 우선순위
1. E5 반복 측정 (기존 주행 재사용, 반나절)
2. ConceptGraphs를 YCB-V에 실행 (E1 표에 한 행 추가)
3. E10 객체 변화 실험 (실제 주행 새로 촬영)
4. SUO-SLAM YCB-V 실행
5. FoundationPose 연결 (E12)
