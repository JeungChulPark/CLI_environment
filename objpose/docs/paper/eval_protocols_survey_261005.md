# 객체 수준 지도 논문들의 실험 방식 조사 (2026-10-05)

27편의 실험 절을 직접 읽어 정리. 수치는 각 논문 표 그대로. 상세 원자료는 조사 에이전트 보고(대화 기록)와 PDF
(objpose/docs/ppt/related_overview_figs/pdf/) 참조.

## 계열별 데이터셋 · 지표 · 주장
| 계열 | 대표 논문 | 데이터셋 | 정량 지표 (무엇을 재는가) | 주장 |
|---|---|---|---|---|
| 6D 포즈를 장면·지도 수준으로 | MoreFusion, SUO-SLAM, CosyPose, Object-RPE, UnPose | YCB-Video (공통), T-LESS, LM-O | ADD-S AUC, ADD(-S) AUC, e_vsd<0.3 재현율 — 모두 프레임별 포즈 | 여러 시점을 합치면 한 장짜리 추정보다 포즈가 정확하다 |
| 6D 객체 지도 | CT-Map | 자체 (Fetch 로봇, 20시퀀스, 13종) | 시퀀스 끝의 지도 객체 포즈가 위치·회전 임계 안인 비율, 재투영 mAP | 문맥·시간 일관성으로 검출과 포즈가 좋아진다 |
| 객체 수준 SLAM | SLAM++, Fusion++, CubeSLAM | 자체 장면, TUM RGB-D, KITTI | ATE RMSE (카메라 궤적), 3D IoU, 메모리·속도 | 객체를 쓰면 지도가 작아지고 SLAM 이 좋아진다 |
| 인스턴스 의미 지도 | Voxblox++, PanopticFusion, FM-Fusion | SceneNN, ScanNet | 3D 인스턴스 분할 AP50/AP25 (점·복셀 마스크) | 분할 품질이 기존 지도 방식보다 높다 |
| 개방형 어휘 지도 | ConceptGraphs, HOV-SG, OpenVox, DualMap, OVO, OVI-MAP, OpenFusion++ | Replica 8장면 + ScanNet 소수 장면 (사실상 표준), HM3DSem | 3D 의미 분할 mIoU / F-mIoU / mAcc (점 단위), 글 질의 검색 R@k, 주행 성공률 | 학습 없이 다양한 물체를 지도에 올리고 언어로 찾는다 |
| 과제·장면 그래프 | Clio, Hydra, Asset-Centric | 자체 실제 장면, uHumans2 | 객체 단위 재현율·정밀도·F1 (상자 중심 포함 규칙), % Found / % Correct | 실시간으로 쓸모 있는 객체 그래프를 만든다 |
| 변화·장기 지도 | Khronos, POCD, Panoptic Multi-TSDFs, Langer(ObChange) | 자체 시뮬, TorWIC, RIO, ObChange | 변화 정밀도·재현율·F1, 복셀 정밀도·재현율, 네 범주 분류 정확도 | 환경이 바뀌어도 지도가 최신 상태를 유지한다 |

## 확인된 사실
- 객체 지도 자체를 객체 단위로 채점한 논문은 소수이고 표준 지표가 없다: Sünderhauf 2017 (TP/FP/FN 개수), CT-Map, Clio·Asset-Centric (osR/osP/F1), Hydra (% Found/% Correct), ConceptGraphs (사람 평가: 노드 정밀도·중복 수), DualMap (ODR = 예측 객체 수/정답 수).
- 등록 객체의 정밀도·재현율·중복·이름 오류를 한 표에 함께 낸 논문, 6D 방향까지 포함해 지도 객체를 채점한 논문은 조사 범위에 없다.
- 6D 포즈 계열은 장면 수준으로 구한 포즈도 카메라 좌표로 되돌려 프레임별 ADD-S 로 잰다.
- 개방형 어휘 계열은 대부분 정답 카메라 자세로 평가한다. SLAM 자세로 지도 품질을 잰 것은 OVO, Hydra, Khronos.
- 변화를 정량으로 잰 것은 Khronos, POCD, Panoptic Multi-TSDFs, Langer. FM-Fusion·ConceptGraphs·OpenVox·OVI-MAP 은 변화 평가 없음. DualMap 은 재배치 뒤 주행 성공률만.
- YCB 급 소형 물체로 지도를 평가한 것은 Langer(ObChange), DualMap(HM3D+YCB, 주행 성공률)뿐.

## 이 연구에 주는 의미
- 본 실험 = YCB-Video: 6D 포즈 계열의 공통 데이터셋. 프레임별 ADD-S/BOP AR(기존 논문과 같은 지표) + 최종 지도의 객체 단위 지표(등록·중복·이름 오류·중심·방향 오차, 기존에 표준이 없어 직접 정의했음을 밝힘).
- 비교: 6D 계열(CosyPose 다시점, SUO-SLAM — 같은 YCB-V, 코드 공개)과 객체 지도 계열(FM-Fusion, ConceptGraphs).
- ObChange: 객체 지도 비교의 두 번째 데이터셋(소형 물체, 실제 로봇). 변화 판정은 Langer 와 같은 범주·기준으로 보조 결과.
- Replica/ScanNet mIoU 는 이 연구의 주장(CAD 기준 6D 포즈)과 맞지 않는다. 물체가 가구급이고 CAD 가 없다.
