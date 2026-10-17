# Object-level mapping 계보 정리 (조사 261004)

확인 수준: 시작~2022년은 대부분 논문 원문·초록으로 확인. 2023~2024년 논문의 세부 속성은 arXiv 메타데이터만 확인한 일반 지식. 2025~2026년 논문은 arXiv 본문을 요약 모델로 읽은 것이라 인용 전에 원문 대조 필요.

## 1. 시대별 흐름

### 1기 (2000~2008) 의미 지도에서 객체 지도로
- Kuipers "Spatial Semantic Hierarchy"(AI 2000), Galindo 등 "Multi-hierarchical semantic maps"(IROS 2005): 계량 지도 위에 개념 층을 얹음. 객체는 장소를 추론하는 단서.
- 객체 자체를 지도 요소로: Limketkai 등 "Relational Object Maps"(IJCAI 2005), Ekvall 등(IROS 2006), Vasudevan 등 "Cognitive maps—an object based approach"(RAS 2007), Ranganathan & Dellaert "Semantic Modeling of Places using Objects"(RSS 2007), Nüchter & Hertzberg(RAS 2008), Rusu 등 "3D point cloud based object maps"(RAS 2008).
- 객체 표현: 3D 위치, 형상 분포, 평면·기본 도형. 방향 없음.

### 2기 (2007~2013) SLAM에 알려진 물체를 넣다 → SLAM++로 정립
- Castle 등(ICRA 2007), Civera 등 "Towards Semantic SLAM using a Monocular Camera"(IROS 2011): 인식한 기지 물체를 단안 EKF SLAM 지도에 삽입.
- Bao & Savarese "Semantic Structure from Motion"(CVPR 2011): 카메라·점·객체 공동 추정(오프라인).
- **SLAM++**(CVPR 2013): "객체 6자유도 자세 그래프가 곧 지도"라는 틀을 세움. 사전 스캔 메시 모델 필요. 제목 "at the Level of Objects", 본문 "object oriented SLAM".

### 3기 (2014~2017) 사전 모델 제약 풀기, 딥러닝 검출기 결합
- Choudhary 등(IROS 2014): 사전 모델 없이 물체 발견. Gálvez-López 등(RAS 2016): 단안 객체 SLAM. Mu 등(IROS 2016): 딥러닝 검출 + 데이터 연관.
- SemanticFusion(ICRA 2017): 서펠 단위 라벨 융합(객체 단위 아님). Bowman 등(ICRA 2017): 확률적 데이터 연관.
- **Sünderhauf 등 "Meaningful Maps With Object-Oriented Semantic Mapping"(IROS 2017)**: ORB-SLAM2(RGB-D) 자세를 그대로 받아 SSD + 3D 분할로 객체별 점군 지도. 정보가 SLAM → 의미로만 흐르므로 "semantic SLAM"이 아니라 "semantic mapping"이라고 스스로 분류. → 외부 SLAM 자세를 받아 쓰는 object-level mapping의 원형.

### 4기 (2018~2022) 세 갈래로 분화
1. 인스턴스별 밀집 재구성: Fusion++(3DV 2018, 제목에 "Object-Level SLAM"), MaskFusion, MID-Fusion, EM-Fusion, TSDF++. 객체 자세는 볼륨 좌표계 기준이라 의미 있는 방향이 아님.
2. 외부 자세를 받는 인스턴스 볼륨 지도: Voxblox++(RA-L 2019), PanopticFusion(IROS 2019), Panoptic Multi-TSDFs(ICRA 2022). 방향 없음. 자세 오차·드리프트가 한계로 남음.
3. 기하 근사·형상 사전: CubeSLAM, QuadricSLAM, EAO-SLAM, OA-SLAM / NodeSLAM, DSP-SLAM, vMAP(CVPR 2023).
4. 장면 그래프: 3D Scene Graph(ICCV 2019) → 3D Dynamic Scene Graphs(RSS 2020) → Hydra(RSS 2022, 실시간).
5. **6D 자세 추정기 + 다시점/SLAM (소수)**: CT-Map(IROS 2018, 외부 SLAM 자세 + CAD 6D 자세 입자 필터), Object-RPE(ECMR 2019, DenseFusion 출력을 EKF로 융합), MoreFusion(CVPR 2020, ORB-SLAM2 자세 + CAD 6D), CosyPose(ECCV 2020, 객체 BA, 오프라인), Fu 등(IROS 2021, 다중 가설), Merrill 등(CVPR 2022), Lu 등(2022, 자기 학습용). 모두 객체별로 학습된 추정기 전제, 탁상 규모.

### 5기 (2023~2024) 개방형 어휘 전환
- 닫힌 라벨 → CLIP 특징: VLMaps, ConceptFusion, OpenScene, LERF(2023).
- 인스턴스 단위로 묶기: OpenMask3D, OVIR-3D, Open-Fusion.
- 객체 = 그래프 노드: ConceptGraphs(ICRA 2024), HOV-SG(RSS 2024), Clio(RA-L 2024). FM-Fusion(RA-L 2024)은 인스턴스 볼륨 + 확률적 라벨 융합.
- 3DGS에 의미: LangSplat, Gaussian Grouping, OpenGaussian(대부분 오프라인).
- 변화·기억: Khronos, DynaMem, DovSG.

### 6기 (2025~2026.10) 온라인·실시간, 효율·규모
- OVO, OpenVox, OpenFusion++, DualMap, OpenGS-SLAM, OpenGS-Fusion(2025): 점진적 융합과 인스턴스 연관의 강건성.
- OVI-MAP(CVPR 2026, 약 30 FPS), FUS3DMaps(2026, 다층 건물 규모).
- 본문 확인한 개방형 어휘 지도 10편 중 9편이 객체 방향을 저장하지 않음.
- 방향을 다루는 객체 SLAM 쪽(DQO-MAP, PRENOM, CubifyGS 등)은 이차곡면·범주 사전 기반이고 파운데이션 6D 추정기와 연결되지 않음.

## 2. 파운데이션 6D 추정기 + SLAM으로 6D 자세 지도를 유지하는 연구 (2024~2026)
- 정확히 같은 설정의 논문은 찾지 못함(검색 범위 안에서의 결론). SAM-6D를 지도와 결합한 논문은 0건.
- 가장 가까운 것:
  - UnPose(CoRL 2025, 2508.15972): 기성 SLAM 자세 + FoundationPose 정련망 + 객체별 3DGS, 자세 그래프. CAD 없는 조작 대상 객체용.
  - Asset-Centric Metric-Semantic Maps(2510.10778): Go2 + RGB-D, 객체를 메시·범주·SE(3) 자세로 저장. 자세는 전역 정합 + ICP. 프레임당 1.68초, 가구급 물체.
  - MR6D(2508.13775): 이동 로봇 6D 자세 벤치마크(프레임 단위, 지도 없음).

## 3. 용어
- "object level / object-oriented / object-based"가 2013~2017년에 섞여 쓰였고, 제목에 "Object-Level SLAM"이 들어간 것은 Fusion++(2018).
- "object-level mapping"이라는 정확한 어구의 최초 사용처는 확정하지 못함.
- Cadena 등(T-RO 2016)은 정보 흐름으로 구분: "SLAM helps Semantics" / "Semantics helps SLAM" / "Joint". Garg 등(2020 서베이)은 객체 수준 의미 지도를 "사전 3D 모델 사용"과 "사전 모델 없음"으로 구분.

## 4. 이 연구의 위치
- 구조: Sünderhauf 2017 → FM-Fusion 계열(외부 SLAM 자세를 받는 mapping).
- 객체 표현: SLAM++ → CT-Map·Object-RPE·MoreFusion 계열(CAD 기준 6D 자세).
- 새 점: 학습 없이 쓰는 파운데이션 추정기, 이동 플랫폼 실시간, 키프레임 기준 저장으로 루프 폐쇄 반영.
