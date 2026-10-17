# 2. 관련 연구 (2~3쪽 학내 학술대회용 축약본, 260930)

## 2.1 SLAM

SLAM은 로봇이 이동하면서 자신의 위치를 추정하고 동시에 주변 환경의 지도를 작성하는 기술이다. 카메라 기반의 ORB-SLAM3[1]는 특징점을 이용하여 단안, 스테레오, RGB-D 카메라를 모두 지원하며, LiDAR 기반으로는 NDT 정합과 자세 그래프 최적화를 사용하는 hdl_graph_slam[2]과 루프 폐쇄 없이 ICP만으로 동작하는 KISS-ICP[3] 등이 있다. 그래프 기반 SLAM은 루프 폐쇄가 일어나면 과거 키프레임의 자세를 다시 최적화하므로, SLAM 자세를 이용해 지도를 만드는 시스템은 이러한 사후 수정을 반영할 수 있어야 한다.

## 2.2 6자유도 객체 자세 추정

PoseCNN[4], DenseFusion[5]과 같은 초기 학습 기반 방법은 대상 물체마다 별도의 학습이 필요하였다. 최근에는 CAD 모델만 주어지면 학습하지 않은 물체의 자세도 추정할 수 있는 SAM-6D[6], FoundationPose[7] 등이 제안되었다. SAM-6D는 SAM[8]이 생성한 마스크 후보를 CAD 모델에서 렌더링한 템플릿과 비교하여 대상 물체를 찾고, 관측 점군과 모델 점군 사이의 대응으로 자세를 구한다. 그러나 이들은 영상 한 장마다 독립적으로 자세를 구하는 방법이어서 프레임 사이의 일관성을 보장하지 않으며, SAM-6D는 영상 전체에 대한 마스크 생성에 많은 시간이 걸려 이동 중인 로봇에 그대로 적용하기 어렵다.

## 2.3 객체 수준 지도 작성

SLAM++[9]와 Fusion++[10]는 인식한 물체를 SLAM의 랜드마크로 사용하여 카메라와 물체의 자세를 함께 최적화하는 객체 수준 SLAM을 제안하였다. CT-Map[13]과 MoreFusion[14]은 외부 SLAM이 추정한 카메라 자세를 이용하여 CAD 모델이 주어진 객체의 6자유도 자세를 여러 시점에서 추정하고 지도에 유지하였으나, 대상 객체마다 인식 및 자세 추정 네트워크를 학습해야 하였다. 반면 FM-Fusion[11]과 ConceptGraphs[12]는 외부에서 받은 카메라 자세 위에 여러 프레임의 인식 결과를 누적하는 객체 수준 지도 작성 방법으로, 비전-언어 모델을 이용하여 다양한 물체를 지도에 올린다. 하지만 이들은 객체를 점군이나 볼륨으로 표현하기 때문에 물체가 향한 방향은 지도에 남지 않으며, 실험도 주로 데이터셋이 제공하는 정답 카메라 자세에서 이루어졌다. 본 논문은 외부 SLAM의 자세를 사용하는 객체 수준 지도 작성에 속하지만, 각 물체를 6자유도 자세로 저장하고 SLAM 키프레임을 기준으로 관리하여 루프 폐쇄에 따른 수정을 지도에 반영한다는 점에서 기존 연구와 차이가 있다.

---

## 참고문헌 (관련 연구 부분)

[1] C. Campos et al., "ORB-SLAM3: An accurate open-source library for visual, visual-inertial, and multimap SLAM," IEEE T-RO, 2021.
[2] K. Koide, J. Miura, E. Menegatti, "A portable three-dimensional LIDAR-based system for long-term and wide-area people behavior measurement," Int. J. Adv. Robot. Syst., 2019.
[3] I. Vizzo et al., "KISS-ICP: In defense of point-to-point ICP," IEEE RA-L, 2023.
[4] Y. Xiang et al., "PoseCNN: A convolutional neural network for 6D object pose estimation in cluttered scenes," RSS, 2018.
[5] C. Wang et al., "DenseFusion: 6D object pose estimation by iterative dense fusion," CVPR, 2019.
[6] J. Lin et al., "SAM-6D: Segment anything model meets zero-shot 6D object pose estimation," CVPR, 2024.
[7] B. Wen et al., "FoundationPose: Unified 6D pose estimation and tracking of novel objects," CVPR, 2024.
[8] A. Kirillov et al., "Segment anything," ICCV, 2023.
[9] R. F. Salas-Moreno et al., "SLAM++: Simultaneous localisation and mapping at the level of objects," CVPR, 2013.
[10] J. McCormac et al., "Fusion++: Volumetric object-level SLAM," 3DV, 2018.
[11] C. Liu et al., "FM-Fusion: Instance-aware semantic mapping boosted by vision-language foundation models," IEEE RA-L, 2024.
[12] Q. Gu et al., "ConceptGraphs: Open-vocabulary 3D scene graphs for perception and planning," ICRA, 2024.
[13] Z. Zeng, Y. Zhou, O. C. Jenkins, K. Desingh, "Semantic mapping with simultaneous object detection and localization," IROS, 2018. (CT-Map)
[14] K. Wada, E. Sucar, S. James, D. Lenton, A. J. Davison, "MoreFusion: Multi-object reasoning for 6D pose estimation from volumetric fusion," CVPR, 2020.
