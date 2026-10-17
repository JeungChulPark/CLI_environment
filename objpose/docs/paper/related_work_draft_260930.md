# 제2장 관련 연구 (초안 260930)

본 장에서는 제안하는 시스템을 이해하는 데 필요한 기술적 배경과 관련 연구를 정리한다. 2.1절에서는 카메라와 LiDAR 기반의 동시적 위치 추정 및 지도 작성 기법, 6자유도 객체 자세 추정 기법, 그리고 본 논문에서 객체 후보를 얻는 데 사용한 개방형 어휘 검출 및 분할 기법을 차례로 설명한다. 2.2절에서는 SLAM과 객체 인식을 결합하여 객체 단위의 지도를 만드는 기존 연구를 객체 수준 SLAM과 객체 수준 지도 작성으로 나누어 살펴보고, 본 연구와의 차이를 비교한다.

## 2.1 기술 및 이론적 배경

### 2.1.1 동시적 위치 추정 및 지도 작성(SLAM)

**가. 개념과 구성**

동시적 위치 추정 및 지도 작성(Simultaneous Localization and Mapping, SLAM)은 로봇이 사전 정보가 없는 환경을 이동하면서 센서 데이터로부터 자신의 위치를 추정하고, 동시에 주변 환경의 지도를 작성하는 문제이다. 위치 추정에는 지도가 필요하고 지도를 만들기 위해서는 정확한 위치가 필요하기 때문에, 두 문제를 하나로 묶어 함께 푸는 방식이 오래전부터 연구되어 왔다.

최근의 SLAM 시스템은 대부분 프론트엔드(front-end)와 백엔드(back-end)로 나뉜다. 프론트엔드는 연속된 센서 입력 사이의 대응 관계를 찾아 센서의 상대 움직임을 추정하며, 이 과정을 오도메트리(odometry)라고 부른다. 백엔드는 프론트엔드가 만든 추정값을 제약 조건으로 삼아 전체 궤적과 지도를 최적화한다. 초기 연구는 확장 칼만 필터(EKF)와 같은 필터링 방법을 주로 사용하였으나, 이후에는 센서 자세를 노드로, 자세 간 상대 변환을 간선으로 하는 그래프를 구성하고 이를 비선형 최소제곱 문제로 푸는 그래프 기반 방법이 널리 쓰이고 있다[1, 2].

모든 프레임을 최적화에 넣으면 계산량이 커지므로, 대부분의 시스템은 시점 변화가 충분한 일부 프레임만 키프레임(keyframe)으로 선택하여 지도에 남긴다. 또한 로봇이 이전에 지나간 장소로 돌아왔을 때 이를 알아차리는 루프 폐쇄(loop closure)가 일어나면, 그동안 누적된 오차를 줄이기 위해 과거 키프레임들의 자세가 한꺼번에 수정된다. 즉 SLAM이 내놓는 자세는 한 번 출력되면 고정되는 값이 아니고, 이후의 관측에 따라 과거 값까지 바뀔 수 있다. 이 성질은 SLAM 자세를 이용해 다른 정보를 지도에 올리는 모든 시스템이 고려해야 하는 부분이며, 본 논문의 3장에서 다시 다룬다.

**나. 카메라 기반 SLAM**

카메라 기반 SLAM(Visual SLAM)은 영상에서 얻은 정보를 이용하는 방식이다. MonoSLAM[3]은 단안 카메라 하나로 실시간 SLAM을 구현한 초기 연구로, EKF를 이용해 카메라 자세와 특징점 위치를 함께 추정하였다. 그러나 특징점 수가 늘어날수록 상태 벡터가 커져 넓은 환경으로 확장하기 어려웠다. PTAM[4]은 추적(tracking)과 지도 작성(mapping)을 별도의 스레드로 분리하고, 지도 작성 쪽에서는 키프레임에 대해 번들 조정(bundle adjustment)을 수행하는 구조를 제안했다. 이 구조는 이후 대부분의 특징점 기반 SLAM에 그대로 이어졌다.

ORB-SLAM[5]은 PTAM의 구조에 루프 폐쇄와 재위치 추정(relocalization)을 더하고, 추적, 지역 지도 작성, 루프 폐쇄의 세 스레드가 모두 ORB 특징을 공유하도록 설계하였다. ORB-SLAM2[6]는 이를 스테레오와 RGB-D 카메라로 확장했으며, ORB-SLAM3[7]는 관성 측정 장치(IMU)를 결합한 시각-관성 SLAM과 여러 개의 지도를 관리하는 다중 지도(Atlas) 기능을 추가하였다. 특징점 대신 픽셀 밝기를 직접 이용하는 직접 방식(direct method)으로는 LSD-SLAM[8]과 DSO[9]가 대표적이다. 직접 방식은 특징이 적은 영역에서도 정보를 활용할 수 있지만, 조명 변화와 카메라 노출 변화에 민감하다는 한계가 있다.

RGB-D 카메라가 보급된 이후에는 깊이 영상을 이용해 밀집(dense) 지도를 만드는 연구도 활발히 이루어졌다. KinectFusion[10]은 깊이 영상을 절단 부호 거리 함수(Truncated Signed Distance Function, TSDF) 복셀에 누적하고, ICP(Iterative Closest Point)로 카메라 자세를 추적하였다. 하지만 고정된 크기의 복셀 공간을 사용하고 루프 폐쇄가 없어 넓은 공간에는 적용하기 어려웠다. ElasticFusion[11]은 서펠(surfel) 기반 지도와 지도 변형을 이용해 루프 폐쇄를 반영하였고, RTAB-Map[12]은 메모리 관리 기법을 통해 장시간 동작하는 그래프 기반 RGB-D SLAM을 구현하였다.

**다. LiDAR 기반 SLAM**

LiDAR는 조명 변화의 영향을 받지 않고 수십 m 범위까지 정확한 거리를 측정할 수 있어, 실외나 넓은 실내 환경의 SLAM에 많이 쓰인다. LOAM[13]은 스캔에서 모서리(edge)와 평면(planar) 특징점을 추출하고, 빠른 오도메트리와 느린 지도 정합을 서로 다른 주기로 수행하여 실시간성과 정확도를 함께 확보하였다. 이후 LIO-SAM[14]은 IMU 사전적분과 LiDAR 오도메트리, 루프 폐쇄를 인자 그래프(factor graph)로 묶었으며, FAST-LIO2[15]는 특징 추출 없이 원시 점을 반복 확장 칼만 필터로 지도에 직접 정합하는 방식을 제안했다.

KISS-ICP[16]는 복잡한 특징 추출이나 센서별 파라미터 조정 없이, 등속 운동 모델에 따른 스캔 왜곡 보정과 점-점 ICP, 적응형 임계값만으로 구성된 LiDAR 오도메트리이다. 구조가 단순하고 다양한 센서에 그대로 적용된다는 장점이 있으나, 루프 폐쇄가 없는 오도메트리이므로 주행 거리가 길어지면 오차가 누적된다. hdl_graph_slam[17]은 NDT(Normal Distributions Transform) 기반 스캔 정합으로 오도메트리를 구하고, 키프레임 사이의 상대 자세, 바닥 평면, 루프 폐쇄를 제약으로 하는 자세 그래프를 g2o[2]로 최적화하는 그래프 기반 LiDAR SLAM이다.

본 논문에서는 RGB-D 카메라 입력에 대해 ORB-SLAM3를, LiDAR 입력에 대해 hdl_graph_slam과 KISS-ICP를 사용하였다. 세 방법은 루프 폐쇄와 키프레임의 유무가 서로 달라, 루프 폐쇄에 따른 자세 수정이 객체 지도에 어떤 영향을 주는지 비교하는 데 활용하였다.

### 2.1.2 6자유도 객체 자세 추정

**가. 문제 정의**

6자유도 객체 자세 추정(6D object pose estimation)은 영상에 나타난 물체의 3차원 회전과 이동을 카메라 좌표계 기준으로 구하는 문제이다. 물체 좌표계에서 카메라 좌표계로의 변환은 식 (2.1)과 같이 회전 행렬 $R \in SO(3)$과 이동 벡터 $t \in \mathbb{R}^3$로 이루어진 동차 변환 행렬로 표현된다.

$$
T_{co} = \begin{bmatrix} R & t \\ \mathbf{0}^\top & 1 \end{bmatrix} \in SE(3) \tag{2.1}
$$

자세 추정 방법은 대상 물체를 어떻게 다루는지에 따라 크게 세 가지로 구분할 수 있다. 인스턴스 수준(instance-level) 방법은 학습에 사용한 특정 물체에 대해서만 자세를 추정한다. 범주 수준(category-level) 방법은 NOCS[18]와 같이 같은 범주에 속하는 처음 보는 물체까지 다루지만, 범주마다 학습 데이터가 필요하다. 마지막으로 새로운 물체(novel/unseen object) 대상 방법은 추론 시점에 CAD 모델이나 몇 장의 참조 영상만 주어지면 추가 학습 없이 자세를 추정한다.

**나. 학습 기반 인스턴스 수준 자세 추정**

딥러닝 이전에는 LINEMOD[19]와 같은 템플릿 정합이나 점 쌍 특징(Point Pair Feature, PPF)[20]을 이용한 기하 정합 방법이 주로 사용되었다. PoseCNN[21]은 합성곱 신경망으로 물체의 분할, 중심 위치, 회전을 각각 예측하는 구조를 제안하였고, 이 연구에서 함께 공개한 YCB-Video 데이터셋은 이후 자세 추정 연구의 표준 평가 데이터 중 하나가 되었다. DenseFusion[22]은 색상 영상과 깊이 점군에서 각각 특징을 뽑아 픽셀 단위로 결합하였으며, PVN3D[23]와 FFB6D[24]는 물체 위의 3차원 키포인트를 예측한 뒤 최소제곱 정합으로 자세를 구하는 방식으로 정확도를 높였다. 그러나 이들 방법은 대상 물체마다 학습 데이터를 만들고 네트워크를 다시 학습해야 하므로, 새로운 물체를 추가할 때마다 비용이 크게 든다.

**다. 새로운 물체 대상 자세 추정**

CosyPose[25]는 렌더링한 영상과 입력 영상을 비교하며 자세를 반복 보정하는 렌더-비교(render-and-compare) 방식을 사용하였고, 여러 시점의 결과를 모아 장면 전체에서 일관된 자세를 구하는 방법도 함께 제시하였다. MegaPose[26]는 대규모 합성 데이터로 학습하여 이 방식을 학습에 쓰지 않은 물체로 확장했다. 최근에는 SAM[27], DINOv2[28]와 같은 대규모 사전학습 모델을 활용하는 방법이 많이 제안되고 있다. CNOS[29]는 SAM으로 영상 내 모든 영역의 마스크를 만든 뒤, DINOv2 특징을 이용해 CAD 모델에서 렌더링한 템플릿과 비교하여 대상 물체를 찾아내었다.

SAM-6D[30]는 이러한 흐름을 자세 추정까지 이어서, 객체 분할 모델(Instance Segmentation Model, ISM)과 자세 추정 모델(Pose Estimation Model, PEM)의 두 단계로 구성하였다. ISM은 SAM이 생성한 마스크 후보마다 템플릿과의 의미 유사도, 외형 유사도, 기하 일치도를 계산하여 대상 물체를 고른다. PEM은 관측 점군과 CAD 모델 점군 사이의 대응점을 두 단계로 찾는다. 대략적 단계(coarse)에서는 대응점으로부터 가중 SVD로 초기 자세를 구하고, 정밀 단계(fine)에서는 초기 자세를 기준으로 더 많은 점을 사용하여 자세를 다듬는다. 템플릿은 CAD 모델을 여러 시점에서 미리 렌더링하여 만들어 두므로, 새로운 물체를 추가할 때 네트워크 학습이 필요 없다. FoundationPose[31]는 CAD 모델이 있는 경우와 참조 영상만 있는 경우를 하나의 구조로 다루며 자세 추정과 추적을 함께 수행하였고, GigaPose[32]는 템플릿 정합과 패치 대응으로 추론 속도를 높였다.

이들 방법은 학습에 쓰지 않은 물체에서도 높은 정확도를 보이지만, 모두 한 장의 영상에서 한 번에 자세를 구하는 프레임 단위 추정기이다. 따라서 카메라가 움직이는 동안의 시간적 일관성이나 여러 프레임 결과의 누적은 고려하지 않는다. 또한 SAM-6D의 경우 영상 전체에 대해 마스크를 생성하는 단계가 전체 처리 시간의 대부분을 차지하여, 이동 중인 로봇에서 연속된 영상을 처리하기에는 속도가 부족하다.

**라. 평가 지표**

자세 추정 성능은 추정 자세와 정답 자세로 각각 변환한 물체 모델 점들 사이의 평균 거리로 평가하는 ADD와, 대칭 물체에 대해 가장 가까운 점과의 거리를 쓰는 ADD-S가 오래 사용되었다[21]. BOP 벤치마크[33]는 여러 데이터셋의 평가 방식을 통일하면서 VSD(Visible Surface Discrepancy), MSSD(Maximum Symmetry-Aware Surface Distance), MSPD(Maximum Symmetry-Aware Projection Distance)의 세 가지 오차를 정의하였고, 각 오차에 대한 재현율을 평균한 AR(Average Recall)을 대표 지표로 사용한다. 본 논문의 5장에서도 프레임 단위 자세 정확도는 BOP AR로 측정하였다.

### 2.1.3 개방형 어휘 객체 검출과 분할

기존 객체 검출기는 학습에 포함된 범주만 검출할 수 있었으나, 최근에는 영상과 문장을 함께 학습한 모델을 이용해 문장으로 지정한 대상을 찾는 개방형 어휘(open-vocabulary) 검출 연구가 활발히 진행되고 있다. Grounding DINO[34]는 트랜스포머 기반 검출기에 문장 인코더를 결합하여 입력 문구에 해당하는 영역을 찾으며, YOLO-World[35]는 YOLO 계열 검출기에 문장 임베딩을 결합하여 실시간에 가까운 속도로 개방형 어휘 검출을 수행하였다.

SAM[27]은 점이나 상자 같은 프롬프트를 입력받아 해당 물체의 마스크를 생성하는 분할 모델로, 대규모 데이터로 학습되어 학습에 없던 물체에도 적용할 수 있다. 다만 영상 인코더가 무거워 실시간 처리에는 부담이 크다. MobileSAM[36]은 SAM의 영상 인코더를 지식 증류(knowledge distillation)로 경량화하여, SAM과 같은 프롬프트 방식을 유지하면서 처리 시간을 크게 줄였다. 본 논문에서는 SAM-6D의 전 영역 마스크 생성 단계를 YOLO-World의 문구 검출과 MobileSAM의 상자 프롬프트 분할로 바꾸어 객체 후보를 얻었으며, 두 모델은 수정 없이 사용하였다.

## 2.2 관련 연구

SLAM과 객체 인식을 결합하는 연구는 두 방향으로 나누어 볼 수 있다. 하나는 객체를 SLAM의 랜드마크로 사용하여 카메라 자세와 객체 자세를 함께 최적화하는 객체 수준 SLAM(object-level SLAM)이다. 다른 하나는 SLAM이 추정한 카메라 자세를 입력으로 받아, 여러 프레임에서 인식한 객체를 하나의 지도에 모으는 객체 수준 지도 작성(object-level mapping)이다. 본 연구는 후자에 속한다.

### 2.2.1 객체 수준 SLAM

SemanticFusion[37]은 ElasticFusion이 만든 서펠 지도에 합성곱 신경망의 픽셀 단위 의미 분할 결과를 확률적으로 누적하여 의미 지도를 만들었다. 그러나 지도의 단위가 여전히 서펠이어서, 같은 범주의 서로 다른 물체를 구분하거나 물체 단위로 다루지는 못했다.

객체를 지도의 기본 단위로 처음 사용한 연구는 SLAM++[38]이다. SLAM++는 미리 준비한 CAD 모델 데이터베이스로 장면의 물체를 인식하고, 카메라와 물체 사이의 6자유도 상대 자세를 제약으로 하는 객체 그래프를 자세 그래프 최적화로 갱신하였다. 이를 통해 밀집 지도보다 훨씬 작은 표현으로 루프 폐쇄와 재위치 추정을 수행할 수 있음을 보였다. 하지만 인식 가능한 물체가 데이터베이스에 있는 몇 종류로 제한되었고, 물체 인식 역시 깊이 기반의 기하 정합에 의존하였다.

Fusion++[39]는 Mask R-CNN의 인스턴스 분할 결과로 물체마다 별도의 TSDF 볼륨을 만들고, 이 객체들을 6자유도 자세 그래프에 저장하여 추적, 재위치 추정, 루프 폐쇄에 사용하였다. MaskFusion[40]과 MID-Fusion[41]은 인스턴스 분할과 밀집 재구성을 결합하여 움직이는 물체까지 각각 추적하였다. 이들 방법은 CAD 모델 없이 임의의 물체를 재구성할 수 있지만, 물체의 자세가 재구성할 때 정한 임의의 좌표계를 기준으로 하므로 물체가 실제로 어느 방향을 향하고 있는지는 알 수 없다.

단안 카메라 환경에서는 물체를 단순한 기하 형상으로 근사하는 연구가 진행되었다. CubeSLAM[42]은 2차원 검출 상자로부터 3차원 직육면체를 생성하여 카메라, 물체, 특징점을 함께 번들 조정하였고, QuadricSLAM[43]은 물체를 이차 곡면(quadric)으로 표현하여 검출 상자와의 일치를 제약으로 사용하였다. DSP-SLAM[44]은 범주별 형상 사전 정보를 이용해 물체의 자세와 형상을 함께 추정하였다. 이러한 근사 표현은 물체의 크기와 대략적인 방향을 제공하지만, 정밀한 6자유도 자세를 얻기에는 부족하다.

6자유도 자세 추정과 SLAM을 직접 결합한 연구로는 Merrill 등[45]의 연구가 있다. 이 연구는 물체 키포인트를 예측하는 네트워크의 불확실도를 최적화의 가중치로 사용하고, 대칭 물체에 대해서는 SLAM이 추정한 카메라 자세를 사전 정보로 활용하여 카메라와 물체 자세를 함께 추정하였다. 다만 물체별로 키포인트 네트워크를 학습해야 하므로 새로운 물체를 추가하려면 학습 데이터가 필요하다.

위의 연구들은 모두 객체 관측을 SLAM의 제약 조건으로 사용하여 카메라 자세 추정 자체를 개선하는 데 중점을 두었다. 이 경우 객체 인식의 오차가 카메라 자세에 그대로 전달될 수 있으며, 특정 SLAM 구조에 객체 처리가 결합되어 다른 SLAM으로 바꾸어 쓰기 어렵다는 한계가 있다.

### 2.2.2 객체 수준 지도 작성

객체 수준 지도 작성 연구는 카메라 자세를 외부 SLAM이나 데이터셋에서 받아 오고, 여러 프레임의 인식 결과를 같은 객체끼리 연관(association)하여 지도에 누적하는 데 집중한다. Sünderhauf 등[46]은 ORB-SLAM2의 카메라 자세 위에서 검출된 물체의 점군을 누적하는 객체 지향 의미 지도를 제안하였다. 이 방식은 SLAM과 객체 처리를 분리하여, SLAM은 기존 방법을 그대로 사용하고 객체 처리만 따로 개선할 수 있게 한다.

2023년 이후에는 비전-언어 기반 모델을 사용하는 연구가 주를 이루고 있다. FM-Fusion[47]은 RAM, Grounding DINO, SAM을 이용해 영상에서 개방형 어휘 인스턴스를 얻고, 확률적 라벨 융합으로 각 인스턴스의 범주를 정한 뒤 인스턴스별 볼륨으로 지도를 구성하였다. 이 연구는 Fusion++를 비교 대상으로 삼아 ScanNet 인스턴스 분할 성능을 크게 높였으나, 실험에서는 데이터셋이 제공하는 카메라 자세를 사용하였다. ConceptGraphs[48]는 2차원 분할 결과를 3차원으로 올려 객체 단위의 점군과 CLIP 특징을 만들고, 객체 사이의 관계를 대규모 언어 모델로 추론하여 3차원 장면 그래프(3D scene graph)를 구성하였다. 실제 환경 실험에서는 RTAB-Map의 카메라 자세를 사용하였다. HOV-SG[49]는 이를 층, 방, 객체의 계층 구조로 확장하였고, 실제 로봇 실험에서 FAST-LIO2 기반 LiDAR SLAM의 자세에 외부 파라미터를 적용해 카메라 자세를 얻었다. Clio[50]는 작업 설명과 관련된 객체만 남기는 장면 그래프를 제안하였으며, 로봇에 내장된 오도메트리를 입력으로 사용하였다.

지도 표현을 복셀이나 가우시안으로 바꾼 연구도 있다. OpenVox[51]는 인스턴스 단위의 확률적 복셀 지도를 실시간으로 갱신하였고, OpenGS-Fusion[52]은 3차원 가우시안 스플래팅(3D Gaussian Splatting)과 절단 부호 거리 함수를 함께 사용하여 객체 수준의 세밀한 지도를 만들었다. 두 연구 모두 실제 로봇 실험에서는 각각 LiDAR SLAM과 ORB-SLAM3의 자세를 사용하였다.

이처럼 객체 수준 지도 작성 연구들은 지도에 올리는 대상을 개방형 어휘로 넓히는 데 중점을 두었다. 그러나 객체를 점군, 복셀, 경계 상자 등으로 표현하기 때문에 물체의 위치와 형태는 알 수 있어도 물체가 향한 방향, 즉 6자유도 자세는 지도에 남지 않는다. 또한 Hydra[53]처럼 자체적으로 루프 폐쇄를 수행하는 경우를 제외하면, 대부분 카메라 자세를 한 번 받은 뒤 고정된 값으로 취급한다. 성능 평가도 주로 정답 카메라 자세가 주어진 데이터셋에서 이루어져, 실제 SLAM 자세의 오차나 루프 폐쇄에 따른 자세 수정이 객체 지도에 미치는 영향은 거의 다루지 않았다.

### 2.2.3 본 연구와의 비교

표 2-1은 앞에서 살펴본 대표적인 연구와 본 연구를 카메라 자세의 출처, 객체 표현, 객체 방향 정보, 지도 보정의 반영 여부로 비교한 것이다.

**표 2-1. 객체 단위 지도 관련 연구 비교**

| 방법 | 분류 | 카메라 자세 출처 | 객체 표현 | 객체 방향 | 루프 폐쇄에 따른 보정 |
|---|---|---|---|---|---|
| SLAM++[38] | 객체 수준 SLAM | 자체 추정 | CAD 모델 | O | O (객체 그래프 최적화) |
| Fusion++[39] | 객체 수준 SLAM | 자체 추정 | 객체별 TSDF | △ (재구성 좌표계 기준) | O (자세 그래프 최적화) |
| Merrill 등[45] | 객체 수준 SLAM | 자체 추정 | 키포인트 | O | O (공동 최적화) |
| FM-Fusion[47] | 객체 수준 지도 작성 | 데이터셋 제공 자세 | 인스턴스 볼륨 | X | X |
| ConceptGraphs[48] | 3차원 장면 그래프 | 데이터셋 제공 자세, RTAB-Map | 점군과 특징 벡터 | X | X |
| HOV-SG[49] | 3차원 장면 그래프 | 시뮬레이터 정답 자세, FAST-LIO2 | 점군과 특징 벡터 | X | X (외부 SLAM에서 완료) |
| Hydra[53] | 3차원 장면 그래프 | Kimera-VIO | 메시와 객체 노드 | X | O (자체 루프 폐쇄) |
| 본 연구 | 객체 수준 지도 작성 | 외부 SLAM (ORB-SLAM3, hdl_graph_slam 등) | CAD 기반 6자유도 자세 | O | O (키프레임 기준 저장) |

본 연구는 SLAM을 수정하지 않고 외부 SLAM의 자세를 받아 쓴다는 점에서 객체 수준 지도 작성 연구와 구조가 같다. 하지만 지도에 저장하는 단위가 점군이나 복셀이 아니라 CAD 모델 기준의 6자유도 자세라는 점에서 차이가 있다. 이를 위해 새로운 물체에 대해 학습 없이 동작하는 SAM-6D를 자세 추정기로 사용하되, 이동 중인 카메라에서 연속으로 처리할 수 있도록 후보 생성 단계를 경량화하고, 비슷한 물체를 잘못 인식한 결과가 지도에 들어가지 않도록 후보 검증과 자세 검증 단계를 추가하였다. 또한 객체 관측을 월드 좌표가 아닌 해당 시점의 SLAM 키프레임을 기준으로 저장하여, 루프 폐쇄로 키프레임 자세가 수정되면 객체 위치도 함께 갱신되도록 하였다. 본 논문에서는 이렇게 만든 지도를 객체 자세 지도(Object Pose Map)라고 부르며, 3장에서 그 구성을 자세히 설명한다.

---

## 참고문헌 (초안, 저자·권호 최종 확인 필요)

[1] G. Grisetti, R. Kümmerle, C. Stachniss, W. Burgard, "A tutorial on graph-based SLAM," IEEE Intelligent Transportation Systems Magazine, 2010.
[2] R. Kümmerle et al., "g2o: A general framework for graph optimization," ICRA, 2011.
[3] A. J. Davison et al., "MonoSLAM: Real-time single camera SLAM," IEEE TPAMI, 2007.
[4] G. Klein, D. Murray, "Parallel tracking and mapping for small AR workspaces," ISMAR, 2007.
[5] R. Mur-Artal, J. M. M. Montiel, J. D. Tardós, "ORB-SLAM: A versatile and accurate monocular SLAM system," IEEE T-RO, 2015.
[6] R. Mur-Artal, J. D. Tardós, "ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras," IEEE T-RO, 2017.
[7] C. Campos et al., "ORB-SLAM3: An accurate open-source library for visual, visual-inertial, and multimap SLAM," IEEE T-RO, 2021.
[8] J. Engel, T. Schöps, D. Cremers, "LSD-SLAM: Large-scale direct monocular SLAM," ECCV, 2014.
[9] J. Engel, V. Koltun, D. Cremers, "Direct sparse odometry," IEEE TPAMI, 2018.
[10] R. A. Newcombe et al., "KinectFusion: Real-time dense surface mapping and tracking," ISMAR, 2011.
[11] T. Whelan et al., "ElasticFusion: Dense SLAM without a pose graph," RSS, 2015.
[12] M. Labbé, F. Michaud, "RTAB-Map as an open-source lidar and visual SLAM library for large-scale and long-term online operation," Journal of Field Robotics, 2019.
[13] J. Zhang, S. Singh, "LOAM: Lidar odometry and mapping in real-time," RSS, 2014.
[14] T. Shan et al., "LIO-SAM: Tightly-coupled lidar inertial odometry via smoothing and mapping," IROS, 2020.
[15] W. Xu et al., "FAST-LIO2: Fast direct lidar-inertial odometry," IEEE T-RO, 2022.
[16] I. Vizzo et al., "KISS-ICP: In defense of point-to-point ICP," IEEE RA-L, 2023.
[17] K. Koide, J. Miura, E. Menegatti, "A portable three-dimensional LIDAR-based system for long-term and wide-area people behavior measurement," International Journal of Advanced Robotic Systems, 2019.
[18] H. Wang et al., "Normalized object coordinate space for category-level 6D object pose and size estimation," CVPR, 2019.
[19] S. Hinterstoisser et al., "Model based training, detection and pose estimation of texture-less 3D objects in heavily cluttered scenes," ACCV, 2012.
[20] B. Drost et al., "Model globally, match locally: Efficient and robust 3D object recognition," CVPR, 2010.
[21] Y. Xiang et al., "PoseCNN: A convolutional neural network for 6D object pose estimation in cluttered scenes," RSS, 2018.
[22] C. Wang et al., "DenseFusion: 6D object pose estimation by iterative dense fusion," CVPR, 2019.
[23] Y. He et al., "PVN3D: A deep point-wise 3D keypoints voting network for 6DoF pose estimation," CVPR, 2020.
[24] Y. He et al., "FFB6D: A full flow bidirectional fusion network for 6D pose estimation," CVPR, 2021.
[25] Y. Labbé et al., "CosyPose: Consistent multi-view multi-object 6D pose estimation," ECCV, 2020.
[26] Y. Labbé et al., "MegaPose: 6D pose estimation of novel objects via render & compare," CoRL, 2022.
[27] A. Kirillov et al., "Segment anything," ICCV, 2023.
[28] M. Oquab et al., "DINOv2: Learning robust visual features without supervision," TMLR, 2024.
[29] V. N. Nguyen et al., "CNOS: A strong baseline for CAD-based novel object segmentation," ICCV Workshops, 2023.
[30] J. Lin et al., "SAM-6D: Segment anything model meets zero-shot 6D object pose estimation," CVPR, 2024.
[31] B. Wen et al., "FoundationPose: Unified 6D pose estimation and tracking of novel objects," CVPR, 2024.
[32] V. N. Nguyen et al., "GigaPose: Fast and robust novel object pose estimation via one correspondence," CVPR, 2024.
[33] T. Hodaň et al., "BOP: Benchmark for 6D object pose estimation," ECCV, 2018.
[34] S. Liu et al., "Grounding DINO: Marrying DINO with grounded pre-training for open-set object detection," ECCV, 2024.
[35] T. Cheng et al., "YOLO-World: Real-time open-vocabulary object detection," CVPR, 2024.
[36] C. Zhang et al., "Faster segment anything: Towards lightweight SAM for mobile applications," arXiv:2306.14289, 2023.
[37] J. McCormac et al., "SemanticFusion: Dense 3D semantic mapping with convolutional neural networks," ICRA, 2017.
[38] R. F. Salas-Moreno et al., "SLAM++: Simultaneous localisation and mapping at the level of objects," CVPR, 2013.
[39] J. McCormac et al., "Fusion++: Volumetric object-level SLAM," 3DV, 2018.
[40] M. Rünz, M. Buffier, L. Agapito, "MaskFusion: Real-time recognition, tracking and reconstruction of multiple moving objects," ISMAR, 2018.
[41] B. Xu et al., "MID-Fusion: Octree-based object-level multi-instance dynamic SLAM," ICRA, 2019.
[42] S. Yang, S. Scherer, "CubeSLAM: Monocular 3-D object SLAM," IEEE T-RO, 2019.
[43] L. Nicholson, M. Milford, N. Sünderhauf, "QuadricSLAM: Dual quadrics from object detections as landmarks in object-oriented SLAM," IEEE RA-L, 2019.
[44] J. Wang, M. Rünz, L. Agapito, "DSP-SLAM: Object oriented SLAM with deep shape priors," 3DV, 2021.
[45] N. Merrill et al., "Symmetry and uncertainty-aware object SLAM for 6DoF object pose estimation," CVPR, 2022.
[46] N. Sünderhauf et al., "Meaningful maps with object-oriented semantic mapping," IROS, 2017.
[47] C. Liu et al., "FM-Fusion: Instance-aware semantic mapping boosted by vision-language foundation models," IEEE RA-L, 2024.
[48] Q. Gu et al., "ConceptGraphs: Open-vocabulary 3D scene graphs for perception and planning," ICRA, 2024.
[49] A. Werby et al., "Hierarchical open-vocabulary 3D scene graphs for language-grounded robot navigation," RSS, 2024.
[50] D. Maggio et al., "Clio: Real-time task-driven open-set 3D scene graphs," IEEE RA-L, 2024.
[51] "OpenVox: Real-time instance-level open-vocabulary probabilistic voxel representation," arXiv:2502.16528, 2025.
[52] "OpenGS-Fusion: Open-vocabulary dense mapping with hybrid 3D Gaussian splatting for refined object-level understanding," arXiv:2508.01150, 2025.
[53] N. Hughes, Y. Chang, L. Carlone, "Hydra: A real-time spatial perception system for 3D scene graph construction and optimization," RSS, 2022.
