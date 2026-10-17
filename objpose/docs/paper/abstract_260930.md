# 요약 / Abstract (초안 260930)

## 요약

본 논문에서는 SLAM이 추정한 카메라 자세와 학습 없이 동작하는 6자유도 객체 자세 추정 기법을 결합하여, 이동 중인 로봇이 주변 물체의 위치와 방향을 지도로 유지하는 객체 자세 지도(Object Pose Map) 구축 방법을 제안한다. 기존 객체 수준 지도 작성 연구는 물체를 점군이나 볼륨으로 표현하여 방향 정보를 남기지 않으며, SAM-6D와 같은 자세 추정기는 프레임마다 독립적으로 동작하고 처리 시간이 길어 이동 로봇에 그대로 적용하기 어렵다. 이를 해결하기 위해 문구 기반 검출과 경량 분할로 객체 후보 생성 시간을 줄이고, 후보 검증과 자세 검증을 추가하여 비슷한 물체의 오인식이 지도에 들어가지 않도록 하였다. 또한 각 관측을 SLAM 키프레임 기준으로 저장하여 루프 폐쇄에 따른 자세 수정이 지도에 반영되도록 하였다. 공개 데이터셋 실험에서 제안 방법은 원본 SAM-6D보다 처리 시간이 짧았고, 같은 물체가 지도에 중복으로 등록되지 않았으며, 물체의 위치와 방향을 함께 담은 객체 자세 지도를 구축할 수 있음을 확인하였다.

**키워드:** SLAM, 6자유도 객체 자세 추정, 객체 수준 지도 작성, SAM-6D

## Abstract

This paper proposes a method for building an Object Pose Map, in which a mobile robot keeps the position and orientation of surrounding objects by combining camera poses from SLAM with a training-free 6D object pose estimator. Existing object-level mapping methods represent objects as point clouds or volumes and do not store their orientation, while pose estimators such as SAM-6D run independently on each frame and are too slow for a moving robot. To address these problems, we reduce the time for object candidate generation using text-prompted detection and lightweight segmentation, and add candidate and pose verification steps so that misrecognized similar objects are not registered in the map. Each observation is stored relative to a SLAM keyframe, so pose corrections from loop closure are reflected in the map. Experiments on a public dataset show that the proposed method runs faster than the original SAM-6D, registers each object in the map without duplication, and builds an object map that contains both the position and orientation of objects.

**Keywords:** SLAM, 6D object pose estimation, object-level mapping, SAM-6D
