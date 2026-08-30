# 실행에 필요한 추가 자산 다운로드 가이드 (Required Assets)

이 저장소(`ldh8628/sam6d-object-memory`)는 **소스 코드 · 설정 · milk 템플릿 · 카메라 config만** 포함합니다.
대용량 바이너리(모델 가중치, CAD `.ply`, 렌더 템플릿, ROS2 bag)는 `.gitignore`로 제외되어 있으므로,
다른 PC에서 `git clone` 후 아래 자산을 **추가로 받아/복사해야** 파이프라인이 동작합니다.

> 요약: **① 모델 가중치(다운로드 가능)** + **② CAD `.ply`(원본 PC에서 복사 — 비공개)** + **③ ROS2 bag(원본 PC/스토리지에서 복사)** 가 필수.
> **④ 렌더 템플릿**과 **⑤ ISM 특징 캐시**는 ①②가 있으면 **재생성 가능**(다운로드 불필요).

---

## 0. 디렉터리 배치 한눈에 보기

```
sam6d_ws/
├─ sam_b.pt                         ← ① ultralytics SAM-b (자동 다운로드)
├─ mobile_sam.pt                    ← ① MobileSAM
├─ yolov8m-worldv2.pt               ← ① YOLO-World (자동 다운로드)
├─ yolov8s-worldv2.pt               ← ① YOLO-World fallback
├─ weights/clip/ViT-B-32.pt         ← ① CLIP (YOLO-World 텍스트 인코더)
├─ data/
│  ├─ cad/<object>/*.ply            ← ② CAD (비공개 · 원본 PC 복사)
│  └─ ros2_bag/<bag>/bag_0.db3      ← ③ 입력 bag (원본 PC 복사)
├─ template/<object>/templates/     ← ④ 렌더 템플릿 (재생성 가능, 2.2G)
├─ outputs/yolo_ism_object_n/template_features/*.pt  ← ⑤ ISM 캐시 (첫 실행 시 자동 생성)
└─ sam6d_master/SAM-6D/
   ├─ checkpoints/mae_pretrain_vit_base.pth
   ├─ Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth
   ├─ Pose_Estimation_Model/checkpoints/mae_pretrain_vit_base.pth
   └─ Instance_Segmentation_Model/checkpoints/
      ├─ dinov2/dinov2_vits14_pretrain.pth
      └─ FastSAM/FastSAM-x.pt , FastSAM-s.pt
```

---

## 1. 모델 가중치 / 체크포인트 (다운로드 가능) — 약 3.0 GB

| # | 파일 | 배치 경로 | 크기 | 취득 방법 |
|---|------|-----------|------|-----------|
| 1 | `sam-6d-pem-base.pth` | `sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/` | 1.3G | `python sam6d_master/SAM-6D/Pose_Estimation_Model/download_sam6d-pem.py` (gdown `1joW9IvwsaRJYxoUmGo68dBVg-HcFNyI7`) |
| 2 | `mae_pretrain_vit_base.pth` | `.../Pose_Estimation_Model/checkpoints/` **및** `sam6d_master/SAM-6D/checkpoints/` (양쪽 동일 파일) | 328M | `wget https://dl.fbaipublicfiles.com/mae/pretrain/mae_pretrain_vit_base.pth` |
| 3 | `dinov2_vits14_pretrain.pth` | `.../Instance_Segmentation_Model/checkpoints/dinov2/` | 85M | `wget https://dl.fbaipublicfiles.com/dinov2/dinov2_vits14/dinov2_vits14_pretrain.pth` (※ 이 저장소 기본값은 **vits14**) |
| 4 | `FastSAM-x.pt` | `.../Instance_Segmentation_Model/checkpoints/FastSAM/` | 139M | `python sam6d_master/SAM-6D/Instance_Segmentation_Model/download_fastsam.py` (gdown `1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv`) |
| 5 | `FastSAM-s.pt` | `.../Instance_Segmentation_Model/checkpoints/FastSAM/` | 23M | `wget https://github.com/ultralytics/assets/releases/download/v8.3.0/FastSAM-s.pt` |
| 6 | `mobile_sam.pt` | 저장소 루트 `sam6d_ws/` | 39M | `wget https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt` |
| 7 | `yolov8m-worldv2.pt` | 저장소 루트 | 55M | ultralytics 첫 실행 시 자동, 또는 `wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8m-worldv2.pt` |
| 8 | `yolov8s-worldv2.pt` | 저장소 루트 | 25M | ultralytics 자동, 또는 `.../yolov8s-worldv2.pt` |
| 9 | `weights/clip/ViT-B-32.pt` | `weights/clip/` | 338M | `wget https://openaipublic.azureedge.net/clip/models/40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/ViT-B-32.pt` |
| 10 | `sam_b.pt` | 저장소 루트 | 358M | ultralytics SAM 자동, 또는 `wget https://github.com/ultralytics/assets/releases/download/v8.3.0/sam_b.pt` |

> **DINOv2 주의**: 공식 `download_dinov2.py`는 다른 크기(vitl 등)를 받을 수 있습니다.
> 이 저장소는 `yolo_ism.py`의 `DEFAULT_DINOV2_CKPT = .../dinov2/dinov2_vits14_pretrain.pth`(ViT-S/14)를 사용하므로 위 **vits14** URL을 받으세요.
> **gdown 미설치 시**: `pip install gdown`.
> **SAM 원본(`sam_vit_b_01ec64.pth` 등)**이 필요하면 `download_sam.py` 참고 (현재 파이프라인은 MobileSAM/FastSAM 사용).

---

## 2. CAD 모델 `.ply` — **비공개 · 원본 PC에서 복사 필수** (약 2.4 GB)

자체 스캔한 객체 CAD이므로 **공개 다운로드 불가**. 원본 PC의 `data/cad/`를 통째로 복사하세요(rsync/scp/USB).

```
data/cad/
├─ Bear/bear_color_with_normal_vertexcolor.ply           (곰인형)
├─ Rabbit/Rabbit_color_with_normal_vertexcolor.ply       (토끼인형)
├─ choco_hazelnut_color_high/...vertexcolor.ply          (초코하임)
├─ Mugcup_color_high/...vertexcolor.ply                  (머그컵)
├─ saffron/saffron_color_...vertexcolor.ply              (샤프란)
├─ Sauce_high/Sauce_color_...vertexcolor.ply             (저당소스)
├─ Febreze_high/Febreze_color_...vertexcolor.ply         (페브리즈)
├─ Sikhye_high/Sikhye_color_...vertexcolor.ply           (식혜)
├─ Dinosaur/ , Rack/ , fail/milk/Milk.ply 등
```

> **단위 주의(중요)**: SAM-6D/BOP는 CAD를 **mm 단위**로 가정합니다(내부에서 /1000).
> `*.ply.orig_meter` 백업이 함께 있으면 현재 `.ply`는 이미 ×1000(mm) 변환된 것입니다.
> meter 단위 CAD를 새로 넣을 경우 `tools/e2e_pipeline/rescale_cad_to_mm.py`로 변환 필요(미변환 시 검출 0).

---

## 3. ROS2 입력 bag — **원본 PC/스토리지에서 복사** (약 44 GB)

파이프라인 입력 데이터. 자체 녹화본이므로 복사해야 합니다.

```
data/ros2_bag/<bag_name>/bag_0.db3 (+ metadata.yaml)
```

예) `rgbd_imu_sdk_bag`의 `SAM_circle / SAM_loop1 / SAM_loop2 / SAM_occlusion`.
필수 토픽: `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/color/camera_info`.

> 다른 위치(예: `data_slam/...`)에 bag이 있으면 심링크로 연결:
> `ln -sfn /abs/path/to/<bag> data/ros2_bag/<bag>` (각 bag은 내부에 `bag/bag_0.db3` 또는 `bag_0.db3` 포함).

---

## 4. 렌더 템플릿 `template/` — **재생성 가능** (다운로드 불필요, 2.2G)

CAD(`.ply`)만 있으면 렌더로 재생성합니다(객체당 42뷰: rgb/mask/xyz).

```bash
# 전체 객체 일괄 렌더
bash tools/render_all_templates.sh
# 또는 단일 객체 (CPU 렌더 예시)
python sam6d_master/SAM-6D/Render/render_custom_templates_cpu.py --help
```

> milk 템플릿(`sam6d_master/SAM-6D/Data/custom/Milk_scaled_195mm/templates`)은 저장소에 포함되어 있습니다.

---

## 5. ISM 템플릿 특징 캐시 — **첫 실행 시 자동 생성** (조치 불필요)

`outputs/yolo_ism_object_n/template_features/<obj>_{cls,appe}.pt` 는 ISM 첫 실행 시 템플릿에서 자동 계산·캐시됩니다.
강제 재생성: `--rebuild-features`.

---

## 6. 실행 환경 (conda)

ISM(YOLO-World 8.4)과 PEM(gorilla+pointnet2)은 ultralytics 버전 충돌로 **별도 env**가 필요합니다.

| 단계 | conda env | 비고 |
|------|-----------|------|
| ISM (segmentation) | `sam_yolo` | YOLO-World + DINOv2 + MobileSAM |
| PEM (6D pose) | `sam6d_ros_humble` | gorilla-core + pointnet2 빌드 필요 |

---

## 7. 설치 검증 체크리스트

```bash
# 가중치 존재 확인
ls -lh sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth \
       sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/dinov2/dinov2_vits14_pretrain.pth \
       mobile_sam.pt weights/clip/ViT-B-32.pt
# CAD 확인
ls data/cad/*/*.ply | head
# bag 확인
ls data/ros2_bag/*/ 2>/dev/null
# 스모크 (ISM 3프레임)  — sam_yolo env
python tools/build_pem_inputs.py --bag <bag> --max-frames 3
```

위가 모두 통과하면 `tools/run_imu_e2e.sh <bag> <stride>` 등으로 전체 파이프라인 실행이 가능합니다.
