# sam6d 모델 weight 이전용 분할 아카이브 (2026-07-06)

다른 PC에서 `git pull` 후 이 아카이브만 풀면 이식된 pipeline을 바로 실행할 수 있다.
(.gitignore로 repo에서 제외된 대용량 weight들)

## 포함 파일 (원본 경로 그대로 보존됨)

- `yolov8m-worldv2.pt`, `yolov8s-worldv2.pt` — YOLO-World
- `mobile_sam.pt` — MobileSAM
- `weights/clip/ViT-B-32.pt` — YOLO-World set_classes용 CLIP
- `sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/dinov2/dinov2_vits14_pretrain.pth`
- `sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth` (1.3GB)

## 파트 (각 ≤800MB)

`sam6d_weights.tar.gz.part-aa` (800MB) / `part-ab` (800MB) / `part-ac` (33MB) + `checksums.md5`

## 복원 방법 (다른 PC의 sam6d_ws 루트에서)

```bash
md5sum -c checksums.md5                       # 전송 무결성 확인
cat sam6d_weights.tar.gz.part-* | tar xzf -   # 원 경로에 그대로 풀림
```

template feature 캐시(outputs/yolo_ism_object_n/template_features/)는 미포함 —
첫 실행 시 자동 재생성됨 (template/<obj>/templates PNG는 별도 필요).
