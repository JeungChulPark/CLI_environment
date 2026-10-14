#!/bin/bash
# .3 에서 실행: YCB-V 900장 GigaPose 거친 단계 (CNOS-FastSAM 검출, 정제 없음)
set -ex
source ~/anaconda3/etc/profile.d/conda.sh
conda activate gigapose
ROOT=$HOME/gigapose_root
cd ~/gigapose
# 1) 템플릿
if [ ! -d $ROOT/datasets/templates/ycbv ]; then
  mkdir -p $ROOT/datasets/tmp/tmpl && unzip -q -o $ROOT/datasets/tmp/templates.zip -d $ROOT/datasets/tmp/tmpl
  ls $ROOT/datasets/tmp/tmpl; mkdir -p $ROOT/datasets/templates
  mv $ROOT/datasets/tmp/tmpl/templates/ycbv $ROOT/datasets/templates/ycbv
fi
ls $ROOT/datasets/templates/ycbv | head -3; ls $ROOT/datasets/templates/ycbv/object_poses | head -3
# 2) 테스트 이미지 -> imagewise -> webdataset (datasets/ycbv/test 에 shard 생성)
if [ ! -f $ROOT/datasets/ycbv/test/key_to_shard.json ]; then
  mkdir -p $ROOT/datasets/tmp/ycbv_image_wise/test
  python -m src.scripts.convert_scenewise_to_imagewise --input $ROOT/datasets/ycbv/test --output $ROOT/datasets/tmp/ycbv_image_wise/test --nprocs 8
  python -m src.scripts.convert_imagewise_to_webdataset --input $ROOT/datasets/tmp/ycbv_image_wise/test --output $ROOT/datasets/ycbv/test --nprocs 4
fi
ls $ROOT/datasets/ycbv/test | head; 
# 3) 거친 단계 실행 (localization: test_targets_bop19.json 의 900장)
python test.py user.local_root_dir=$ROOT test_dataset_name=ycbv run_id=ycbv4090 test_setting=localization machine.num_workers=8 2>&1 | tee $ROOT/test_ycbv4090.log
ls -la $ROOT/results/large_ycbv4090/predictions/
echo RUN_DONE
