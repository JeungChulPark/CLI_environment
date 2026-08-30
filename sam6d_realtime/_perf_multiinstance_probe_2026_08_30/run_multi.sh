#!/usr/bin/env bash
# run_multi.sh N [tag] — bench_continuous.py 를 N개 동시 실행 (barrier 정렬)
N="${1:?instance count}"
TAG="${2:-multi$N}"
DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$DIR")"
BARRIER="$DIR/outputs/${TAG}_barrier"
rm -rf "$BARRIER"

source ~/anaconda3/etc/profile.d/conda.sh
conda activate sam6d
source ~/anaconda3/envs/sam6d/setup.bash
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
# MPS 데몬이 떠 있으면 클라이언트가 붙도록 pipe 경로를 알려 준다 (없으면 무해)
export CUDA_MPS_PIPE_DIRECTORY=/tmp/nvidia-mps CUDA_MPS_LOG_DIRECTORY=/tmp/nvidia-mps-log
cd "$REPO"

pids=()
for i in $(seq 0 $((N - 1))); do
  python -u "$DIR/bench_continuous.py" --tag "$TAG" --frames 60 --stride 15 \
    --barrier-dir "$BARRIER" --barrier-count "$N" --instance-id "$i" \
    > "$DIR/outputs/${TAG}_i${i}.log" 2>&1 &
  pids+=($!)
  sleep 3   # 모델 로드 디스크 경합 완화
done
rc=0
for p in "${pids[@]}"; do wait "$p" || rc=1; done
echo "run_multi done rc=$rc"
exit $rc
