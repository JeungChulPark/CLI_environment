# Sourced by run_obchange*.sh. On a machine without the verification machine's default paths (the laptop layout:
# ~/DeepLearning/Datasets + ~/DeepLearning/third_party), point every script at the laptop copies.
# Variables already set in the environment are kept.
if [ ! -d "$HOME/DeepLearning/Dataset/bop/ycbv" ] && [ -d "$HOME/DeepLearning/Datasets/paper_benchmarks/YCB_Video" ]; then
  : "${YCBV_DIR:=$HOME/DeepLearning/Datasets/paper_benchmarks/YCB_Video}"
  : "${YCBV_ISM:=$HOME/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper/ycbv_objects.yaml}"
  : "${FASTSAM_X:=$HOME/DeepLearning/CLI_environment/sam6d_realtime/sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt}"
  W=$HOME/DeepLearning/Datasets/objmap_obchange
  : "${SEQ_BAGS:=$W/bags}" "${FMFUSION_DATA:=$W/fmfusion}" "${DUALMAP_DATA:=$W/dualmap}"
  : "${OBJMAP_OBCHANGE_FRAMES:=$W/frames}"
  : "${FMFUSION_ROOT:=$HOME/DeepLearning/third_party/FM-Fusion}"
  : "${DUALMAP_REPO:=$HOME/DeepLearning/third_party/DualMap}"
  : "${GSAM_FMFUSION:=$HOME/DeepLearning/third_party/Grounded-Segment-Anything}"
  : "${BASELINE_WEIGHTS:=$GSAM_FMFUSION/checkpoints}"
  : "${FMF:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fmf_python.sh}"
  export YCBV_DIR YCBV_ISM FASTSAM_X SEQ_BAGS FMFUSION_DATA DUALMAP_DATA OBJMAP_OBCHANGE_FRAMES FMFUSION_ROOT \
         DUALMAP_REPO GSAM_FMFUSION BASELINE_WEIGHTS FMF
  # IntegrateInstanceMap links Open3D / glog from here
  case ":${LD_LIBRARY_PATH:-}:" in *":$HOME/DeepLearning/third_party/install/lib:"*) ;;
    *) export LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH};; esac
fi
# room name, its lower-case dataset name, and the log folder (Kitchen keeps out/obchange_logs)
ROOM=${ROOM:-Kitchen}; LAST=${LAST:-6}
R=$(echo "$ROOM" | tr '[:upper:]' '[:lower:]')
if [ "$R" = kitchen ]; then L=out/obchange_logs; else L=out/obchange_logs_$R; fi
