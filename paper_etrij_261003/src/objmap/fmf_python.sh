#!/bin/sh
# FM-Fusion detector python on the laptop: gsam_venv does not have GroundingDINO / segment_anything installed as
# packages (fmfusion_detect.py imports `groundingdino`), so put their source folders on PYTHONPATH for this process only.
G=${GSAM_FMFUSION:-$HOME/DeepLearning/third_party/Grounded-Segment-Anything}
PYTHONPATH=$G/GroundingDINO:$G/segment_anything${PYTHONPATH:+:$PYTHONPATH} exec $HOME/DeepLearning/third_party/gsam_venv/bin/python "$@"
