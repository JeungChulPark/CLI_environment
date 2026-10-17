#!/usr/bin/env python3
"""val_score.py — score_estcmp metrics for arbitrary run folders (validation frames).
    python val_score.py ycbv <run dir> [<run dir> ...]"""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import score_estcmp as S
ds = sys.argv[1]
cfg = dict(S.RUNS[ds]); cfg["runs"] = {Path(p).name: Path(p) for p in sys.argv[2:]}
S.score(ds, cfg)
