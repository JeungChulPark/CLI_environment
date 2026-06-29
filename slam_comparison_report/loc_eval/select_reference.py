#!/usr/bin/env python3
"""Select the representative (median) reference run per dataset x system.

For each system's 5 stored runs, pairwise-align (Sim3) and compute ATE; the run
with the smallest mean ATE to the other four is the 'median' representative.
Writes reference_index.md and prints a JSON mapping.
"""
import json, sys
from pathlib import Path
import numpy as np
import traj_io, metrics

REP = Path(__file__).resolve().parents[1]            # slam_comparison_report/
EVAL = REP / "localization_eval"
DATASETS = ["SLAM_one_lap", "SLAM_one_lap_back_and_forth",
            "SLAM_forward_backward_repeat", "SLAM_three_laps"]
SYSTEMS = {"orb3": "orb3_easyLC", "rtab": "rtab"}


def pairwise_mean_ate(runs):
    n = len(runs)
    M = np.full((n, n), np.nan)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            a, b = runs[i], runs[j]
            m = min(len(a), len(b))
            if m < 3:
                continue
            # crude index pairing for ranking only (same sequence, same order)
            ai = np.linspace(0, len(a) - 1, m).astype(int)
            bi = np.linspace(0, len(b) - 1, m).astype(int)
            out, _, _ = metrics.ate_rpe(a[ai], b[bi], with_scale=True)
            M[i, j] = out["ate_m"]["rmse"] or np.nan
    return np.nanmean(M, axis=1)


def main():
    mapping = {}
    lines = ["# Reference Index (representative / median run)", "",
             "기준 trajectory = 각 dataset×system의 5개 저장 런 중 **상호 ATE 평균이 최소**인 대표(중앙값) 런.",
             "포맷: TUM (`t x y z qx qy qz qw`). GT 아님 — run-to-run 재현성 기준.", ""]
    for sys_key, folder in SYSTEMS.items():
        lines.append(f"## {sys_key} (`{folder}/`)")
        lines.append("")
        lines.append("| dataset | runs found | pose counts | chosen ref | mean-ATE(m) |")
        lines.append("|---|---|---|---|---|")
        for ds in DATASETS:
            base = REP / folder / ds
            runs_xyz, run_names, npose = [], [], []
            for r in range(1, 6):
                f = base / f"run{r}" / "trajectory.txt"
                t, xyz, q = traj_io.load_tum(f)
                if len(xyz) >= 3:
                    runs_xyz.append(xyz); run_names.append(f"run{r}"); npose.append(len(xyz))
            if not runs_xyz:
                lines.append(f"| {ds} | 0 | - | NONE | - |")
                mapping.setdefault(sys_key, {})[ds] = None
                continue
            if len(runs_xyz) == 1:
                best_i, score = 0, 0.0
            else:
                means = pairwise_mean_ate(runs_xyz)
                best_i = int(np.nanargmin(means)); score = float(means[best_i])
            chosen = run_names[best_i]
            mapping.setdefault(sys_key, {})[ds] = {
                "ref_run": chosen,
                "ref_path": f"{folder}/{ds}/{chosen}/trajectory.txt",
                "n_pose": npose[best_i]}
            lines.append(f"| {ds} | {len(run_names)} | {npose} | **{chosen}** | {score:.4f} |")
        lines.append("")
    EVAL.mkdir(parents=True, exist_ok=True)
    (EVAL / "reference_index.md").write_text("\n".join(lines))
    (EVAL / "reference_map.json").write_text(json.dumps(mapping, indent=2))
    print(json.dumps(mapping, indent=2))


if __name__ == "__main__":
    main()
