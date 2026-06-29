#!/usr/bin/env python3
"""Evaluate one dataset x system: align N new runs against the median reference.

Usage: eval_condition.py <orb3|rtab> <dataset> <new_run_dir1> [<dir2> ...]
  - reads reference_map.json for the chosen reference trajectory
  - each new_run_dir must contain trajectory.txt (TUM)
Outputs into localization_eval/{dataset}/{orb3_slam|rtabmap}/:
  reference_trajectory.csv, estimated_trajectory.csv (run1 primary),
  aligned_estimated_trajectory.csv, trajectory_metrics.json, framewise_error.csv,
  top_view_reference.png, top_view_estimated.png, top_view_overlay.png,
  tracking_status.png, trajectory_error_plot.png
"""
import sys, json, csv
from pathlib import Path
import numpy as np
import traj_io, metrics, viz

REP = Path(__file__).resolve().parents[1]
EVAL = REP / "localization_eval"
SUB = {"orb3": "orb3_slam", "rtab": "rtabmap"}


def main():
    sys_key, ds = sys.argv[1], sys.argv[2]
    new_dirs = [Path(p) for p in sys.argv[3:]]
    refmap = json.loads((EVAL / "reference_map.json").read_text())
    ref_info = refmap.get(sys_key, {}).get(ds)
    out = EVAL / ds / SUB[sys_key]; out.mkdir(parents=True, exist_ok=True)

    if not ref_info:
        (out / "trajectory_metrics.json").write_text(json.dumps(
            {"system": sys_key, "dataset": ds, "error": "no reference run found"}, indent=2))
        print(f"{sys_key} {ds}: NO REFERENCE"); return

    rt, rxyz, rq = traj_io.load_tum(REP / ref_info["ref_path"])
    traj_io.save_csv(out / "reference_trajectory.csv", rt, rxyz, rq)
    viz.top_view_single(out / "top_view_reference.png", rxyz,
                        f"{sys_key} | {ds} | reference ({ref_info['ref_run']})", "#222222", "reference")

    runs_metrics = []
    primary_saved = False
    for k, d in enumerate(new_dirs, 1):
        et, exyz, eq = traj_io.load_tum(d / "trajectory.txt")
        rec = {"new_run": d.name, "src": str(d), "n_pose_est": int(len(exyz))}
        if len(exyz) < 3:
            rec["status"] = "FAILED_EMPTY"; runs_metrics.append(rec); continue

        ref_p, ref_qp, est_p, est_qp, t_used, cov = traj_io.associate_interp(
            rt, rxyz, rq, et, exyz, eq)
        rec["coverage_frac"] = round(cov, 4)
        rec["success_rate_assoc"] = round(cov, 4)
        rec["n_matched"] = int(len(ref_p))
        if len(ref_p) < 3:
            rec["status"] = "TOO_FEW_MATCHES"; runs_metrics.append(rec); continue

        m_sim3, est_aln, T = metrics.ate_rpe(ref_p, est_p, ref_qp, est_qp, with_scale=True)
        m_se3, _, _ = metrics.ate_rpe(ref_p, est_p, ref_qp, est_qp, with_scale=False)
        issues = metrics.detect_tracking_issues(et, exyz)
        rec.update({"status": "OK", "sim3": m_sim3, "se3": {"ate_m": m_se3["ate_m"],
                    "rot_err_deg": m_se3.get("rot_err_deg")},
                    "tracking": {"n_gaps": issues["n_gaps"], "n_jumps": issues["n_jumps"],
                                 "median_dt": issues.get("median_dt"),
                                 "gaps": issues["gaps"][:20], "jumps": issues["jumps"][:20]}})
        runs_metrics.append(rec)

        if not primary_saved:  # first OK run = primary artifacts
            traj_io.save_csv(out / "estimated_trajectory.csv", et, exyz, eq)
            traj_io.save_csv(out / "aligned_estimated_trajectory.csv", t_used, est_aln, est_qp)
            ate_series = np.linalg.norm(est_aln - ref_p, axis=1)
            with open(out / "framewise_error.csv", "w", newline="") as fh:
                w = csv.writer(fh); w.writerow(["matched_idx", "timestamp_ref", "ate_m"])
                for i in range(len(ate_series)):
                    w.writerow([i, f"{t_used[i]:.6f}", f"{ate_series[i]:.6f}"])
            viz.top_view_single(out / "top_view_estimated.png", exyz,
                                f"{sys_key} | {ds} | estimated ({d.name})", "#ff7f0e", "estimated")
            viz.top_view_overlay(out / "top_view_overlay.png", rxyz, est_aln,
                                 f"{sys_key} | {ds} | overlay (ATE rmse={m_sim3['ate_m']['rmse']:.3f}m)")
            viz.tracking_status(out / "tracking_status.png", et, exyz, issues,
                                f"{sys_key} | {ds} | tracking status (gaps={issues['n_gaps']}, jumps={issues['n_jumps']})")
            viz.error_plot(out / "trajectory_error_plot.png", ate_series,
                           f"{sys_key} | {ds} | per-pose ATE ({d.name})")
            primary_saved = True

    # aggregate across OK runs
    ok = [r for r in runs_metrics if r.get("status") == "OK"]
    agg = None
    if ok:
        ate_rmse = [r["sim3"]["ate_m"]["rmse"] for r in ok]
        rot_med = [r["sim3"]["rot_err_deg"]["median"] for r in ok if r["sim3"].get("rot_err_deg")]
        agg = {"n_runs_ok": len(ok), "n_runs_total": len(runs_metrics),
               "ate_rmse_m": {"median": float(np.median(ate_rmse)),
                              "std": float(np.std(ate_rmse)), "values": ate_rmse},
               "assoc_success_rate": {"median": float(np.median([r["success_rate_assoc"] for r in ok]))}}
        if rot_med:
            agg["rot_err_deg"] = {"median": float(np.median(rot_med)), "values": rot_med}
    result = {"system": sys_key, "dataset": ds,
              "reference": ref_info, "n_new_runs": len(new_dirs),
              "alignment_note": "Sim3 (scale-incl) primary; SE3 also reported. est associated by relative-time nearest (|dt|<0.05s).",
              "sparse_warning": (len(rxyz) < 20),
              "runs": runs_metrics, "aggregate": agg}
    (out / "trajectory_metrics.json").write_text(json.dumps(result, indent=2))
    print(f"{sys_key} {ds}: ref={ref_info['ref_run']}({len(rxyz)}p) ok={len(ok)}/{len(new_dirs)} "
          f"ate_rmse_median={agg['ate_rmse_m']['median']:.3f}m" if agg else
          f"{sys_key} {ds}: ref={ref_info['ref_run']} NO OK RUNS")


if __name__ == "__main__":
    main()
