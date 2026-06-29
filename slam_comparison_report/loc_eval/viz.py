#!/usr/bin/env python3
"""Visualization helpers for localization eval (X-Z top view)."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _setup(ax, title):
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Z [m]")
    ax.grid(True, lw=0.3, color="#e8e8e8")
    ax.set_title(title, fontsize=11)


def top_view_single(path, xyz, title, color, label):
    fig, ax = plt.subplots(figsize=(8, 8), dpi=140)
    if len(xyz):
        ax.plot(xyz[:, 0], xyz[:, 2], "-", c=color, lw=1.6, label=label)
        ax.scatter([xyz[0, 0]], [xyz[0, 2]], c="#e8000b", s=110, edgecolors="white", zorder=5, label="Start")
        ax.scatter([xyz[-1, 0]], [xyz[-1, 2]], c="#1f77ff", s=110, edgecolors="white", zorder=5, label="End")
    _setup(ax, title); ax.legend(loc="best", fontsize=9)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def top_view_overlay(path, ref_xyz, est_aligned, title):
    fig, ax = plt.subplots(figsize=(8, 8), dpi=140)
    if len(ref_xyz):
        ax.plot(ref_xyz[:, 0], ref_xyz[:, 2], "-", c="#222222", lw=2.0, label="reference (median run)", zorder=2)
    if len(est_aligned):
        ax.plot(est_aligned[:, 0], est_aligned[:, 2], "-", c="#ff7f0e", lw=1.4, alpha=0.9,
                label="estimated (aligned)", zorder=3)
    if len(ref_xyz):
        ax.scatter([ref_xyz[0, 0]], [ref_xyz[0, 2]], c="#e8000b", s=110, edgecolors="white", zorder=5, label="Start")
    _setup(ax, title); ax.legend(loc="best", fontsize=9)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def tracking_status(path, t, xyz, issues, title):
    """Path colored normal; gaps (red segments) and jumps (magenta x) marked."""
    fig, ax = plt.subplots(figsize=(8, 8), dpi=140)
    if len(xyz):
        ax.plot(xyz[:, 0], xyz[:, 2], "-", c="#3a7", lw=1.3, label="tracked", zorder=2)
        for g in issues.get("gaps", []):
            i = g["i"]
            ax.plot(xyz[i:i + 2, 0], xyz[i:i + 2, 2], "-", c="#e8000b", lw=3.0,
                    zorder=4, label="lost/gap" if g is issues["gaps"][0] else None)
        jx = [xyz[j["i"], 0] for j in issues.get("jumps", [])]
        jz = [xyz[j["i"], 2] for j in issues.get("jumps", [])]
        if jx:
            ax.scatter(jx, jz, c="#d000d0", marker="x", s=60, zorder=5, label="pose jump")
    _setup(ax, title); ax.legend(loc="best", fontsize=9)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def error_plot(path, ate_series, title):
    fig, ax = plt.subplots(figsize=(9, 4), dpi=140)
    if len(ate_series):
        ax.plot(np.arange(len(ate_series)), ate_series, "-", c="#c0392b", lw=1.2)
        ax.axhline(np.median(ate_series), color="#2980b9", ls="--", lw=1,
                   label=f"median={np.median(ate_series):.3f} m")
    ax.set_xlabel("matched pose index"); ax.set_ylabel("ATE [m]")
    ax.grid(True, lw=0.3, color="#e8e8e8"); ax.set_title(title, fontsize=11)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)
