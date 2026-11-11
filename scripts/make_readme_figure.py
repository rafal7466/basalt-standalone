#!/usr/bin/env python3
"""Generate the README figures from the stored EuRoC MH_01 results.

    python scripts/make_readme_figure.py

Reads results/MH_01_{stereo,mono}/ (trajectory_tum.txt, metrics.csv) and
results/MH_01_groundtruth_tum.txt, aligns each trajectory to the groundtruth
with the same SE(3) Umeyama alignment as bw_vio, and writes light and dark
variants of two figures to docs/images/:
  * mh01_trajectory_{light,dark}.png - top view + position error over time
  * mh01_metrics_{light,dark}.png    - ATE / RPE and runtime per frame
Requires numpy and matplotlib.
"""
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
OUT = ROOT / "docs" / "images"
GT = RES / "MH_01_groundtruth_tum.txt"

RUNS = [("MH_01_stereo", "Stereo VIO", "#1f77b4"),
        ("MH_01_mono", "Mono VIO", "#d62728")]

THEMES = {
    "light": dict(bg="white", fg="#24292f", grid="#d0d7de", gt="#24292f"),
    "dark": dict(bg="#0d1117", fg="#e6edf3", grid="#30363d", gt="#e6edf3"),
}


def load_tum(path):
    return np.loadtxt(path, comments="#")[:, :4]  # t x y z


def align_se3(est, gt):
    """Interpolate the groundtruth at the estimate times, then SE(3) Umeyama (as src/eval.cpp)."""
    t = est[:, 0]
    ok = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    t, pe = t[ok], est[ok, 1:4]
    pg = np.column_stack([np.interp(t, gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
    me, mg = pe.mean(0), pg.mean(0)
    C = (pg - mg).T @ (pe - me) / len(pe)
    U, _, Vt = np.linalg.svd(C)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    pa = (R @ pe.T).T + (mg - R @ me)
    return t, pa, np.linalg.norm(pa - pg, axis=1)


def metrics(folder):
    with open(RES / folder / "metrics.csv", newline="") as f:
        return {r["metric"]: r for r in csv.DictReader(f)}


def style(ax, th):
    ax.set_facecolor(th["bg"])
    for s in ax.spines.values():
        s.set_color(th["grid"])
    ax.tick_params(colors=th["fg"])
    for lab in (ax.xaxis.label, ax.yaxis.label, ax.title):
        lab.set_color(th["fg"])
    ax.grid(True, color=th["grid"], lw=0.6, alpha=0.8)
    ax.set_axisbelow(True)


def legend(ax, th, **kw):
    leg = ax.legend(fontsize=8.5, facecolor=th["bg"], edgecolor=th["grid"], **kw)
    for t in leg.get_texts():
        t.set_color(th["fg"])


def trajectory_figure(theme):
    th = THEMES[theme]
    gt = load_tum(GT)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw=dict(width_ratios=[1, 1.35]))
    fig.patch.set_facecolor(th["bg"])
    a1.plot(gt[:, 1], gt[:, 2], color=th["gt"], lw=2.2, ls="--", label="Groundtruth (Leica + Vicon)")
    for folder, label, col in RUNS:
        t, pa, err = align_se3(load_tum(RES / folder / "trajectory_tum.txt"), gt)
        a1.plot(pa[:, 0], pa[:, 1], color=col, lw=1.1, label=label)
        a2.plot(t - gt[0, 0], err, color=col, lw=1.0,
                label=f"{label}  (ATE RMSE {np.sqrt(np.mean(err**2)):.3f} m)")
    a1.set_aspect("equal", adjustable="datalim")
    a1.set_xlabel("x [m]"); a1.set_ylabel("y [m]")
    a1.set_title("EuRoC MH_01_easy - top view (SE(3) aligned)")
    a2.set_xlabel("time [s]"); a2.set_ylabel("position error [m]")
    a2.set_title("Absolute position error over time")
    a2.set_ylim(0, None)
    for ax in (a1, a2):
        style(ax, th)
        legend(ax, th, loc="best")
    fig.tight_layout()
    fig.savefig(OUT / f"mh01_trajectory_{theme}.png", dpi=150, facecolor=th["bg"])
    plt.close(fig)


def metrics_figure(theme):
    th = THEMES[theme]
    M = [metrics(f) for f, _, _ in RUNS]
    labels = [l for _, l, _ in RUNS]
    cols = [c for _, _, c in RUNS]
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.2), gridspec_kw=dict(width_ratios=[1.3, 1, 1.1]))
    fig.patch.set_facecolor(th["bg"])

    a = axs[0]
    x = np.arange(2); w = 0.36
    for k, (key, name, c) in enumerate((("ate_se3_m", "ATE SE(3)", "#1f77b4"), ("ate_sim3_m", "ATE Sim(3)", "#2ca02c"))):
        v = [float(m[key]["rmse"]) for m in M]
        bars = a.bar(x + (k - 0.5) * w, v, w, color=c, label=name)
        for b, val in zip(bars, v):
            a.text(b.get_x() + b.get_width() / 2, val, f"{val:.3f}", ha="center", va="bottom", fontsize=8, color=th["fg"])
    a.set_xticks(x, labels); a.set_ylabel("RMSE [m]"); a.set_title("Absolute trajectory error")
    legend(a, th)

    a = axs[1]
    vt = [100 * float(m["rpe_trans_1s_m"]["rmse"]) for m in M]
    vr = [float(m["rpe_rot_1s_deg"]["rmse"]) for m in M]
    bars = a.bar(x - w / 2, vt, w, color="#8250df", label="translation [cm]")
    bars2 = a.bar(x + w / 2, vr, w, color="#bf8700", label="rotation [deg]")
    for b, val in list(zip(bars, vt)) + list(zip(bars2, vr)):
        a.text(b.get_x() + b.get_width() / 2, val, f"{val:.2f}", ha="center", va="bottom", fontsize=8, color=th["fg"])
    a.set_xticks(x, labels); a.set_title("Relative pose error over 1 s (RMSE)")
    legend(a, th)

    a = axs[2]
    of = [float(m["time_optical_flow_ms"]["mean"]) for m in M]
    vio = [float(m["time_vio_ms"]["mean"]) for m in M]
    a.bar(x, of, 0.55, color="#57606a", label="optical flow")
    a.bar(x, vio, 0.55, bottom=of, color=cols, label="VIO estimator")
    for i in range(2):
        a.text(x[i], of[i] + vio[i], f"{of[i] + vio[i]:.1f} ms", ha="center", va="bottom", fontsize=9, color=th["fg"])
    a.set_xticks(x, labels); a.set_ylabel("mean time per frame [ms]")
    a.set_title("Runtime (i9-13950HX, Release)")
    legend(a, th)
    for ax in axs:
        style(ax, th)
        ax.set_ylim(0, None)
        ax.margins(y=0.15)
    fig.tight_layout()
    fig.savefig(OUT / f"mh01_metrics_{theme}.png", dpi=150, facecolor=th["bg"])
    plt.close(fig)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for theme in THEMES:
        trajectory_figure(theme)
        metrics_figure(theme)
    print("figures written to", OUT)
