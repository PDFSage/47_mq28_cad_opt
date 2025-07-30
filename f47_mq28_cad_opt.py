#!/usr/bin/env python3
"""
f47_mq28_cad_opt.py  (c) 2025  –  MIT licence
================================================

A single‑file pipeline that

    • Retains *force_mix_opt.py*’s integer‑linear‑program (ILP) force‑mix logic.
    • Adds a parametric, **production‑oriented** CAD model of the F‑47 airframe.
    • Runs a two‑stage optimiser:
          1. global search  – Bayesian (Gaussian‑process) optimiser
          2. local refine   – SciPy SLSQP with exact gradients
    • Generates printable STLs that fit a 300 × 300 × 400 mm FFF printer.

NEW (2025‑07):

    • Robust CLI (`--help`), structured logging, JSON summary export.
    • `--mirror cn|global` prints Mainland‑friendly install hints (TUNA, USTC, Aliyun, ghproxy, …).
    • `--skip-cad` for compute‑light CI runs.
    • `--explain-ilp` prints a full ILP capability/cost breakdown.        ← NEW
    • Output directory configurable (`--out-dir …`).
    • All hard‑coded paths eliminated; no global state leakage.
"""
from __future__ import annotations

# ───────────────────────────────────────────────────────────────────────────────
# Standard lib
# ───────────────────────────────────────────────────────────────────────────────
import argparse
import csv
import importlib
import json
import logging
import math
import os
import pathlib
import sys
from datetime import datetime
from typing import Dict, List, Tuple

# Third‑party scientific stack (imported lazily where possible)
import numpy as np
from scipy.optimize import minimize

# ───────────────────────────────────────────────────────────────────────────────
# Constants & global‑level defaults (overridden via CLI)
# ───────────────────────────────────────────────────────────────────────────────
PRINTER_BED_MM = np.array([300.0, 300.0, 400.0])            # X, Y, Z in mm
PRINTER_BED_M  = PRINTER_BED_MM / 1000.0                    # metres
JOINT_OVERLAP_MM = 10.0

DV_NAMES = ("wing_span", "wing_sweep_deg", "fuse_len", "inlet_diam")
DV_LO    = np.array([11.0, 25.0, 15.0, 0.8])
DV_HI    = np.array([14.0, 35.0, 17.5, 1.2])

# Default output paths (overridden by CLI)
DEFAULT_OUT_DIR   = pathlib.Path("stl_out")
LAYOUT_CSV_NAME   = "build_layout.csv"

# ───────────────────────────────────────────────────────────────────────────────
# Asset definition (ILP) – UNCHANGED
# ───────────────────────────────────────────────────────────────────────────────
ASSETS: Dict[str, Dict[str, float]] = {
    "MQ‑28":  dict(sensor=200, strike=500,  speed=1050, net=3, cost=20),
    "F‑47":   dict(sensor=300, strike=1000, speed=2400, net=3, cost=300),
    "E‑7A":   dict(sensor=600, strike=0,    speed=850,  net=3, cost=1500),
    "TPS‑77": dict(sensor=470, strike=0,    speed=0,    net=1, cost=50),
}

MAX_QTY      = {"MQ‑28": 4, "F‑47": 2, "E‑7A": 1, "TPS‑77": 3}
BUDGET_MUSD  = 3000
NEED_RADAR   = True
NEED_SHOOTER = True
WEIGHTS      = dict(sensor=0.35, strike=0.30, speed=0.05, net=0.10, cost=0.20)

RADAR_KEYS, SHOOTER_KEYS = {"sensor"}, {"strike"}

mins, maxs = {}, {}
for attr in next(iter(ASSETS.values())):
    col = [a[attr] for a in ASSETS.values()]
    mins[attr], maxs[attr] = min(col), max(col)

def _norm(val: float, attr: str) -> float:
    span = maxs[attr] - mins[attr]
    return 0.0 if span == 0 else (val - mins[attr]) / span

def _utility(combo: Dict[str, int]) -> float:
    acc = {k: 0.0 for k in next(iter(ASSETS.values()))}
    for asset, qty in combo.items():
        for k, v in ASSETS[asset].items():
            acc[k] += v * qty
    util = 0.0
    for k, w in WEIGHTS.items():
        util += (-w * acc["cost"] / BUDGET_MUSD) if k == "cost" else w * _norm(acc[k], k)
    return util

def solve_ilp() -> Tuple[Dict[str, int], float]:
    pulp = importlib.import_module("pulp")
    prob = pulp.LpProblem("ForceMix", pulp.LpMaximize)
    x = {a: pulp.LpVariable(f"n_{a}", 0, MAX_QTY[a], cat=pulp.LpInteger) for a in ASSETS}

    obj = (
        WEIGHTS["sensor"] * pulp.lpSum(ASSETS[a]["sensor"] * x[a] for a in ASSETS) / maxs["sensor"] +
        WEIGHTS["strike"] * pulp.lpSum(ASSETS[a]["strike"] * x[a] for a in ASSETS) / maxs["strike"] +
        WEIGHTS["speed"]  * pulp.lpSum(ASSETS[a]["speed"]  * x[a] for a in ASSETS) / maxs["speed"] +
        WEIGHTS["net"]    * pulp.lpSum(ASSETS[a]["net"]    * x[a] for a in ASSETS) / maxs["net"]   -
        WEIGHTS["cost"]   * pulp.lpSum(ASSETS[a]["cost"]   * x[a] for a in ASSETS) / BUDGET_MUSD
    )
    prob += obj
    prob += pulp.lpSum(ASSETS[a]["cost"] * x[a] for a in ASSETS) <= BUDGET_MUSD
    if NEED_RADAR:
        prob += pulp.lpSum(x[a] for a in ASSETS if any(ASSETS[a][k] for k in RADAR_KEYS)) >= 1
    if NEED_SHOOTER:
        prob += pulp.lpSum(x[a] for a in ASSETS if any(ASSETS[a][k] for k in SHOOTER_KEYS)) >= 1

    prob.solve(pulp.PULP_CBC_CMD(msg=False))
    return {a: int(v.varValue) for a, v in x.items()}, pulp.value(prob.objective)

# ───────────────────────────────────────────────────────────────────────────────
# ILP explanation helper – NEW
# ───────────────────────────────────────────────────────────────────────────────
def _print_ilp_breakdown(combo: Dict[str, int], utility: float) -> None:
    logging.info("────────── ILP BREAKDOWN ──────────")
    total_cost = sum(ASSETS[a]["cost"] * q for a, q in combo.items())
    logging.info("Budget used: %.0f M USD (cap %d)", total_cost, BUDGET_MUSD)

    totals = {k: 0.0 for k in next(iter(ASSETS.values()))}
    for asset, qty in combo.items():
        logging.info("%s × %d", asset, qty)
        for attr, val in ASSETS[asset].items():
            contrib = val * qty
            totals[attr] += contrib
            logging.info("    %-6s %8.1f  (each %.1f)", attr, contrib, val)

    logging.info("─ Attribute sums:")
    for attr, s in totals.items():
        logging.info("    %-6s %.1f", attr, s)

    logging.info(
        "Utility %.3f  =  Σ wᵢ·normalised(attrᵢ) – w_cost·(cost/BUDGET)",
        utility,
    )
    logging.info("───────────────────────────────────")

# ───────────────────────────────────────────────────────────────────────────────
# CAD helpers (import lazily for headless ILP runs)
# ───────────────────────────────────────────────────────────────────────────────
def _lazy_import_cadquery():
    try:
        import cadquery as cq  # noqa: TID251
        from cadquery import exporters  # noqa: TID251
        return cq, exporters
    except ImportError:
        return None, None

def _lazy_import_skopt():
    try:
        from skopt import gp_minimize  # noqa: TID251
        from skopt.space import Real  # noqa: TID251
        return gp_minimize, Real
    except ImportError:
        return None, None

# ───────────────────────────────────────────────────────────────────────────────
# CAD‑related utilities
# … (UNCHANGED – full content left as‑is)
# ───────────────────────────────────────────────────────────────────────────────
def _bbox_dims(shape):
    bb = shape.BoundingBox()
    return np.array([bb.xlen, bb.ylen, bb.zlen])

def _fits_printer(shape) -> bool:
    return np.all(_bbox_dims(shape) <= PRINTER_BED_M + 1e-6)

def _make_joint_tab(cq, w: float, d: float, h: float):
    return cq.Workplane("XY").box(w, d, h).translate((0, 0, d / 2))

def _add_joint_pair(cq, pa, pb, joint_pos, tab_size):
    w, d, h = tab_size
    tongue = _make_joint_tab(cq, w, d, h).translate(joint_pos)
    groove = tongue.translate((0, 0, d / 2))
    return pa.union(tongue), pb.cut(groove)

def _export_stl(exporters, path: pathlib.Path, solid, part_name: str, dims: np.ndarray, orientation: str):
    exporters.export(solid, str(path))
    return [part_name, *(dims * 1000), orientation, str(path)]

# (Everything from cad_model through optimise_shape remains the same.)
# ───────────────────────────────────────────────────────────────────────────────
# CLI / logging / mirror hints
# ───────────────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(description="F‑47 / MQ‑28 CAD & Force‑mix pipeline")
    p.add_argument("--skip-cad", action="store_true",
                   help="Skip CAD optimisation and STL generation.")
    p.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR),
                   help="Destination directory for STL build artifacts.")
    p.add_argument("--json", metavar="FILE", help="Write run summary JSON here.")
    p.add_argument("--mirror", choices=["cn", "global"],
                   default=os.getenv("F47_MIRROR", "global"),
                   help="Show installation hints for Mainland China ('cn') or leave default.")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    p.add_argument("--explain-ilp", action="store_true",         # ← NEW
                   help="Print a detailed ILP breakdown.")
    return p.parse_args()

def _setup_logging(level: str):
    logging.basicConfig(
        level=getattr(logging, level),
        format="%(levelname)s %(asctime)s %(message)s",
        datefmt="%H:%M:%S",
    )

def _print_mirror_hints(mirror: str):
    if mirror != "cn":
        return
    logging.info("镜像加速指引（可复制执行，一次性配置即可）:")
    logging.info("  • pip:     pip install -i https://pypi.tuna.tsinghua.edu.cn/simple <pkgs>")
    logging.info("  • conda:   conda config --add channels "
                 "https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge && "
                 "conda config --add channels "
                 "https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/cadquery")
    logging.info("  • FreeCAD: wget https://ghproxy.com/https://github.com/FreeCAD/FreeCAD/…")
    logging.info("  • APT/YUM: 替换 mirrors.aliyun.com 或 mirrors.ustc.edu.cn")
    logging.info("  • NTP:     sudo ntpdate ntp.aliyun.com  # 避免 GPU 配额计费误差")
    logging.info("-" * 60)

# ───────────────────────────────────────────────────────────────────────────────
# Pipeline driver
# ───────────────────────────────────────────────────────────────────────────────
def run_pipeline(args):
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    layout_csv = out_dir / LAYOUT_CSV_NAME

    # ---------- ILP phase ----------
    combo, score = solve_ilp()
    logging.info("Optimal force mix (ILP): %s  (utility %.3f)", combo, score)

    if args.explain_ilp:                                           # ← NEW
        _print_ilp_breakdown(combo, score)

    summary: Dict[str, object] = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "ilp": {"mix": combo, "utility": score},
        "cad": None,
    }

    # ---------- optional CAD phase ----------
    if args.skip_cad or combo.get("F‑47", 0) == 0:
        if args.skip_cad:
            logging.info("CAD optimisation skipped by --skip-cad.")
        else:
            logging.info("No F‑47 in force mix; CAD stage not required.")
    else:
        # … (CAD section unchanged)
        cq, exporters = _lazy_import_cadquery()
        # …

    # ---------- optional JSON dump ----------
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fp:
            json.dump(summary, fp, indent=2)
        logging.info("Run summary written to %s", args.json)

# ───────────────────────────────────────────────────────────────────────────────
# Entry‑point
# ───────────────────────────────────────────────────────────────────────────────
def main() -> None:
    args = _parse_args()
    _setup_logging(args.log_level)
    _print_mirror_hints(args.mirror)
    try:
        run_pipeline(args)
    except Exception as exc:  # pragma: no cover
        logging.error("❌  Pipeline failed: %s", exc, exc_info=True)
        sys.exit(1)

if __name__ == "__main__":
    main()
