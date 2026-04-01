#!/usr/bin/env python3
"""
prep_action_csv.py  –  Prepare an action CSV from real-robot joint position
                       recordings, ready to feed into csv_play.py via --csv.

How actions are defined in the HANU_A4 training environment
────────────────────────────────────────────────────────────
The policy outputs a normalised action vector  a ∈ ℝ²³.
IsaacLab converts it to a joint-position target via:

    q_target[t] = q_default + a[t] × scale          (scale = 0.25 rad)

So to recover  a  from measured joint positions:

    a[t] = (q_measured[t] − q_initial) / scale
         = Δq_relative[t] / 0.25

where  q_initial = q_measured[0]  (first row of the recording) and
Δq_relative  is the deviation from the robot's starting pose.

Output
──────
A CSV with one row per simulation step and 23 columns (one per joint in
POLICY_JOINT_ORDER), saved to  <csv_sim_dir>/actions.csv.
Feed it into csv_play.py with  --csv <path>  (--action_scale 1.0 because
scaling is baked in here).

Usage
─────
# Auto mode – find csv_sim/ inside the run folder
python prep_action_csv.py --load_run logs/rsl_rl/hanu_a4_flat/2026-03-30_06-14-28

# Manual mode
python prep_action_csv.py --pos_csv joint_state_*_position.csv --out actions.csv

# Then play
python csv_play.py --task ... --checkpoint ... \\
    --obs_csv logs/.../csv_sim/observations.csv --obs_raw \\
    --csv     logs/.../csv_sim/actions.csv      --action_scale 1.0
"""

import argparse
import glob
import os

import numpy as np
import pandas as pd

# ──────────────────────────────────────────────────────────────────────────────
# Policy joint order (23 DOFs – must match POLICY_JOINT_ORDER in prep_obs_csv.py)
# ──────────────────────────────────────────────────────────────────────────────
POLICY_JOINT_ORDER: list[str] = [
    # ── legs: hip ────────────────────────────────────────────────────────────
    "Joint_l_hip_yaw",
    "Joint_l_hip_roll",
    "Joint_l_hip_pitch",
    "Joint_r_hip_yaw",
    "Joint_r_hip_roll",
    "Joint_r_hip_pitch",
    # ── legs: knee ───────────────────────────────────────────────────────────
    "Joint_l_knee_pitch",
    "Joint_r_knee_pitch",
    # ── feet: ankle ──────────────────────────────────────────────────────────
    "Joint_l_ankle_pitch",
    "Joint_l_ankle_roll",
    "Joint_r_ankle_pitch",
    "Joint_r_ankle_roll",
    # ── arms: shoulder ───────────────────────────────────────────────────────
    "Joint_l_shoulder_pitch",
    "Joint_l_shoulder_roll",
    "Joint_l_shoulder_yaw",
    "Joint_r_shoulder_pitch",
    "Joint_r_shoulder_roll",
    "Joint_r_shoulder_yaw",
    # ── arms: elbow ──────────────────────────────────────────────────────────
    "Joint_l_elbow_pitch",
    "Joint_r_elbow_pitch",
    # ── others: neck + abdomen ────────────────────────────────────────────────
    "Joint_neck_pitch",
    "Joint_neck_yaw",
    "Joint_abdomen_yaw",
]
NUM_JOINTS = len(POLICY_JOINT_ORDER)  # 23

#: Action scale used during training (env.yaml: actions.joint_pos.scale)
ACTION_SCALE: float = 0.25

#: Action clipping range (env.yaml: actions.joint_pos.clip)
ACTION_CLIP: tuple[float, float] = (-100.0, 100.0)

#: Metadata columns in the joint_state CSV (not joint angles)
META_COLS = {"timestep", "timestamp"}


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def load_csv(path: str) -> pd.DataFrame:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"[ACT] File not found: {path}")
    df = pd.read_csv(path)
    print(f"[ACT] Loaded  {len(df)} rows × {len(df.columns)} cols  ←  {path}")
    return df


def _latest(files: list[str]) -> str:
    return max(files, key=os.path.getmtime)


def find_csv_sim_pos(load_run: str) -> tuple[str, str]:
    """
    Locate the latest *_position.csv inside  <load_run>/csv_sim/.

    Returns (pos_path, csv_sim_dir).
    """
    run_dir = os.path.abspath(load_run)
    if not os.path.isdir(run_dir):
        raise NotADirectoryError(f"[ACT] Not a directory: {run_dir}")

    csv_sim = os.path.join(run_dir, "csv_sim")
    if not os.path.isdir(csv_sim):
        raise FileNotFoundError(
            f"[ACT] 'csv_sim' not found in: {run_dir}\n"
            f"      Expected: {csv_sim}"
        )

    pos_files = glob.glob(os.path.join(csv_sim, "*position*.csv"))
    if not pos_files:
        raise FileNotFoundError(f"[ACT] No *position*.csv in {csv_sim}")

    chosen = _latest(pos_files)
    if len(pos_files) > 1:
        print(f"[ACT] Multiple position CSVs – using latest: {os.path.basename(chosen)}")
    else:
        print(f"[ACT] Position CSV: {os.path.basename(chosen)}")

    print(f"[ACT] csv_sim dir: {csv_sim}")
    return chosen, csv_sim


def extract_joints(df: pd.DataFrame) -> np.ndarray:
    """
    Extract POLICY_JOINT_ORDER columns from *df*.

    Returns float32 array (T, NUM_JOINTS), missing joints filled with 0.0.
    """
    T = len(df)
    out = np.zeros((T, NUM_JOINTS), dtype=np.float32)
    available = set(df.columns) - META_COLS
    found, missing = 0, []

    for j, jname in enumerate(POLICY_JOINT_ORDER):
        if jname in df.columns:
            out[:, j] = df[jname].values.astype(np.float32)
            found += 1
        else:
            missing.append(jname)

    print(f"[ACT] Mapped {found}/{NUM_JOINTS} joints.")
    if missing:
        print(f"[ACT] ⚠  Missing (filled 0.0): {missing}")

    unused = available - set(POLICY_JOINT_ORDER)
    if unused:
        print(f"[ACT] Unused CSV cols: {sorted(unused)}")

    return out


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__,
    )

    # ── source ────────────────────────────────────────────────────────────────
    parser.add_argument(
        "--load_run", type=str, default="",
        help=(
            "Path to the run folder. The script finds csv_sim/ inside it and "
            "picks the latest *_position.csv automatically."
        ),
    )
    parser.add_argument(
        "--pos_csv", type=str, default="",
        help="Explicit path to joint_state_*_position.csv (overrides --load_run).",
    )

    # ── output ────────────────────────────────────────────────────────────────
    parser.add_argument(
        "--out", type=str, default="",
        help=(
            "Output action CSV path. "
            "Default: <csv_sim_dir>/actions.csv when --load_run is used, "
            "otherwise actions.csv in the current directory."
        ),
    )

    # ── scaling ───────────────────────────────────────────────────────────────
    parser.add_argument(
        "--scale", type=float, default=ACTION_SCALE,
        help=f"Action scale used during training (default: {ACTION_SCALE}). "
             "Divides Δq to produce normalised action values.",
    )
    parser.add_argument(
        "--no_clip", action="store_true", default=False,
        help=f"Skip clipping actions to [{ACTION_CLIP[0]}, {ACTION_CLIP[1]}].",
    )
    parser.add_argument(
        "--info", action="store_true", default=False,
        help="Print joint mapping info and exit without writing output.",
    )

    args = parser.parse_args()

    # ── resolve paths ─────────────────────────────────────────────────────────
    csv_sim_dir = ""

    if args.load_run:
        pos_path, csv_sim_dir = find_csv_sim_pos(args.load_run)
        if args.pos_csv:        # explicit override
            pos_path = args.pos_csv
    elif args.pos_csv:
        pos_path = args.pos_csv
    else:
        parser.error("Provide either --load_run or --pos_csv.")

    out_path = args.out or (
        os.path.join(csv_sim_dir, "actions.csv") if csv_sim_dir else "actions.csv"
    )

    # ── load ──────────────────────────────────────────────────────────────────
    df_pos = load_csv(pos_path)

    if args.info:
        print("\n── joint_state_position columns ──────────────────────────────────")
        for i, c in enumerate(df_pos.columns):
            mark = "←" if c in POLICY_JOINT_ORDER else ("(meta)" if c in META_COLS else "skip")
            print(f"  [{i:2d}] {c:<42s} {mark}")
        print("\n── Policy joint order (23 joints) ─────────────────────────────────")
        for i, jn in enumerate(POLICY_JOINT_ORDER):
            present = "✓" if jn in df_pos.columns else "✗ MISSING"
            print(f"  [{i:2d}] {jn:<42s} {present}")
        return

    # ── extract raw positions (T, 23) ─────────────────────────────────────────
    joint_pos_raw = extract_joints(df_pos)         # absolute values from CSV
    T = joint_pos_raw.shape[0]

    # ── subtract initial (row-0) position ─────────────────────────────────────
    # The CSV stores positions relative to the robot's initial recording state.
    # q_relative[t] = q_measured[t] - q_measured[0]
    initial_pos = joint_pos_raw[0].copy()          # (23,)
    delta_q     = joint_pos_raw - initial_pos      # (T, 23)  Δq in radians

    print(f"[ACT] Initial (row-0) joint positions [rad]:")
    for j, (jn, v) in enumerate(zip(POLICY_JOINT_ORDER, initial_pos.tolist())):
        print(f"        [{j:2d}] {jn:<40s} {v:+.6f}")

    # ── convert Δq → normalised action ────────────────────────────────────────
    # a[t] = Δq[t] / scale
    # (inverse of:  q_target = q_default + a * scale)
    actions = delta_q / args.scale                 # (T, 23)
    print(f"\n[ACT] Divided by action scale = {args.scale}")

    # ── optional clipping ─────────────────────────────────────────────────────
    if not args.no_clip:
        lo, hi = ACTION_CLIP
        clipped = np.sum(
            (actions < lo) | (actions > hi)
        )
        actions = np.clip(actions, lo, hi)
        if clipped:
            print(f"[ACT] ⚠  Clipped {int(clipped)} values to [{lo}, {hi}].")
        else:
            print(f"[ACT] No values exceeded clip range [{lo}, {hi}].")

    # ── summary stats ─────────────────────────────────────────────────────────
    print(f"\n── Action statistics ({'after clipping' if not args.no_clip else 'no clipping'}) ──")
    print(f"  {'Joint':<42s} {'min':>8}  {'max':>8}  {'std':>8}")
    print(f"  {'-'*42} {'-'*8}  {'-'*8}  {'-'*8}")
    for j, jn in enumerate(POLICY_JOINT_ORDER):
        col = actions[:, j]
        print(f"  {jn:<42s} {col.min():>+8.4f}  {col.max():>+8.4f}  {col.std():>8.4f}")
    print(f"\n  Overall   min={actions.min():+.4f}  max={actions.max():+.4f}  "
          f"std={actions.std():.4f}  |actions|_∞={np.abs(actions).max():.4f}")

    # ── write output ──────────────────────────────────────────────────────────
    header = [f"act_{jn}" for jn in POLICY_JOINT_ORDER]
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    df_out = pd.DataFrame(actions, columns=header)
    df_out.to_csv(out_path, index=False, float_format="%.8f")

    print(f"\n[OUT] → {out_path}")
    print(f"[OUT] Shape: {T} steps × {NUM_JOINTS} joints")
    print(f"\n[HINT] To play with these actions:")
    print(f"       python csv_play.py --task ... --checkpoint ... \\")
    print(f"           --csv {out_path} --action_scale 1.0")


if __name__ == "__main__":
    main()
