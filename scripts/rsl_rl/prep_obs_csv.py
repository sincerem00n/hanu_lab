#!/usr/bin/env python3
"""
prep_obs_csv.py  –  Assemble a 78-dim policy-observation CSV from the three
                    simulation-recorded CSVs produced by the Hanu-A4 logger:

      joint_state_*_position.csv    – measured joint positions
      joint_state_*_velocity.csv    – measured joint velocities
      imu_record_*.csv              – IMU angular velocity + quaternion

    CSV column layout (confirmed from actual logs):

    joint_state_position / velocity
    ────────────────────────────────
    timestep, timestamp,
    Joint_E1R, Joint_abdomen_yaw, Joint_l_shoulder_pitch, Joint_neck_pitch,
    Joint_r_shoulder_pitch, Joint_l_hip_yaw, Joint_r_hip_yaw,
    Joint_l_shoulder_roll, Joint_neck_yaw, Joint_r_shoulder_roll,
    Joint_l_hip_pitch, Joint_r_hip_pitch, Joint_l_shoulder_yaw,
    Joint_r_shoulder_yaw, Joint_l_hip_roll, Joint_r_hip_roll,
    Joint_l_elbow_pitch, Joint_r_elbow_pitch, Joint_l_knee_pitch,
    Joint_r_knee_pitch, Joint_l_wrist_yaw, Joint_r_wrist_yaw,
    Joint_l_ankle_pitch, Joint_r_ankle_pitch, Joint_l_wrist_pitch,
    Joint_r_wrist_pitch, Joint_l_ankle_roll, Joint_r_ankle_roll,
    Joint_l_wrist_roll, Joint_r_wrist_roll

    imu_record
    ────────────
    timestep, timestamp,
    gyro_x, gyro_y, gyro_z,        ← angular velocity (rad/s)
    acc_x, acc_y, acc_z,           ← linear acceleration (m/s²)
    quat_w, quat_x, quat_y, quat_z,
    roll, pitch, yaw

Output policy observation vector (78 dims):
    cols  0– 2  base_ang_vel        (3)  gyro_x/y/z × 0.25 (policy-scaled)
    cols  3– 5  projected_gravity   (3)  from quaternion
    cols  6– 8  velocity_commands   (3)  constant (CLI args)
    cols  9–31  joint_pos          (23)  relative positions × 1.0
    cols 32–54  joint_vel          (23)  relative velocities × 0.05
    cols 55–77  actions            (23)  zeros (last-action term init)

Usage
─────
# Auto-discover CSVs from a run folder (recommended)
python prep_obs_csv.py --load_run logs/rsl_rl/hanu_a4_flat/2026-03-30_06-14-28 --vy_cmd 0.4
  → reads  logs/.../csv_sim/joint_state_*_position.csv  (latest)
  → reads  logs/.../csv_sim/joint_state_*_velocity.csv  (latest)
  → reads  logs/.../csv_sim/imu_record_*.csv            (latest)
  → writes logs/.../csv_sim/observations.csv

# Inspect column mapping without writing
python prep_obs_csv.py --load_run logs/rsl_rl/... --info

# Override individual files if needed
python prep_obs_csv.py --load_run logs/rsl_rl/... \\
    --pos_csv /path/to/custom_position.csv

# Manual mode (all three paths explicit)
python prep_obs_csv.py \\
    --pos_csv joint_state_*_position.csv \\
    --vel_csv joint_state_*_velocity.csv \\
    --imu_csv imu_record_*.csv \\
    --out observations.csv --vy_cmd 0.4

# Already-scaled output (no --obs_raw needed in csv_play.py)
python prep_obs_csv.py --load_run logs/rsl_rl/... --apply_scales

# Then play:
python csv_play.py --task ... --checkpoint ... \\
    --obs_csv logs/.../csv_sim/observations.csv --obs_raw
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

# ──────────────────────────────────────────────────────────────────────────────
# Exact column names from the recorded CSVs
# ──────────────────────────────────────────────────────────────────────────────

#: Columns to skip (non-joint metadata) in position / velocity CSVs
META_COLS = {"timestep", "timestamp"}

#: Angular-velocity columns in imu_record (rad/s, body frame)
IMU_GYRO_COLS  = ["gyro_x",  "gyro_y",  "gyro_z"]

#: Linear-acceleration columns (m/s², body frame) – used as fallback
IMU_ACC_COLS   = ["acc_x",   "acc_y",   "acc_z"]

#: Quaternion columns [w, x, y, z]
IMU_QUAT_COLS  = ["quat_w",  "quat_x",  "quat_y",  "quat_z"]

#: Timestamp column name (used to align the three CSVs)
TIMESTAMP_COL  = "timestamp"

# ──────────────────────────────────────────────────────────────────────────────
# Policy joint order (23 joints that enter the neural network)
#
# IsaacLab resolves joint_names_expr patterns in the ORDER they appear in the
# actuator groups.  For HANU_A4_CFG the groups are:
#   legs   → .*_hip_.*,  .*_knee_.*            (8 DOFs)
#   feet   → .*_ankle_.*                        (4 DOFs)
#   arms   → .*_shoulder_.*, .*_elbow_.*        (8 DOFs)
#   others → .*_neck_.*, .*_abdomen_.*          (3 DOFs)
#
# Within each group the articulation orders joints alphabetically by name
# (IsaacLab default with preserve_order=True).
# ──────────────────────────────────────────────────────────────────────────────
POLICY_JOINT_ORDER: list[str] = [
    # ── legs: hip (yaw < roll < pitch order per side, L before R) ─────────────
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
    # ── arms: shoulder (pitch < roll < yaw, L before R) ──────────────────────
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
NUM_JOINTS = len(POLICY_JOINT_ORDER)   # 23

# ──────────────────────────────────────────────────────────────────────────────
# Default joint positions (rad) from HANU_A4_CFG.init_state.joint_pos
# Used to compute joint_pos_rel = joint_pos_measured − default
# ──────────────────────────────────────────────────────────────────────────────
JOINT_DEFAULT_POS: dict[str, float] = {
    # legs
    "Joint_l_hip_yaw":    -0.08,
    "Joint_r_hip_yaw":    -0.08,
    "Joint_l_hip_roll":    0.02,
    "Joint_r_hip_roll":    0.02,
    "Joint_l_hip_pitch":  -0.14,
    "Joint_r_hip_pitch":  -0.14,
    "Joint_l_knee_pitch":  0.17,
    "Joint_r_knee_pitch":  0.17,
    # feet
    "Joint_l_ankle_pitch": 0.08,
    "Joint_r_ankle_pitch": 0.08,
    "Joint_l_ankle_roll":  0.02,
    "Joint_r_ankle_roll": -0.02,
    # arms
    "Joint_l_shoulder_pitch": -0.06,
    "Joint_r_shoulder_pitch": -0.06,
    "Joint_l_shoulder_roll":   0.25,
    "Joint_r_shoulder_roll":   0.25,
    "Joint_l_shoulder_yaw":    0.00,
    "Joint_r_shoulder_yaw":    0.00,
    "Joint_l_elbow_pitch":     0.00,
    "Joint_r_elbow_pitch":     0.00,
    # others
    "Joint_neck_pitch":    0.00,
    "Joint_neck_yaw":      0.00,
    "Joint_abdomen_yaw":   0.00,
}

# ──────────────────────────────────────────────────────────────────────────────
# Policy observation scales (from env.yaml)
# ──────────────────────────────────────────────────────────────────────────────
POLICY_SCALES = {
    "base_ang_vel":       0.25,
    "projected_gravity":  1.00,
    "velocity_commands":  1.00,
    "joint_pos":          1.00,
    "joint_vel":          0.05,
    "actions":            1.00,
}
OBS_DIM = 78


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def load_csv(path: str, label: str) -> pd.DataFrame:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"[{label}] Not found: {path}")
    df = pd.read_csv(path)
    print(f"[{label}] {len(df)} rows × {len(df.columns)} cols  ←  {path}")
    return df


def align(df_pos: pd.DataFrame,
          df_vel: pd.DataFrame,
          df_imu: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """
    Align the three DataFrames to the same time base.

    All three CSVs share the same timestep/timestamp column (they are
    recorded in lock-step by the Hanu logger).  If lengths differ, truncate
    to the shortest.
    """
    min_len = min(len(df_pos), len(df_vel), len(df_imu))
    if len(df_pos) != len(df_vel) or len(df_vel) != len(df_imu):
        print(f"[ALIGN] Length mismatch: pos={len(df_pos)}, vel={len(df_vel)}, "
              f"imu={len(df_imu)} → truncating to {min_len} rows.")
    return (
        df_pos.iloc[:min_len].reset_index(drop=True),
        df_vel.iloc[:min_len].reset_index(drop=True),
        df_imu.iloc[:min_len].reset_index(drop=True),
    )


def quat_to_projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Rotate the world-frame gravity direction [0, 0, -1] into the robot body
    frame using the body quaternion (w, x, y, z).

    This mirrors IsaacLab's ``projected_gravity`` observation term.

    Parameters
    ----------
    quat_wxyz : float32 array  (T, 4)

    Returns
    -------
    proj_g : float32 array  (T, 3)
    """
    q = quat_wxyz.copy().astype(np.float64)
    norm = np.linalg.norm(q, axis=-1, keepdims=True) + 1e-10
    q /= norm

    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    # Passive rotation: v_body = R^T · [0, 0, -1]
    # R^T columns from unit quaternion:
    #   R^T[:, 2] = [2(xz+wy), 2(yz-wx), 1-2(x²+y²)]
    # So R^T · e_z = that column, then negate for -e_z (gravity).
    gx = -(2 * (x * z + w * y))
    gy = -(2 * (y * z - w * x))
    gz = -(1 - 2 * (x * x + y * y))

    proj_g = np.stack([gx, gy, gz], axis=-1).astype(np.float32)
    # Re-normalise to unit length
    n = np.linalg.norm(proj_g, axis=-1, keepdims=True) + 1e-8
    return proj_g / n


def extract_joints(df: pd.DataFrame, label: str) -> np.ndarray:
    """
    Extract joints in POLICY_JOINT_ORDER from *df*.  Missing joints are
    filled with their default value (position) or 0.0 (velocity).

    Returns (T, NUM_JOINTS) float32 array.
    """
    T = len(df)
    out = np.zeros((T, NUM_JOINTS), dtype=np.float32)
    available = set(df.columns) - META_COLS

    for j, jname in enumerate(POLICY_JOINT_ORDER):
        if jname in df.columns:
            out[:, j] = df[jname].values.astype(np.float32)
        else:
            print(f"[{label}] ⚠  '{jname}' not found – filled with 0.0")

    found = sum(1 for jn in POLICY_JOINT_ORDER if jn in df.columns)
    print(f"[{label}] Mapped {found}/{NUM_JOINTS} joints.  "
          f"Unused input cols: {available - set(POLICY_JOINT_ORDER)}")
    return out


def print_info(df_pos: pd.DataFrame,
               df_vel: pd.DataFrame,
               df_imu: pd.DataFrame) -> None:
    sep = "─" * 60
    print(f"\n{sep}")
    print("  joint_state_position columns")
    print(f"{sep}")
    for i, c in enumerate(df_pos.columns):
        mark = "←" if c in POLICY_JOINT_ORDER else ("(meta)" if c in META_COLS else "skip")
        print(f"  [{i:2d}] {c:<40s} {mark}")

    print(f"\n{sep}")
    print("  joint_state_velocity columns  (same structure)")
    print(f"{sep}")
    for i, c in enumerate(df_vel.columns):
        mark = "←" if c in POLICY_JOINT_ORDER else ("(meta)" if c in META_COLS else "skip")
        print(f"  [{i:2d}] {c:<40s} {mark}")

    print(f"\n{sep}")
    print("  imu_record columns")
    print(f"{sep}")
    for i, c in enumerate(df_imu.columns):
        role = ""
        if c in IMU_GYRO_COLS:  role = "← base_ang_vel"
        elif c in IMU_ACC_COLS:  role = "← (fallback projected_gravity)"
        elif c in IMU_QUAT_COLS: role = "← projected_gravity"
        elif c in META_COLS:     role = "(meta)"
        print(f"  [{i:2d}] {c:<40s} {role}")

    print(f"\n{sep}")
    print(f"  Policy joint order ({NUM_JOINTS} joints)  →  obs cols 9–77")
    print(f"{sep}")
    for i, jn in enumerate(POLICY_JOINT_ORDER):
        dflt = JOINT_DEFAULT_POS.get(jn, 0.0)
        print(f"  [{i:2d}] {jn:<40s} default={dflt:+.4f} rad")
    print()


# ──────────────────────────────────────────────────────────────────────────────
# csv_sim auto-discovery
# ──────────────────────────────────────────────────────────────────────────────

def _latest(files: list[str]) -> str:
    """Return the most recently modified file from a list."""
    return max(files, key=os.path.getmtime)


def find_csv_sim_files(load_run: str) -> tuple[str, str, str, str]:
    """
    Locate the csv_sim/ directory inside *load_run* and return paths to the
    latest position, velocity, and IMU CSVs, plus the csv_sim directory itself.

    Parameters
    ----------
    load_run : str
        Path to the run folder, e.g.
        ``logs/rsl_rl/hanu_a4_flat/2026-03-30_06-14-28``.

    Returns
    -------
    (pos_path, vel_path, imu_path, csv_sim_dir)
    """
    load_run = os.path.abspath(load_run)
    if not os.path.isdir(load_run):
        raise NotADirectoryError(f"[LOAD_RUN] Not a directory: {load_run}")

    csv_sim_dir = os.path.join(load_run, "csv_sim")
    if not os.path.isdir(csv_sim_dir):
        raise FileNotFoundError(
            f"[LOAD_RUN] 'csv_sim' folder not found in: {load_run}\n"
            f"           Expected: {csv_sim_dir}"
        )

    import glob
    pos_files = glob.glob(os.path.join(csv_sim_dir, "*position*.csv"))
    vel_files = glob.glob(os.path.join(csv_sim_dir, "*velocity*.csv"))
    imu_files = glob.glob(os.path.join(csv_sim_dir, "imu_record*.csv"))

    def _require(files: list[str], pattern: str) -> str:
        if not files:
            raise FileNotFoundError(
                f"[LOAD_RUN] No file matching '{pattern}' in {csv_sim_dir}"
            )
        chosen = _latest(files)
        if len(files) > 1:
            print(f"[LOAD_RUN] Multiple matches for '{pattern}'; using latest: {os.path.basename(chosen)}")
        return chosen

    pos_path = _require(pos_files, "*position*.csv")
    vel_path = _require(vel_files, "*velocity*.csv")
    imu_path = _require(imu_files, "imu_record*.csv")

    print(f"[LOAD_RUN] csv_sim  →  {csv_sim_dir}")
    print(f"[LOAD_RUN]   pos    →  {os.path.basename(pos_path)}")
    print(f"[LOAD_RUN]   vel    →  {os.path.basename(vel_path)}")
    print(f"[LOAD_RUN]   imu    →  {os.path.basename(imu_path)}")

    return pos_path, vel_path, imu_path, csv_sim_dir


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__,
    )

    # ── source discovery ──────────────────────────────────────────────────────
    parser.add_argument(
        "--load_run", type=str, default="",
        help=(
            "Path to the run folder (e.g. logs/rsl_rl/hanu_a4_flat/2026-03-30_…). "
            "The script finds csv_sim/ inside it and picks the latest "
            "position / velocity / imu CSVs automatically. "
            "Overridden by --pos_csv / --vel_csv / --imu_csv when those are given."
        ),
    )
    parser.add_argument("--pos_csv", type=str, default="",
                        help="Override: explicit path to joint_state_*_position.csv")
    parser.add_argument("--vel_csv", type=str, default="",
                        help="Override: explicit path to joint_state_*_velocity.csv")
    parser.add_argument("--imu_csv", type=str, default="",
                        help="Override: explicit path to imu_record_*.csv")
    parser.add_argument(
        "--out", type=str, default="",
        help=(
            "Output path for the assembled observations CSV. "
            "Default: <csv_sim_dir>/observations.csv when --load_run is used, "
            "otherwise observations.csv in the current directory."
        ),
    )

    # ── velocity commands ─────────────────────────────────────────────────────
    parser.add_argument("--vx_cmd", type=float, default=0.0,
                        help="Commanded vx (m/s) – constant for all steps.")
    parser.add_argument("--vy_cmd", type=float, default=0.4,
                        help="Commanded vy (m/s) – constant for all steps.")
    parser.add_argument("--wz_cmd", type=float, default=0.0,
                        help="Commanded ωz (rad/s) – constant for all steps.")

    # ── processing options ────────────────────────────────────────────────────
    parser.add_argument("--apply_scales", action="store_true", default=False,
                        help=(
                            "Multiply each term by its policy scale before writing. "
                            "With this flag do NOT use --obs_raw in csv_play.py."
                        ))
    parser.add_argument("--no_rel_pos", action="store_true", default=False,
                        help="Write absolute joint positions instead of "
                             "subtracting the initial (row-0) values from the CSV.")
    parser.add_argument("--info", action="store_true", default=False,
                        help="Print column mapping info and exit without writing output.")
    args = parser.parse_args()

    # ── resolve source file paths ─────────────────────────────────────────────
    csv_sim_dir: str = ""

    if args.load_run:
        auto_pos, auto_vel, auto_imu, csv_sim_dir = find_csv_sim_files(args.load_run)
        # Allow explicit flags to override individual files
        pos_path = args.pos_csv or auto_pos
        vel_path = args.vel_csv or auto_vel
        imu_path = args.imu_csv or auto_imu
    else:
        # Fully manual mode – all three must be provided
        if not (args.pos_csv and args.vel_csv and args.imu_csv):
            parser.error(
                "Provide either --load_run OR all three of "
                "--pos_csv / --vel_csv / --imu_csv."
            )
        pos_path, vel_path, imu_path = args.pos_csv, args.vel_csv, args.imu_csv

    # ── resolve output path ───────────────────────────────────────────────────
    if args.out:
        out_path = args.out
    elif csv_sim_dir:
        out_path = os.path.join(csv_sim_dir, "observations.csv")
    else:
        out_path = "observations.csv"

    # ── load ──────────────────────────────────────────────────────────────────
    df_pos = load_csv(pos_path, "POS")
    df_vel = load_csv(vel_path, "VEL")
    df_imu = load_csv(imu_path, "IMU")

    if args.info:
        print_info(df_pos, df_vel, df_imu)
        return

    # ── align ─────────────────────────────────────────────────────────────────
    df_pos, df_vel, df_imu = align(df_pos, df_vel, df_imu)
    T = len(df_pos)
    print(f"[PREP] Building observation tensor: {T} steps × {OBS_DIM} dims")

    # ── 1. base_ang_vel (3) ── gyro_x/y/z ────────────────────────────────────
    ang_vel = np.zeros((T, 3), dtype=np.float32)
    for i, col in enumerate(IMU_GYRO_COLS):
        ang_vel[:, i] = df_imu[col].values.astype(np.float32)
    print(f"[IMU]  base_ang_vel  range: [{ang_vel.min():.4f}, {ang_vel.max():.4f}]")

    # ── 2. projected_gravity (3) ── from quaternion ───────────────────────────
    if all(c in df_imu.columns for c in IMU_QUAT_COLS):
        quat = df_imu[IMU_QUAT_COLS].values.astype(np.float32)
        proj_gravity = quat_to_projected_gravity(quat)
        print("[IMU]  projected_gravity computed from quaternion (quat_w/x/y/z).")
    else:
        print("[IMU]  ⚠  Quaternion columns not found – using upright fallback [0,0,-1].")
        proj_gravity = np.zeros((T, 3), dtype=np.float32)
        proj_gravity[:, 2] = -1.0

    # ── 3. velocity_commands (3) ── constant ──────────────────────────────────
    vel_cmd = np.zeros((T, 3), dtype=np.float32)
    vel_cmd[:, 0] = args.vx_cmd
    vel_cmd[:, 1] = args.vy_cmd
    vel_cmd[:, 2] = args.wz_cmd
    print(f"[CMD]  velocity_commands = [vx={args.vx_cmd}, vy={args.vy_cmd}, wz={args.wz_cmd}]")

    # ── 4. joint_pos (23) ── relative to initial CSV position ───────────────
    # The CSV stores positions relative to the robot's initial (row-0) state.
    # Subtracting row-0 gives joint_pos_rel = pos[t] - pos[0], which matches
    # the IsaacLab joint_pos_rel observation term (deviation from default pose).
    joint_pos = extract_joints(df_pos, "POS")
    if not args.no_rel_pos:
        initial_pos = joint_pos[0].copy()   # (23,) – actual initial joint angles
        joint_pos -= initial_pos
        print("[POS]  joint_pos_rel: subtracted initial (row-0) joint positions.")
        print(f"[POS]  initial row-0 values: {[f'{v:.4f}' for v in initial_pos.tolist()]}")
    else:
        print("[POS]  absolute joint positions (--no_rel_pos).")

    # ── 5. joint_vel (23) ─────────────────────────────────────────────────────
    joint_vel = extract_joints(df_vel, "VEL")

    # ── 6. actions (23) ── zeros ──────────────────────────────────────────────
    actions = np.zeros((T, NUM_JOINTS), dtype=np.float32)
    print("[ACT]  last-action term initialised to zeros.")

    # ── assemble (T, 78) ──────────────────────────────────────────────────────
    obs = np.concatenate([
        ang_vel,       # cols  0– 2
        proj_gravity,  # cols  3– 5
        vel_cmd,       # cols  6– 8
        joint_pos,     # cols  9–31
        joint_vel,     # cols 32–54
        actions,       # cols 55–77
    ], axis=-1)

    assert obs.shape == (T, OBS_DIM), f"Shape mismatch: {obs.shape}"

    # ── optional: apply policy scales ─────────────────────────────────────────
    if args.apply_scales:
        slices_scales = [
            ( 0,  3, POLICY_SCALES["base_ang_vel"]),
            ( 3,  6, POLICY_SCALES["projected_gravity"]),
            ( 6,  9, POLICY_SCALES["velocity_commands"]),
            ( 9, 32, POLICY_SCALES["joint_pos"]),
            (32, 55, POLICY_SCALES["joint_vel"]),
            (55, 78, POLICY_SCALES["actions"]),
        ]
        for s, e, sc in slices_scales:
            obs[:, s:e] *= sc
        print("[SCALE] Applied policy scales.  csv_play.py: do NOT use --obs_raw.")
    else:
        print("[SCALE] Raw sensor values.  csv_play.py: use --obs_raw.")

    # ── build column header ───────────────────────────────────────────────────
    header = (
        ["ang_vel_x", "ang_vel_y", "ang_vel_z"]
        + ["proj_grav_x", "proj_grav_y", "proj_grav_z"]
        + ["vel_cmd_x", "vel_cmd_y", "vel_cmd_z"]
        + [f"jpos_{j}" for j in POLICY_JOINT_ORDER]
        + [f"jvel_{j}" for j in POLICY_JOINT_ORDER]
        + [f"act_{j}"  for j in POLICY_JOINT_ORDER]
    )
    assert len(header) == OBS_DIM

    # ── write output ───────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    df_out = pd.DataFrame(obs, columns=header)
    df_out.to_csv(out_path, index=False, float_format="%.8f")

    print(f"\n[OUT]  → {out_path}")
    print(f"[OUT]  Shape: {obs.shape[0]} steps × {obs.shape[1]} cols")

    # ── summary ───────────────────────────────────────────────────────────────
    print("\n── Range summary ─────────────────────────────────────────────────────")
    term_slices = [
        ("base_ang_vel      [0:3]",   0,   3),
        ("projected_gravity [3:6]",   3,   6),
        ("velocity_commands [6:9]",   6,   9),
        ("joint_pos         [9:32]",  9,  32),
        ("joint_vel         [32:55]", 32, 55),
        ("actions           [55:78]", 55, 78),
    ]
    for lbl, s, e in term_slices:
        b = obs[:, s:e]
        print(f"  {lbl}  min={b.min():+8.4f}  max={b.max():+8.4f}  std={b.std():.4f}")
    print("──────────────────────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
