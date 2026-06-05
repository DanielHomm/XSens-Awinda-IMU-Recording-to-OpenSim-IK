"""
calculate_joint_angles.py

Compute and plot joint angles directly from Xsens IMU quaternions.

Joints computed
---------------
  Hip R / Hip L    : pelvis_imu → femur_r/l_imu
  Knee R / Knee L  : femur_r/l_imu → tibia_r/l_imu
  Ankle R / Ankle L: tibia_r/l_imu → calcn_r/l_imu

Angle method
------------
  Raw Euler decomposition (ZXY) does not work reliably when sensors on the left
  and right sides are mounted as mirror images of each other — the flexion signal
  ends up in different Euler components per side.

  Instead, for each joint:
    1. Compute the relative rotation q_rel = q_prox^-1 * q_dist at every frame.
    2. Normalise by the mean of the first REF_FRAMES frames (neutral standing pose).
       → deviation rotation is the identity at t≈0.
    3. Convert deviation rotations to rotation vectors [N, 3].
    4. Find the dominant motion axis via PCA (SVD of the rotation-vector matrix).
    5. Project each rotation vector onto that axis → scalar angle in degrees.
    6. Sign convention: positive = flexion (dominant excursion is positive).

  This is independent of sensor mounting orientation and is directly comparable
  between the left and right sides without any explicit calibration.

Usage
-----
    conda activate opensim
    python calculate_joint_angles.py [data_directory]

    # Use first 2 s as reference instead of default 1 s:
    python calculate_joint_angles.py [data_directory] --ref-frames 80
"""

import os
import sys
import glob
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


SAMPLE_RATE = 100  # Hz  default; auto-detected from file header when possible
REF_FRAMES  = 100  # frames averaged as the neutral reference pose (~1 s)

JOINT_DEFINITIONS = [
    ("Hip Right",   "pelvis_imu", "femur_r_imu"),
    ("Hip Left",    "pelvis_imu", "femur_l_imu"),
    ("Knee Right",  "femur_r_imu", "tibia_r_imu"),
    ("Knee Left",   "femur_l_imu", "tibia_l_imu"),
    ("Ankle Right", "tibia_r_imu", "calcn_r_imu"),
    ("Ankle Left",  "tibia_l_imu", "calcn_l_imu"),
]

COLORS = {
    "Hip Right":   "#1f77b4",
    "Hip Left":    "#aec7e8",
    "Knee Right":  "#2ca02c",
    "Knee Left":   "#98df8a",
    "Ankle Right": "#d62728",
    "Ankle Left":  "#ff9896",
}


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def detect_sample_rate(txt_path: str) -> int | None:
    """Read the update rate from the Xsens TXT file header comments."""
    with open(txt_path, "r") as fh:
        for line in fh:
            if not line.startswith("//"):
                break
            low = line.lower()
            if "update rate" in low or "sample rate" in low or "output rate" in low:
                for token in line.split():
                    try:
                        rate = float(token)
                        if 10 < rate < 2000:
                            return int(rate)
                    except ValueError:
                        pass
    return None


def find_mapping_xml(directory: str) -> str:
    matches = glob.glob(os.path.join(directory, "*.xml"))
    if not matches:
        raise FileNotFoundError(f"No XML mapping file found in: {directory}")
    return matches[0]


def parse_imu_mapping(xml_path: str):
    """Return (trial_prefix, {segment_name: sensor_hardware_id})."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    prefix_elem = root.find(".//trial_prefix")
    trial_prefix = prefix_elem.text.strip() if prefix_elem is not None else ""

    seg_map = {}
    for sensor in root.findall(".//ExperimentalSensor"):
        # Format A (OpenSim 4.x): name attr = "_<sensor_id>", segment in <name_in_model>
        # Format B (older):        name attr = segment_name,   sensor in <xsens_imu_name>
        name_attr     = sensor.get("name", "").strip()
        name_in_model = sensor.findtext("name_in_model", "").strip()
        xsens_id      = sensor.findtext("xsens_imu_name", "").strip()

        if name_in_model and name_attr.startswith("_"):
            seg_map[name_in_model] = name_attr.lstrip("_")
        elif name_attr and xsens_id:
            seg_map[name_attr] = xsens_id

    return trial_prefix, seg_map


def find_sensor_file(directory: str, trial_prefix: str, sensor_id: str):
    for pattern in [
        os.path.join(directory, f"{trial_prefix}_{sensor_id}.txt"),
        os.path.join(directory, f"*{sensor_id}*.txt"),
    ]:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None


def load_quaternions(txt_path: str) -> np.ndarray:
    """Load Xsens .txt and return [N, 4] in [w, x, y, z] order."""
    header_idx = None
    with open(txt_path, "r") as fh:
        for i, line in enumerate(fh):
            if line.strip().startswith("PacketCounter"):
                header_idx = i
                break
    if header_idx is None:
        raise ValueError(f"Column header not found in {txt_path}")

    df = pd.read_csv(txt_path, sep="\t", skiprows=header_idx, low_memory=False)
    df.columns = [c.strip() for c in df.columns]
    df = df.loc[:, ~df.columns.str.startswith("Unnamed")]
    return df[["Quat_q0", "Quat_q1", "Quat_q2", "Quat_q3"]].values.astype(float)


# ---------------------------------------------------------------------------
# Angle computation
# ---------------------------------------------------------------------------

def _to_scipy(q_wxyz: np.ndarray) -> Rotation:
    return Rotation.from_quat(q_wxyz[:, [1, 2, 3, 0]])  # scipy: [x,y,z,w]


def _deviation_rotvecs(q_prox: np.ndarray, q_dist: np.ndarray,
                       n_ref: int = REF_FRAMES):
    """
    Shared setup: relative rotation, reference subtraction, rotation vectors.
    Returns (rotvecs [N,3], ref_angle_deg, r_ref).
    """
    r_p   = _to_scipy(q_prox)
    r_d   = _to_scipy(q_dist)
    r_rel = r_p.inv() * r_d

    n_ref    = min(n_ref, len(q_prox) // 4)
    r_ref    = r_rel[:n_ref].mean()
    ref_angle = float(np.degrees(np.linalg.norm(r_ref.as_rotvec())))

    r_delta = r_ref.inv() * r_rel
    rotvecs = r_delta.as_rotvec()          # axis*angle vectors, [N, 3]
    return rotvecs, ref_angle, r_ref


def pca_diagnostics(rotvecs: np.ndarray):
    """
    Return (Vt [3,3], singular_values [3], variance_ratios [3]).
    Rows of Vt are the PCA axes (PC1 first).
    """
    centered = rotvecs - rotvecs.mean(axis=0)
    _, s, Vt = np.linalg.svd(centered, full_matrices=False)
    var = s ** 2
    return Vt, s, var / var.sum()


def flexion_axis_weighted(rotvecs: np.ndarray, min_frac: float = 0.05):
    """
    Find the dominant rotation axis by weighting each frame by its rotation
    magnitude (frames near peak flexion define the axis most reliably).
    Only frames with magnitude > min_frac * peak are used.
    """
    mags = np.linalg.norm(rotvecs, axis=1)
    peak = mags.max()
    if peak < 1e-6:
        return np.array([0.0, 0.0, 1.0])

    mask = mags > peak * min_frac
    if mask.sum() < 5:
        mask = np.ones(len(mags), dtype=bool)

    # Scale each rotation vector by its own magnitude before SVD.
    # This makes large-rotation frames dominate the direction finding.
    weighted = rotvecs[mask] * mags[mask, None]
    _, _, Vt = np.linalg.svd(weighted - weighted.mean(axis=0), full_matrices=False)
    return Vt[0]


def pca_joint_angle(q_prox: np.ndarray, q_dist: np.ndarray,
                    n_ref: int = REF_FRAMES,
                    label: str = "") -> np.ndarray:
    """
    Reference-normalised joint angle projected onto the dominant flexion axis.

    Uses magnitude-weighted SVD (instead of plain PCA) so that frames with
    large rotation define the axis — more robust when secondary motions exist.
    Prints diagnostics so axis quality can be assessed.
    """
    rotvecs, ref_angle, _ = _deviation_rotvecs(q_prox, q_dist, n_ref)

    # Magnitude-weighted axis (primary method)
    axis = flexion_axis_weighted(rotvecs)
    raw = np.degrees(rotvecs @ axis)

    # Step 1 — ensure squatting/flexion direction is positive
    if abs(raw.min()) > abs(raw.max()):
        raw  = -raw
        axis = -axis

    # Step 2 — shift so 0° = most-extended position anywhere in the trial.
    # This is robust to the reference window being captured mid-motion
    # (e.g. the trial starting with the joint already partially flexed).
    # For a correctly captured reference the shift is ~0; when the reference
    # is offset by N° the shift recovers the full range automatically.
    angles = raw - raw.min()

    # --- diagnostics ---
    _, _, var_ratio = pca_diagnostics(rotvecs)

    # Max deviation rotation magnitude (upper bound on any projected angle)
    max_rot_mag = float(np.max(np.linalg.norm(rotvecs, axis=1)))

    # Bending angle range: raw angle between sensor Z-axes in global frame.
    # Independent of reference subtraction — shows whether the sensors physically
    # move through the expected range of motion.
    z_p = _to_scipy(q_prox).apply([0.0, 0.0, 1.0])
    z_d = _to_scipy(q_dist).apply([0.0, 0.0, 1.0])
    cos_b = np.clip(np.einsum("ij,ij->i", z_p, z_d), -1.0, 1.0)
    bending = np.degrees(np.arccos(cos_b))
    bend_range = float(bending.max() - bending.min())

    print(f"  {label:<14} "
          f"ref={ref_angle:5.1f}°  "
          f"var=[{var_ratio[0]:.0%},{var_ratio[1]:.0%},{var_ratio[2]:.0%}]  "
          f"max_rot={max_rot_mag:5.1f}°  "
          f"bend_rng={bend_range:5.1f}°  "
          f"→ peak {angles.max():.1f}°")

    return angles


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_joint_angles(t: np.ndarray, results: dict, save_path: str):
    groups = [
        ("Hip Flexion",   ["Hip Right",   "Hip Left"]),
        ("Knee Flex/Ext", ["Knee Right",  "Knee Left"]),
        ("Ankle Angle",   ["Ankle Right", "Ankle Left"]),
    ]
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)

    for ax, (title, joints) in zip(axes, groups):
        for jn in joints:
            if jn in results:
                ax.plot(t, results[jn], color=COLORS[jn], lw=1.8, label=jn)
        ax.set_title(title, fontsize=11)
        ax.set_ylabel("Angle (deg)")
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="k", lw=0.5, ls="--")

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(
        "Joint Angles from IMU Data\n"
        "Reference-normalised, PCA-projected  |  0 ° = initial standing pose",
        fontsize=12,
    )
    plt.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {save_path}")
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Calculate joint angles from Xsens IMU data.")
    parser.add_argument("data_dir", nargs="?", help="Directory containing .txt files and mapping XML")
    parser.add_argument("--ref-frames", type=int, default=REF_FRAMES,
                        help=f"Frames averaged as neutral reference pose (default {REF_FRAMES})")
    parser.add_argument("--sample-rate", type=float, default=None,
                        help="Sensor sample rate in Hz (auto-detected from file header if omitted)")
    args = parser.parse_args()

    data_dir = args.data_dir
    if not data_dir:
        data_dir = input("Enter path to data directory: ").strip().strip('"')
    data_dir = os.path.normpath(data_dir)

    print(f"\nData directory : {data_dir}")
    xml_path = find_mapping_xml(data_dir)
    print(f"Mapping file   : {os.path.basename(xml_path)}")

    trial_prefix, seg_map = parse_imu_mapping(xml_path)
    print(f"Trial prefix   : {trial_prefix}")

    # Load all needed sensors
    quats: dict[str, np.ndarray] = {}
    needed = {seg for _, p, d in JOINT_DEFINITIONS for seg in (p, d)}
    first_file = None

    print("\nLoading sensor files:")
    for seg in sorted(needed):
        if seg not in seg_map:
            print(f"  SKIP  {seg}  (not in mapping)")
            continue
        sid  = seg_map[seg]
        path = find_sensor_file(data_dir, trial_prefix, sid)
        if path is None:
            print(f"  MISS  {seg}  (no file found for sensor id={sid})")
            continue
        quats[seg] = load_quaternions(path)
        if first_file is None:
            first_file = path
        print(f"  OK    {seg}  — {quats[seg].shape[0]} samples  ({os.path.basename(path)})")

    if not quats:
        sys.exit("No sensor data loaded. Verify the data directory and mapping file.")

    # Determine sample rate: CLI > auto-detect from file > default
    fs = args.sample_rate
    if fs is None and first_file is not None:
        fs = detect_sample_rate(first_file)
    if fs is None:
        fs = SAMPLE_RATE
        print(f"\nWARNING: Could not detect sample rate from file header; "
              f"using default {SAMPLE_RATE} Hz. Override with --sample-rate.")
    else:
        print(f"\nSample rate      : {fs:.0f} Hz")

    # Align to shortest recording
    n = min(q.shape[0] for q in quats.values())
    quats = {k: v[:n] for k, v in quats.items()}
    t = np.arange(n) / fs
    print(f"Recording length : {n} samples  ({t[-1]:.2f} s)")
    print(f"Reference frames : {args.ref_frames}  ({args.ref_frames / fs:.2f} s)\n")

    # Compute joint angles
    results: dict[str, np.ndarray] = {}
    for label, prox, dist in JOINT_DEFINITIONS:
        if prox not in quats or dist not in quats:
            print(f"  SKIP  {label}  (missing segment data)")
            continue
        angles = pca_joint_angle(quats[prox], quats[dist],
                                 n_ref=args.ref_frames, label=label)
        results[label] = angles

    if not results:
        sys.exit("No joint angles could be computed.")

    out_path = os.path.join(data_dir, "joint_angles.png")
    print(f"\nGenerating figure:")
    plot_joint_angles(t, results, out_path)
    plt.show()


if __name__ == "__main__":
    main()
