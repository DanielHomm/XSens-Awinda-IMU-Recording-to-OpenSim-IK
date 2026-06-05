"""
Step 5 — Ground-constrain pelvis height.

IMU IK only solves orientations.  pelvis_ty stays at its default (0.93 m), so
during a squat the model appears to lift its feet instead of dropping its hips.
This script runs forward kinematics on each frame, finds where the feet end up,
and sets pelvis_ty so the lower foot stays at y=0 (ground).

Valid for squats and any bilateral-contact motion (both feet always grounded).

Usage:
    conda activate opensim
    python 5_pelvis_height.py

Inputs (from steps 3–4):
    calibrated_model.osim
    squat_kinematics.mot      (or static_kinematics.mot)

Output:
    squat_kinematics_grounded.mot
"""

import numpy as np
import pandas as pd
import opensim as osim
from pathlib import Path

PIPELINE_DIR = Path(__file__).resolve().parent

CALIBRATED_MODEL = PIPELINE_DIR / "calibrated_model.osim"
INPUT_MOT        = PIPELINE_DIR / "squat_kinematics.mot"
OUTPUT_MOT       = PIPELINE_DIR / "squat_kinematics_grounded.mot"

GROUND_Y = 0.0

TRANSLATIONAL = {"pelvis_tx", "pelvis_ty", "pelvis_tz"}


def read_mot(path):
    with open(path) as f:
        lines = f.readlines()
    end = next(i for i, ln in enumerate(lines) if "endheader" in ln)
    return pd.read_csv(path, sep="\t", skiprows=end + 1)


def write_mot(path, df):
    n_rows, n_cols = df.shape
    with open(path, "w") as f:
        f.write(f"Coordinates\nnRows={n_rows}\nnColumns={n_cols}\n"
                f"inDegrees=yes\n\nendheader\n")
        df.to_csv(f, sep="\t", index=False)


model = osim.Model(str(CALIBRATED_MODEL))
state = model.initSystem()

df       = read_mot(INPUT_MOT)
mot_cols = set(df.columns) - {"time"}

coord_set = model.getCoordinateSet()
calcn_r   = model.getBodySet().get("calcn_r")
calcn_l   = model.getBodySet().get("calcn_l")
ty_coord  = coord_set.get("pelvis_ty")

settable = [c for c in mot_cols if coord_set.contains(c)]

pelvis_ty_values = []
for _, row in df.iterrows():
    for cname in settable:
        coord = coord_set.get(cname)
        val   = float(row[cname])
        coord.setValue(state, np.radians(val) if cname not in TRANSLATIONAL else val)

    ty_coord.setValue(state, 0.0)
    model.realizePosition(state)

    h_r = calcn_r.getPositionInGround(state).get(1)
    h_l = calcn_l.getPositionInGround(state).get(1)
    pelvis_ty_values.append(GROUND_Y - min(h_r, h_l))

df_out = df.copy()
df_out["pelvis_ty"] = pelvis_ty_values

write_mot(OUTPUT_MOT, df_out)
print(f"Wrote: {OUTPUT_MOT.name}")
print(f"\nVisualize in OpenSim GUI:")
print(f"  File -> Open Model  -> {CALIBRATED_MODEL}")
print(f"  File -> Load Motion -> {OUTPUT_MOT}")
