"""
Step 4 — IMU Inverse Kinematics.

Usage:
    conda activate opensim
    python 4_imu_ik.py

Inputs (from steps 2–3):  squat_orientations.sto
                          calibrated_model.osim
Output:                   squat_kinematics.mot
"""

import numpy as np
import opensim as osim
from pathlib import Path

IN_DIR = Path("example_data/IMU_Data")
OUT_DIR = Path(__file__).resolve().parent

CALIBRATED_MODEL = OUT_DIR / "calibrated_model.osim"
SQUAT_STO        = IN_DIR / "walking_orientations.sto"    # ← reoriented by step 2
OUTPUT_MOT       = OUT_DIR / "walking.mot"

model = osim.Model(str(CALIBRATED_MODEL))
model.finalizeConnections()

tool = osim.IMUInverseKinematicsTool()
tool.setModel(model)
tool.set_sensor_to_opensim_rotations(osim.Vec3(np.deg2rad(90), 0, 0))
tool.set_orientations_file(str(SQUAT_STO))
tool.setResultsDir(str(OUT_DIR))
tool.setOutputMotionFileName(str(OUTPUT_MOT))
tool.run()

# Clamp denormal values (e.g. unconstrained knee-beta, subtalar) that the
# GUI parser (stod) cannot handle.
lines = OUTPUT_MOT.read_text().splitlines()
end   = next(i for i, ln in enumerate(lines) if "endheader" in ln)
cleaned = lines[:end + 2]
for ln in lines[end + 2:]:
    parts = ln.split("\t")
    cleaned.append("\t".join(
        "0" if abs(float(v)) < 1e-10 else v for v in parts
    ))
OUTPUT_MOT.write_text("\n".join(cleaned) + "\n")

print(f"Wrote: {OUTPUT_MOT.name}")
print(f"\nVisualize in OpenSim GUI:")
print(f"  File -> Open Model  -> {CALIBRATED_MODEL}")
print(f"  File -> Load Motion -> {OUTPUT_MOT}")
print("If you want the pelvis to drop during the squat run: 4_pelvis_height.py")
