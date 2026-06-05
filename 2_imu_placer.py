"""
Step 3 — IMU Placer: calibrate sensor offset frames from the static pose.

Usage:
    conda activate opensim
    python 3_imu_placer.py

Input  (from step 2):  static_orientations.sto
Output:                calibrated_model.osim
"""

import numpy as np
import opensim as osim
from pathlib import Path

PIPELINE_DIR = Path(__file__).resolve().parent

# YOUR data — step 2 flips the 180°X-rotated sensors so tutorial settings apply:
DATASET_DIR   = Path("example_data/IMU_Data")
ANAT_STO      = DATASET_DIR / "static_orientations.sto"    # ← reoriented by step 2
SENSOR_TO_OPENSIM = osim.Vec3(np.deg2rad(90), 0, 0)
BASE_HEADING_AXIS = "z"

MODEL_IN  = Path("example_data/Rajagopal2015_opensense.osim")
MODEL_OUT = PIPELINE_DIR / "calibrated_model.osim"
BASE_IMU_LABEL = "pelvis_imu"

placer = osim.IMUPlacer()
placer.set_model_file(str(MODEL_IN))
placer.set_orientation_file_for_calibration(str(ANAT_STO))
placer.set_sensor_to_opensim_rotations(SENSOR_TO_OPENSIM)
placer.set_base_imu_label(BASE_IMU_LABEL)
placer.set_base_heading_axis(BASE_HEADING_AXIS)
placer.run()

model = placer.getCalibratedModel()
model.printToXML(str(MODEL_OUT))
print(f"Wrote: {MODEL_OUT.name}")
print("\nNext: run 3_imu_ik.py")
