"""
Step 1 — Read Xsens data and write orientation STO files.

Follows the OpenSense tutorial:
  https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53084203/

Reads raw Xsens .txt files via OpenSim's XsensDataReader and writes two
orientation .sto files (anatomical pose + squat).  The IMU Placer in step 2
attaches and calibrates IMU frames directly on the base model, so no model
modification is needed here.

Usage:
    conda activate opensim
    python 1_read_xsense_to_opensim.py

Outputs (all in DATASET_DIR):
    anatomical_pose_orientations.sto
    squat_orientations.sto
"""

import opensim as osim
from pathlib import Path

DATASET_DIR = Path("./example_data/IMU_Data/")
ANAT_DIR  = DATASET_DIR / "Static/pose1"
SQUAT_DIR = DATASET_DIR / "Dynamic/Lunges"
WALK_DIR  = DATASET_DIR / "Dynamic/Walking"  # optional third trial

ANAT_STO_OUT  = DATASET_DIR / "static_orientations.sto"
SQUAT_STO_OUT = DATASET_DIR / "lunge_orientations.sto"
WALK_STO_OUT  = DATASET_DIR / "walking_orientations.sto"  # optional third trial


def read_and_write_sto(data_dir: Path, out_path: Path, actual_duration_s: float = None):
    settings = osim.XsensDataReaderSettings(str(data_dir / "myIMUMappings.xml"))
    reader   = osim.XsensDataReader(settings)
    tables   = reader.read(str(data_dir) + "/")

    table = reader.getOrientationsTable(tables)

    if actual_duration_s is not None:
        # MTw2 Awinda: SampleTimeFine is 0 in TXT export, so OpenSim assumes 40 Hz.
        # Rescale timestamps to match the true duration read from the MTB file.
        raw_times = table.getIndependentColumn()
        scale = actual_duration_s / raw_times[-1]
        fixed = osim.TimeSeriesTableQuaternion()
        fixed.setColumnLabels(table.getColumnLabels())
        for i in range(table.getNumRows()):
            fixed.appendRow(raw_times[i] * scale, table.getRowAtIndex(i))
        table = fixed

    osim.STOFileAdapterQuaternion.write(table, str(out_path))
    print(f"  Wrote: {out_path.name}  ({table.getNumRows()} frames, t_end={table.getIndependentColumn()[-1]:.3f} s)")


print("Reading Xsens data ...")
read_and_write_sto(ANAT_DIR,  ANAT_STO_OUT,  actual_duration_s=12.1253)
read_and_write_sto(SQUAT_DIR, SQUAT_STO_OUT, actual_duration_s=22.22)  # fill in from MTB
read_and_write_sto(WALK_DIR,  WALK_STO_OUT,  actual_duration_s=23.2387)  # fill in from MTB
print("\nNext: run 2_imu_placer.py")