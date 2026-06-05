# XSens Awinda IMU Recording to OpenSim IK

A Python pipeline that converts raw **Xsens Awinda MTw2** IMU recordings into full-body inverse kinematics in OpenSim — without requiring MVN Analyze (Movella's paid software).

The pipeline follows the [OpenSense tutorial](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53084203/) and adds several practical fixes for real-world Xsens TXT exports.

---

## Overview

```
Raw Xsens .txt files
        │
        ▼
1_read_xsense_to_opensim.py   →   orientation .sto files
        │
        ▼
2_imu_placer.py               →   calibrated_model.osim
        │
        ▼
3_imu_ik.py                   →   kinematics .mot file
        │
        ▼  (optional, for squats / bilateral contact)
4_pelvis_height.py            →   grounded .mot file
```

A standalone bonus script (`visualizations/calculate_joint_angles.py`) computes joint angles directly from raw quaternions — no OpenSim required.

---

## Hardware & Software Requirements

| Item | Details |
|---|---|
| IMU system | Xsens Awinda with MTw2 sensors (8 sensors: pelvis, torso, both femurs, tibias, and calcanei) |
| Recording software | MT Manager (free from Movella) |
| Musculoskeletal model | Rajagopal 2015 with OpenSense IMU frames (`Rajagopal2015_opensense.osim`) |
| Python | 3.10+ via the `opensim` conda environment |

**Python dependencies**

```bash
conda create -n opensim -c opensim-org opensim
conda activate opensim
pip install numpy pandas scipy matplotlib
```

---

## Sensor Placement

Eight MTw2 sensors are placed on:

| Sensor label | Body segment |
|---|---|
| `pelvis_imu` | Sacrum / lower back |
| `torso_imu` | Sternum |
| `femur_r_imu` | Right thigh (lateral) |
| `femur_l_imu` | Left thigh (lateral) |
| `tibia_r_imu` | Right shin (lateral) |
| `tibia_l_imu` | Left shin (lateral) |
| `calcn_r_imu` | Right foot (dorsum) |
| `calcn_l_imu` | Left foot (dorsum) |

The mapping between hardware sensor IDs and body segments is defined in `myIMUMappings.xml` — one file per recording folder (see `example_data/IMU_Data/Dynamic/Walking/myIMUMappings.xml` for the format).

---

## Data Directory Layout

```
example_data/IMU_Data/
├── Static/
│   └── pose1/                   ← anatomical calibration pose (N-pose)
│       ├── MT_012012B9_000-000_<sensor_id>.txt
│       └── myIMUMappings.xml
└── Dynamic/
    ├── Walking/
    ├── Lunges/
    └── S2S/                     ← sit-to-stand
        ├── MT_012012B9_001-000_<sensor_id>.txt
        └── myIMUMappings.xml
```

Export each trial from MT Manager as **text files** (`.txt`). Keep the `.mtb` binary alongside in case you need to re-export.

---

## Pipeline Steps

### Step 1 — Read Xsens data → orientation STO files

```bash
conda activate opensim
python 1_read_xsense_to_opensim.py
```

Reads raw Xsens `.txt` quaternion files via OpenSim's `XsensDataReader` and writes orientation `.sto` files used by the rest of the pipeline.

**Known issue handled:** The MTw2 Awinda TXT export sets `SampleTimeFine = 0`, causing OpenSim to assume 40 Hz for all recordings. The script corrects timestamps by rescaling to the true trial duration (read from the `.mtb` file with MT Manager and passed as `actual_duration_s`).

**Outputs** (in `example_data/IMU_Data/`):
- `static_orientations.sto`
- `lunge_orientations.sto`
- `walking_orientations.sto`

Edit the `actual_duration_s` values in the script to match your own recordings.

---

### Step 2 — IMU Placer → calibrated model

```bash
python 2_imu_placer.py
```

Runs OpenSim's `IMUPlacer` to estimate the offset between each sensor's physical frame and the model body frame. The subject must stand in an anatomical (N-pose) reference position during the static recording.

**Input:** `static_orientations.sto`, `Rajagopal2015_opensense.osim`  
**Output:** `calibrated_model.osim`

---

### Step 3 — IMU IK → joint kinematics

```bash
python 3_imu_ik.py
```

Runs `IMUInverseKinematicsTool` to solve joint angles frame-by-frame by minimising orientation error between model IMU frames and sensor data.

Also clamps near-zero denormal values in the output that can crash the OpenSim GUI's `stod` parser (affects unconstrained degrees of freedom like knee-beta and subtalar).

**Input:** `walking_orientations.sto`, `calibrated_model.osim`  
**Output:** `walking.mot`

Visualise in the OpenSim GUI:
```
File → Open Model  → calibrated_model.osim
File → Load Motion → walking.mot
```

---

### Step 4 — Pelvis height grounding (optional)

```bash
python 4_pelvis_height.py
```

IMU IK only recovers orientations. The translational coordinate `pelvis_ty` stays at its default (0.93 m), so during a squat the model appears to levitate instead of dropping its hips.

This script runs forward kinematics on every frame, finds the lowest foot position, and sets `pelvis_ty` so the foot stays at ground level (y = 0).

**Suitable for:** squats and any motion where both feet remain in ground contact throughout.

**Input:** `calibrated_model.osim`, `squat_kinematics.mot`  
**Output:** `squat_kinematics_grounded.mot`

---

## Bonus: Joint Angles from Raw Quaternions

```bash
conda activate opensim
python visualizations/calculate_joint_angles.py example_data/IMU_Data/Dynamic/Walking
```

Computes hip, knee, and ankle flexion angles directly from the raw Xsens quaternion data — no OpenSim installation needed.

**Method:**
1. Compute relative rotation between proximal and distal sensor at each frame.
2. Normalise against the mean of the first ~1 s (neutral standing pose).
3. Find the dominant flexion axis via magnitude-weighted SVD (robust to mirror-image sensor mounting between left/right sides).
4. Project rotation vectors onto that axis → scalar angle in degrees.

**Options:**

```bash
# Use first 2 s as neutral reference instead of default 1 s
python visualizations/calculate_joint_angles.py <data_dir> --ref-frames 80

# Override sample rate if auto-detection fails
python visualizations/calculate_joint_angles.py <data_dir> --sample-rate 100
```

**Output:** `joint_angles.png` saved in the data directory.

---

## Visualize Motion (Python)
- under construction (visulization should be done in OpenSim for now)

---

## Example Data

The `example_data/` folder contains recordings of one subject performing:
- **Walking** (forward gait)
- **Lunges**
- **Sit-to-Stand (S2S)**
- **Static pose** (anatomical calibration)

Pre-generated orientation `.sto` files and a calibrated model are included so you can run steps 2–4 without re-running step 1.

---

## Adapting to Your Own Data

1. Record a static anatomical pose and your dynamic trials in MT Manager.
2. Export each trial as `.txt` files into separate folders.
3. Create a `myIMUMappings.xml` in each folder (copy from `example_data` and update the `trial_prefix` and sensor IDs).
4. In `1_read_xsense_to_opensim.py`, update `ANAT_DIR`, `SQUAT_DIR`, and the `actual_duration_s` values to match your recordings.
5. Run scripts 1 → 2 → 3 (→ 4 for squats).

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Wrong motion speed / timing | Set `actual_duration_s` in step 1 to the true trial length from MT Manager |
| OpenSim GUI fails to load `.mot` | Step 3 cleans denormals; re-run if you modified the output manually |
| Pelvis floats during squat | Run step 4 (`4_pelvis_height.py`) |
| Joint angles look mirrored | Check the `SENSOR_TO_OPENSIM` rotation in step 2; must match your sensor mounting convention |
| `calculate_joint_angles.py` shows flat signal | Increase `--ref-frames` or verify the XML mapping file sensor IDs match the `.txt` filenames |
