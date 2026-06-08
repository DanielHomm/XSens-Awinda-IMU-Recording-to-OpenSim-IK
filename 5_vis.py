import opensim as osim
import os
import time
import numpy as np

# ── Paths / options — edit these ──────────────────────────────────────────────
CALIBRATED_MODEL = "./calibrated_model.osim"
MOTION_FILE      = "walking.mot"
GEOMETRY_PATH    = "/Users/arbeit/Applications/OpenSim 4.5/OpenSim 4.5.app/Contents/Resources/opensim/Geometry"
SPEED            = 1.0   # playback speed multiplier (0.5 = half, 2.0 = double)

# ── Add geometry so the model renders correctly ───────────────────────────────
osim.ModelVisualizer.addDirToGeometrySearchPaths(GEOMETRY_PATH)
osim.Logger.setLevelString("Error")   # silence per-frame coordinate warnings

# ── Load model ────────────────────────────────────────────────────────────────
print(f"Loading model:  {os.path.basename(CALIBRATED_MODEL)}")
print(f"Loading motion: {os.path.basename(MOTION_FILE)}")

model = osim.Model(CALIBRATED_MODEL)
# Unlock all coordinates so upper body renders in neutral pose
for i in range(model.getCoordinateSet().getSize()):
    model.getCoordinateSet().get(i).set_locked(False)
model.finalizeConnections()

model.setUseVisualizer(True)
state = model.initSystem()

# ── Load motion and map columns → coordinates ─────────────────────────────────
table      = osim.TimeSeriesTable(MOTION_FILE)
in_degrees = table.getTableMetaDataAsString("inDegrees") == "yes"
labels     = list(table.getColumnLabels())
times      = list(table.getIndependentColumn())
n_frames   = table.getNumRows()

coord_set = model.getCoordinateSet()
plan = []   # (column_index, coordinate, is_rotational)
for ci, lab in enumerate(labels):
    try:
        coord = coord_set.get(lab)
    except Exception:
        continue   # column with no matching coordinate
    plan.append((ci, coord, coord.getMotionType() == osim.Coordinate.Rotational))

def apply_frame(i):
    row = table.getRowAtIndex(i)
    for ci, coord, rotational in plan:
        v = row[ci]
        if in_degrees and rotational:
            v = np.deg2rad(v)
        coord.setValue(state, v, False)
    model.realizePosition(state)

# ── Configure the simbody-visualizer so the WHOLE model is in view ─────────────
# (VisualizerUtilities.showMotion gives no camera control, so its default framing
#  clips the model out of view — you get an empty white window. Driving the
#  visualizer directly lets us zoom to fit head-to-feet.)
viz = model.updVisualizer().updSimbodyVisualizer()
viz.setShowSimTime(True)
viz.setGroundHeight(-1.5)            # drop ground plane so feet aren't occluded

apply_frame(0)
state.setTime(times[0])
viz.report(state)
viz.zoomCameraToShowAllGeometry()    # frame the whole skeleton

print(f"Playing {n_frames} frames at {SPEED}x. Close the window or press Ctrl-C to stop.")

# ── Play (loops until the window is closed / Ctrl-C) ──────────────────────────
try:
    while True:
        for i in range(n_frames):
            tic = time.time()
            apply_frame(i)
            state.setTime(times[i])
            viz.report(state)
            if i + 1 < n_frames:
                rem = (times[i + 1] - times[i]) / SPEED - (time.time() - tic)
                if rem > 0:
                    time.sleep(rem)
except KeyboardInterrupt:
    print("\nStopped.")
except Exception as e:
    print(f"\nVisualizer closed: {e}")

print("Done.")
