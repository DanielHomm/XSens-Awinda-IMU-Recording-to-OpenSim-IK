import opensim as osim
import os

# ── Paths — edit these ────────────────────────────────────────────────────────
CALIBRATED_MODEL = "./calibrated_model.osim"
MOTION_FILE      = "./walking.mot"
GEOMETRY_PATH    = r"C:\Simulation\OpenSim 4.5\Geometry"

# ── Add geometry so model renders correctly ───────────────────────────────────
osim.ModelVisualizer.addDirToGeometrySearchPaths(GEOMETRY_PATH)

# ── Load model and motion ─────────────────────────────────────────────────────
print(f"Loading model:  {os.path.basename(CALIBRATED_MODEL)}")
print(f"Loading motion: {os.path.basename(MOTION_FILE)}")

model = osim.Model(CALIBRATED_MODEL)
# Unlock all coordinates so upper body renders in neutral pose
for i in range(model.getCoordinateSet().getSize()):
    model.getCoordinateSet().get(i).set_locked(False)
model.finalizeConnections()

model.setUseVisualizer(True)
state = model.initSystem()

motion_table = osim.TimeSeriesTable(MOTION_FILE)

try:
    osim.VisualizerUtilities.showMotion(model, motion_table)
except Exception as e:
    print(f"Visualizer closed: {e}")

print("Done.")