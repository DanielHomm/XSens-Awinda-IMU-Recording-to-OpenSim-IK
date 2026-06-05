import os
import sys
from pathlib import Path
import opensim as osim

GEOMETRY_DIR = Path(
    "/Users/arbeit/Applications/OpenSim 4.5/"
    "OpenSim 4.5.app/Contents/Resources/opensim/Geometry"
)

REPO_DIR = Path(__file__).resolve().parent
MODEL_PATH = REPO_DIR / "calibrated_model.osim"
MOTION_PATH = REPO_DIR / "walking.mot"

def ensure_simbody_visualizer_on_path():
    prefix = Path(sys.executable).resolve().parent.parent
    candidates = [
        prefix / "simbody" / "libexec" / "simbody",
        prefix / "libexec" / "simbody",
    ]
    for candidate in candidates:
        if (candidate / "simbody-visualizer.app").exists():
            old = os.environ.get("PATH", "")
            if str(candidate) not in old.split(os.pathsep):
                os.environ["PATH"] = str(candidate) + os.pathsep + old
            return
    raise RuntimeError("simbody-visualizer.app not found.")

def main():
    ensure_simbody_visualizer_on_path()

    model_dir = MODEL_PATH.resolve().parent
    local_geom = model_dir / "Geometry"

    if GEOMETRY_DIR.exists():
        osim.ModelVisualizer.addDirToGeometrySearchPaths(str(GEOMETRY_DIR))
    if local_geom.exists():
        osim.ModelVisualizer.addDirToGeometrySearchPaths(str(local_geom))

    osim.Logger.setLevelString("Error")

    model = osim.Model(str(MODEL_PATH))
    motion = osim.TimeSeriesTable(str(MOTION_PATH))

    osim.VisualizerUtilities.showMotion(model, motion)

if __name__ == "__main__":
    main()