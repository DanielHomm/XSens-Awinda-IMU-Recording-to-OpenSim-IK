"""
5_visualize.py — play back the IK motion in the native simbody-visualizer
(no OpenSim GUI required).

Unlike VisualizerUtilities.showMotion(), this drives the simbody-visualizer
directly so the camera can be zoomed to fit the whole model — otherwise the
default framing clips the feet and the upper body.

Usage
-----
    conda activate opensim
    python 5_visualize.py [MOTION.mot] [--model MODEL.osim] [--speed 1.0]

    python 5_visualize.py                 # walking.mot + calibrated_model.osim
    python 5_visualize.py lunge.mot
    python 5_visualize.py walking.mot --speed 0.5    # half speed

The motion loops. Press Ctrl-C in the terminal (or close the window) to exit.
You can rotate/zoom/pan freely in the window with the mouse.
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import opensim as osim

# ---------------------------------------------------------------------------
# Constants — change GEOMETRY_DIR if your OpenSim installation differs
# ---------------------------------------------------------------------------

GEOMETRY_DIR = Path(
    "/Users/arbeit/Applications/OpenSim 4.5/"
    "OpenSim 4.5.app/Contents/Resources/opensim/Geometry"
)

REPO_DIR       = Path(__file__).resolve().parent
DEFAULT_MODEL  = REPO_DIR / "calibrated_model.osim"
DEFAULT_MOTION = REPO_DIR / "walking.mot"


# ---------------------------------------------------------------------------
# Locate the simbody-visualizer executable
# ---------------------------------------------------------------------------

def _ensure_simbody_visualizer_on_path() -> None:
    """
    The conda opensim package installs simbody-visualizer.app but does NOT add
    its directory to PATH; Simbody's VisualizerProtocol searches PATH for the
    app bundle, so we prepend it here (current process only).

    IMPORTANT: there are usually two visualizers in the env and they must match
    the SimTK version the opensim Python module loaded, or the parent↔visualizer
    handshake fails with a bare std::exception:
      <prefix>/libexec/simbody/          → bundled with opensim   (often 3.7)
      <prefix>/simbody/libexec/simbody/  → standalone simbody pkg (often 3.8)
    The opensim Python package here ships libSimTKcommon.3.8, so the standalone
    simbody location is preferred.
    """
    prefix = Path(sys.executable).parent.parent   # e.g. .../envs/opensim
    candidates = [
        prefix / "simbody" / "libexec" / "simbody",   # standalone pkg (3.8) — preferred
        prefix / "libexec" / "simbody",               # opensim-bundled (3.7) — fallback
    ]
    for candidate in candidates:
        if (candidate / "simbody-visualizer.app").exists():
            old = os.environ.get("PATH", "")
            if str(candidate) not in old.split(os.pathsep):
                os.environ["PATH"] = str(candidate) + os.pathsep + old
            return


# ---------------------------------------------------------------------------
# Playback
# ---------------------------------------------------------------------------

def run(model_path: Path, motion_path: Path, speed: float = 1.0) -> None:
    _ensure_simbody_visualizer_on_path()
    osim.ModelVisualizer.addDirToGeometrySearchPaths(str(GEOMETRY_DIR))
    osim.Logger.setLevelString("Error")   # silence "coordinate X is locked" spam

    model = osim.Model(str(model_path))
    model.setUseVisualizer(True)
    state = model.initSystem()

    # --- load motion ---
    table      = osim.TimeSeriesTable(str(motion_path))
    in_degrees = table.getTableMetaDataAsString("inDegrees") == "yes"
    labels     = list(table.getColumnLabels())
    times      = list(table.getIndependentColumn())
    n_frames   = table.getNumRows()

    # Precompute (column_index, coordinate, is_rotational) for matching columns.
    coord_set = model.getCoordinateSet()
    plan = []
    for ci, lab in enumerate(labels):
        try:
            coord = coord_set.get(lab)
        except Exception:
            continue   # column with no matching coordinate
        rotational = coord.getMotionType() == osim.Coordinate.Rotational
        plan.append((ci, coord, rotational))

    def apply_frame(i: int) -> None:
        row = table.getRowAtIndex(i)
        for ci, coord, rotational in plan:
            v = row[ci]
            if in_degrees and rotational:
                v = np.deg2rad(v)
            try:
                coord.setValue(state, v, False)   # locked coords are skipped (warn)
            except Exception:
                pass
        model.realizePosition(state)

    # --- configure the simbody-visualizer ---
    viz = model.updVisualizer().updSimbodyVisualizer()
    viz.setShowSimTime(True)
    viz.setGroundHeight(-1.5)         # drop the ground plane so the feet aren't occluded
    viz.setWindowTitle(f"{motion_path.name} - {model_path.name}")

    # Draw the first frame, then frame the whole model so nothing is clipped.
    apply_frame(0)
    state.setTime(times[0])
    viz.report(state)
    viz.zoomCameraToShowAllGeometry()

    print(f"Playing {n_frames} frames at {speed}× speed. "
          f"Ctrl-C to stop. Mouse drags to rotate/zoom/pan.")

    # --- loop ---
    try:
        while True:
            for i in range(n_frames):
                tic = time.time()
                apply_frame(i)
                state.setTime(times[i])
                viz.report(state)
                if i + 1 < n_frames:
                    dt = (times[i + 1] - times[i]) / max(speed, 1e-6)
                    rem = dt - (time.time() - tic)
                    if rem > 0:
                        time.sleep(rem)
    except KeyboardInterrupt:
        print("\nStopped.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Play an IK motion in the simbody-visualizer.")
    parser.add_argument("motion", nargs="?", type=Path, default=DEFAULT_MOTION,
                        help=f"Path to .mot file (default: {DEFAULT_MOTION.name})")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL,
                        help=f"Path to .osim model (default: {DEFAULT_MODEL.name})")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (default: 1.0)")
    args = parser.parse_args()

    if not args.motion.exists():
        sys.exit(f"Error: motion file not found: {args.motion}")
    if not args.model.exists():
        sys.exit(f"Error: model file not found: {args.model}")

    print(f"Model  : {args.model}")
    print(f"Motion : {args.motion}")
    run(args.model, args.motion, speed=args.speed)


if __name__ == "__main__":
    main()
