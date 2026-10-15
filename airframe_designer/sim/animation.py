"""Export true CG pose to Blender's Z-up structural-origin animation."""
from __future__ import annotations
import csv
import io
import json
import zipfile
import numpy as np
from ..dynamics.quaternion import q_to_rotmat


def blender_rows(motion, cg):
    """NED world and FRD body -> X north/forward, Y west/left, Z up.

    Input: time, CG north/east/down, body-to-NED quaternion (wxyz).
    Output: elapsed time, structural reference position, quaternion (wxyz).
    """
    rows = []
    previous = None
    t0 = None
    cg = np.asarray(cg, float)
    for row in motion:
        a = np.asarray(row, float)
        if a.shape != (8,) or not np.isfinite(a).all() or np.linalg.norm(a[4:]) < 1e-12:
            raise ValueError("Invalid animation pose")
        if t0 is None:
            t0 = a[0]
        elapsed = float(a[0] - t0)
        if rows and elapsed <= rows[-1][0]:
            raise ValueError("Animation timestamps must increase")
        q = a[4:] / np.linalg.norm(a[4:])
        pos = (a[1:4] - q_to_rotmat(q) @ cg) * [1, -1, -1]
        qb = q * [1, 1, -1, -1]
        if previous is not None and np.dot(previous, qb) < 0:
            qb = -qb
        rows.append([elapsed, *pos.tolist(), *qb.tolist()])
        previous = qb
    if not rows:
        raise ValueError("No motion has been recorded")
    return rows


IMPORTER = '''# Run in Blender's Scripting workspace. Motion data is embedded.
import bpy
import json
import math
from mathutils import Matrix, Quaternion, Vector

DATA = json.loads(__DATA__)
FPS = 30  # Change before running, if desired.
START_FRAME = 1
scene = bpy.context.scene
scene.render.fps = FPS
scene.render.fps_base = 1.0
selected = list(bpy.context.selected_objects)
root = bpy.data.objects.new("Aircraft motion", None)
scene.collection.objects.link(root)
root.empty_display_type = 'ARROWS'
root.rotation_mode = 'QUATERNION'
# Model must already be in metres, +X nose, +Y left, +Z up,
# with its structural reference at world origin (see README).
for obj in selected:
    if obj.parent not in selected:
        world = obj.matrix_world.copy()
        obj.parent = root
        obj.matrix_parent_inverse = Matrix.Identity(4)
        obj.matrix_world = world
rows = DATA["rows"]
scene.frame_start = START_FRAME
scene.frame_end = START_FRAME + math.ceil(rows[-1][0] * FPS)
# Bake each output frame with linear position and shortest-path quaternion SLERP.
i = 0
for frame in range(scene.frame_start, scene.frame_end + 1):
    t = min((frame - START_FRAME) / FPS, rows[-1][0])
    while i + 1 < len(rows) and rows[i + 1][0] <= t:
        i += 1
    a = rows[i]
    b = rows[min(i + 1, len(rows) - 1)]
    factor = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    root.location = Vector(a[1:4]).lerp(Vector(b[1:4]), factor)
    root.rotation_quaternion = Quaternion(a[4:8]).slerp(Quaternion(b[4:8]), factor)
    root.keyframe_insert(data_path="location", frame=frame)
    root.keyframe_insert(data_path="rotation_quaternion", frame=frame)
scene.frame_set(START_FRAME)
bpy.context.view_layer.objects.active = root
root.select_set(True)
print("Imported aircraft motion:", DATA["aircraft"], len(rows), "samples")
'''


def animation_zip(motion, cg, name):
    rows = blender_rows(motion, cg)
    data = {"aircraft": name, "units": "metres, seconds", "rows": rows}
    script = IMPORTER.replace('__DATA__', repr(json.dumps(data, allow_nan=False)))
    csv_file = io.StringIO()
    writer = csv.writer(csv_file)
    writer.writerow(['time_s', 'x_m', 'y_m', 'z_m', 'qw', 'qx', 'qy', 'qz'])
    writer.writerows(rows)
    readme = '''BLENDER AIRCRAFT ANIMATION

1. Extract the ZIP. In Blender, arrange your aircraft model in metres,
   with the nose along +X, left wing along +Y and top along +Z.
   The design's structural reference (not its CG) must be at world (0,0,0).
   This is the unrotated design pose, not the tilted ground/hover pose.
2. Select the aircraft objects. In Scripting > Text Editor > Open,
   open import_animation.py and click Run Script. It creates an animated
   Empty and parents the selected model under it. Existing model animation,
   constraints or rigid-body simulation should be removed from a COPY first.
   With nothing selected, the Empty alone previews the trajectory.
3. Press Play. Default output is 30 fps; edit FPS before running to change it.
   The script embeds all motion, so no CSV path needs to be configured.

motion.csv is also provided for custom importers: elapsed simulation seconds,
position in metres, quaternion W/X/Y/Z. Both world and model coordinates are
Z-up: X north/forward, Y west/left, Z up. Position refers to the aircraft's
structural reference; the CG offset and true body orientation are included.
Recording uses the metrics sample rate (normally 50 Hz). Real simulation time
is retained even for accelerated batch runs. Rotation is quaternion-based;
nose pitch is included. The importer resamples to output frames using linear
position and quaternion SLERP. Ground is Z=0. Failed/stopped flights export
only their recorded segment. Mesh geometry and motor animation are not exported.
Old runs made before this feature do not contain the required body poses.
The importer creates a new Empty each time; undo an import before repeating it.
'''
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('motion.csv', csv_file.getvalue())
        archive.writestr('import_animation.py', script)
        archive.writestr('README.txt', readme)
    return buffer.getvalue()
