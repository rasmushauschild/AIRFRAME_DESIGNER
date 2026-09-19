"""CAD segment: the solids of a STEP file as mass items.

A STEP file is imported once (``import_step``): every solid becomes a ``CadBody`` with its volume, centroid and
unit-density inertia (from OpenCascade), plus a triangle mesh cached beside the file for the 3D view. The airframe
then stores only what the user decides - a mass per body, a drag offset, removed flags, how the CAD axes map onto
the structural frame - and ``mass_items()`` turns that into point masses (with the solid's own inertia) for
``MassProperties.resolve``.

Frames: CAD coordinates -> scale -> rotation (degrees about the structural x, then y, then z axes) -> origin offset
-> structural FRD (x forward, y right, z down), metres. A Y-up CAD export, for instance, needs rotation x = -90.
"""
from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

from .mass import MassItem

# Legacy axes presets (schema files written before rotation_deg existed): folded into rotation_deg on load.
# Rows are FRD x, y, z expressed in CAD x, y, z.
AXES_PRESETS: dict[str, tuple[str, list[list[float]]]] = {
    "x_fwd_z_up": ("X forward, Y left, Z up (right-handed CAD)", [[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
    "x_aft_z_up": ("X aft, Y right, Z up", [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    "y_fwd_z_up": ("Y forward, X right, Z up", [[0, 1, 0], [1, 0, 0], [0, 0, -1]]),
    "y_aft_z_up": ("Y aft, X left, Z up", [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]),
    "x_fwd_y_up": ("X forward, Y up, Z right", [[1, 0, 0], [0, 0, 1], [0, -1, 0]]),
    "x_aft_y_up": ("X aft, Y up, Z left", [[-1, 0, 0], [0, 0, -1], [0, -1, 0]]),
    "frd": ("X forward, Y right, Z down (already FRD)", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
}


@dataclass
class CadBody:
    id: str = ""                    # stable key: "<index>:<name>" in file order
    name: str = "body"
    mass: float = 0.0               # kg, set by the user
    offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # drag offset, FRD metres
    removed: bool = False
    # measured in the CAD frame (native axes, metres): filled by import_step, kept in the airframe file
    volume: float = 0.0             # m^3
    centroid: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    inertia_unit: list[float] = field(default_factory=lambda: [0.0] * 6)   # Ixx Iyy Izz Ixy Ixz Iyz about the centroid, density 1 kg/m^3

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "CadBody":
        b = cls(**{k: v for k, v in (d or {}).items() if k in cls.__dataclass_fields__})
        b.offset = [float(v) for v in (b.offset or [0, 0, 0])][:3] + [0.0] * (3 - len(b.offset or []))
        b.centroid = [float(v) for v in (b.centroid or [0, 0, 0])]
        b.inertia_unit = [float(v) for v in (b.inertia_unit or [0] * 6)] + [0.0] * (6 - len(b.inertia_unit or []))
        return b


@dataclass
class CadModel:
    file: str = ""                  # path relative to the project (airframes/cad/<name>.step)
    axes: str = "frd"               # legacy preset (see AXES_PRESETS); new files use rotation_deg only
    rotation_deg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # about structural x, y, z (applied in that order)
    origin: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # FRD position of the CAD origin, m
    scale: float = 1.0              # extra factor on top of the STEP unit conversion (1 = file units are right)
    visible: bool = True
    bodies: list[CadBody] = field(default_factory=list)

    # ------------------------------------------------------------ frames
    def matrix(self) -> np.ndarray:
        """CAD -> FRD rotation: Rz(rz) @ Ry(ry) @ Rx(rx) on top of the (legacy) axes preset."""
        return rotation_matrix(self.rotation_deg) @ np.asarray(AXES_PRESETS.get(self.axes, AXES_PRESETS["frd"])[1], float)

    def to_frd(self, p) -> np.ndarray:
        """CAD point (native axes, metres) -> structural FRD."""
        return self.matrix() @ (np.asarray(p, float) * self.scale) + np.asarray(self.origin, float)

    def body_pos(self, b: CadBody) -> list[float]:
        """Where the body's centroid sits in the structural frame, drag offset included."""
        return [float(v) for v in self.to_frd(b.centroid) + np.asarray(b.offset, float)]

    def active(self) -> list[CadBody]:
        return [b for b in self.bodies if not b.removed]

    # ------------------------------------------------------------ mass
    def mass_items(self) -> list[MassItem]:
        """Point masses for MassProperties.resolve: each body's mass at its (dragged) centroid with the solid's own
        inertia scaled to that mass. A body with zero mass contributes nothing."""
        R = self.matrix()
        out = []
        for b in self.active():
            if b.mass <= 0 or b.volume <= 0:
                continue
            ixx, iyy, izz, ixy, ixz, iyz = b.inertia_unit
            I = np.array([[ixx, -ixy, -ixz], [-ixy, iyy, -iyz], [-ixz, -iyz, izz]]) * (b.mass / b.volume)
            I = (R @ I @ R.T) * (self.scale ** 2)     # rotate into FRD; inertia grows with length^2 under a uniform scale
            out.append(MassItem(name=f"cad:{b.name}", mass=float(b.mass), pos=self.body_pos(b),
                                inertia=[float(I[0, 0]), float(I[1, 1]), float(I[2, 2])],
                                inertia_products=[float(-I[0, 1]), float(-I[0, 2]), float(-I[1, 2])]))
        return out

    def totals(self) -> dict:
        """Mass and CG of the bodies alone (what the CAD section shows)."""
        items = self.mass_items()
        m = sum(i.mass for i in items)
        if m <= 0:
            return {"mass": 0.0, "cg": None, "bodies": len(self.active())}
        cg = sum(np.asarray(i.pos, float) * i.mass for i in items) / m
        return {"mass": float(m), "cg": [float(v) for v in cg], "bodies": len(self.active())}

    # ------------------------------------------------------------ io
    def to_dict(self) -> dict:
        d = {"file": self.file, "rotation_deg": list(self.rotation_deg), "origin": list(self.origin), "scale": self.scale, "visible": self.visible,
             "bodies": [b.to_dict() | {"pos": self.body_pos(b)} for b in self.bodies]}
        return d

    @classmethod
    def from_dict(cls, d: dict | None) -> "CadModel | None":
        if not d or not d.get("file"):
            return None
        m = cls(file=str(d.get("file", "")), axes=str(d.get("axes", "frd") or "frd"),
                rotation_deg=[float(v) for v in (d.get("rotation_deg") or [0, 0, 0])],
                origin=[float(v) for v in (d.get("origin") or [0, 0, 0])], scale=float(d.get("scale", 1.0) or 1.0),
                visible=bool(d.get("visible", True)),
                bodies=[CadBody.from_dict(b) for b in d.get("bodies", []) or []])
        if m.axes not in AXES_PRESETS:
            m.axes = "frd"
        if m.axes != "frd":      # legacy preset: fold it into the angles so the UI shows what is applied
            m.rotation_deg = euler_deg(m.matrix())
            m.axes = "frd"
        return m


def rotation_matrix(deg) -> np.ndarray:
    """Rz(rz) @ Ry(ry) @ Rx(rx): rotate about x, then y, then z (fixed structural axes), angles in degrees."""
    rx, ry, rz = np.radians(np.asarray(deg, float))
    cx, sx, cy, sy, cz, sz = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry), np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def euler_deg(R) -> list[float]:
    """Inverse of rotation_matrix (x, y, z angles in degrees)."""
    R = np.asarray(R, float)
    sy = -R[2, 0]
    if abs(sy) < 1 - 1e-9:
        ry = np.arcsin(sy); rx = np.arctan2(R[2, 1], R[2, 2]); rz = np.arctan2(R[1, 0], R[0, 0])
    else:   # gimbal lock: put everything into x and z
        ry = np.pi / 2 * np.sign(sy); rx = np.arctan2(-R[1, 2], R[1, 1]); rz = 0.0
    return [round(float(np.degrees(v)), 4) + 0.0 for v in (rx, ry, rz)]


# ================================================================== STEP import (OpenCascade via OCP)
def _safe_stem(name: str) -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", Path(name).stem).strip("_") or "model"
    return stem[:80]


def cache_path(step_path: Path) -> Path:
    return step_path.with_suffix(step_path.suffix + ".bodies.json")


def _ocp_names(shape_tool, label) -> str:
    from OCP.TDataStd import TDataStd_Name
    n = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), n):
        return str(n.Get().ToExtString())
    return ""


def _mesh_shape(shape, deflection: float):
    """Triangulate a shape; returns (vertices Nx3, triangles Mx3) in the shape's own frame."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location

    BRepMesh_IncrementalMesh(shape, deflection, False, 0.35, True)
    verts, tris = [], []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    base = 0
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            n = tri.NbNodes()
            for i in range(1, n + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                t = tri.Triangle(i)
                a, b, c = t.Value(1), t.Value(2), t.Value(3)
                tris.append((base + a - 1, base + c - 1, base + b - 1) if rev else (base + a - 1, base + b - 1, base + c - 1))
            base += n
        ex.Next()
    return np.asarray(verts, np.float64).reshape(-1, 3), np.asarray(tris, np.int64).reshape(-1, 3)


def _mass_props(shape) -> tuple[float, np.ndarray, np.ndarray]:
    """Volume, centroid and the inertia tensor about the centroid for density 1."""
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    v = float(props.Mass())
    c = props.CentreOfMass()
    c = np.array([c.X(), c.Y(), c.Z()])
    M = props.MatrixOfInertia()   # about the centre of mass (checked against an off-origin box)
    Ic = np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return v, c, Ic


def _collect_solids(step_path: Path) -> list[tuple[str, object]]:
    """[(name, TopoDS_Shape solid)] in file order, names from the STEP product structure when present."""
    try:
        from OCP.Interface import Interface_Static
    except ImportError as e:
        raise RuntimeError("STEP import needs OpenCascade: .venv/bin/pip install cadquery-ocp") from e
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopoDS import TopoDS
    from OCP.STEPControl import STEPControl_Reader
    STEPControl_Reader()   # constructing a reader registers the STEP statics; only then can the unit be set
    if not Interface_Static.SetCVal_s("xstep.cascade.unit", "M"):   # lengths in metres whatever the file's unit
        raise RuntimeError("cannot set the STEP length unit")

    solids: list[tuple[str, object]] = []
    try:
        from OCP.STEPCAFControl import STEPCAFControl_Reader
        from OCP.TDocStd import TDocStd_Document
        from OCP.TCollection import TCollection_ExtendedString
        from OCP.XCAFDoc import XCAFDoc_DocumentTool
        from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
        doc = TDocStd_Document(TCollection_ExtendedString("afd"))
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)
        Interface_Static.SetCVal_s("xstep.cascade.unit", "M")
        if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
            raise RuntimeError("STEP read failed")
        reader.Transfer(doc)
        st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

        def walk(label, loc, inherited_name):
            name = _ocp_names(st, label) or inherited_name
            if st.IsAssembly_s(label):
                comps = TDF_LabelSequence()
                st.GetComponents_s(label, comps)
                for k in range(1, comps.Length() + 1):
                    walk(comps.Value(k), loc, name)
                return
            if st.IsReference_s(label):
                from OCP.TDF import TDF_Label
                ref = TDF_Label()
                st.GetReferredShape_s(label, ref)
                sub_loc = loc.Multiplied(st.GetLocation_s(label))
                walk(ref, sub_loc, name)
                return
            shape = st.GetShape_s(label)
            if shape.IsNull():
                return
            shape = shape.Moved(loc)
            ex = TopExp_Explorer(shape, TopAbs_SOLID)
            k = 0
            while ex.More():
                k += 1
                solids.append((name or f"body {len(solids) + 1}", TopoDS.Solid(ex.Current())))
                ex.Next()

        from OCP.TopLoc import TopLoc_Location
        free = TDF_LabelSequence()
        st.GetFreeShapes(free)
        for i in range(1, free.Length() + 1):
            walk(free.Value(i), TopLoc_Location(), "")
    except Exception as e:   # no XCAF names (old OCP, odd file): fall back to anonymous solids
        import sys
        print(f"[cad] XCAF read failed ({type(e).__name__}: {e}); using plain STEP reader", file=sys.stderr)
        solids = []
    if not solids:
        reader = STEPControl_Reader()
        Interface_Static.SetCVal_s("xstep.cascade.unit", "M")
        if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
            raise RuntimeError(f"cannot read STEP file {step_path.name}")
        reader.TransferRoots()
        shape = reader.OneShape()
        ex = TopExp_Explorer(shape, TopAbs_SOLID)
        while ex.More():
            solids.append((f"body {len(solids) + 1}", TopoDS.Solid(ex.Current())))
            ex.Next()
    # de-duplicate names so ids stay unique and readable
    seen: dict[str, int] = {}
    out = []
    for name, s in solids:
        seen[name] = seen.get(name, 0) + 1
        out.append((name if seen[name] == 1 else f"{name} ({seen[name]})", s))
    return out


def import_step(step_path: str | Path, log=None) -> dict:
    """Parse a STEP file: per solid the mass properties and a mesh. Writes the cache beside the file and returns it.
    Cache format: {"file", "sha1", "bodies": [{"id","name","volume","centroid","inertia_unit","vertices","indices"}]}"""
    step_path = Path(step_path)
    data = step_path.read_bytes()
    sha = hashlib.sha1(data).hexdigest()
    cp = cache_path(step_path)
    if cp.exists():
        try:
            cached = json.loads(cp.read_text())
            if cached.get("sha1") == sha:
                return cached
        except Exception:
            pass
    solids = _collect_solids(step_path)
    if not solids:
        raise RuntimeError(f"{step_path.name} contains no solids (surfaces only?)")
    # a common tessellation tolerance: 0.15 % of the whole model's diagonal
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    for _, s in solids:
        BRepBndLib.Add_s(s, box, False)
    cmin, cmax = box.CornerMin(), box.CornerMax()
    xmin, ymin, zmin, xmax, ymax, zmax = cmin.X(), cmin.Y(), cmin.Z(), cmax.X(), cmax.Y(), cmax.Z()
    diag = float(np.hypot(np.hypot(xmax - xmin, ymax - ymin), zmax - zmin)) or 1.0
    deflection = max(2e-4, 0.0015 * diag)
    bodies = []
    for k, (name, s) in enumerate(solids):
        v, c, Ic = _mass_props(s)
        verts, tris = _mesh_shape(s, deflection)
        bodies.append({
            "id": f"{k}:{name}", "name": name, "volume": v, "centroid": [float(x) for x in c],
            "inertia_unit": [float(Ic[0, 0]), float(Ic[1, 1]), float(Ic[2, 2]), float(-Ic[0, 1]), float(-Ic[0, 2]), float(-Ic[1, 2])],
            "vertices": [round(float(x), 6) for x in verts.ravel()], "indices": [int(x) for x in tris.ravel()],
        })
        if log:
            log(f"[cad] {name}: {v * 1e3:.3f} L, centroid {np.round(c, 4).tolist()}, {len(tris)} triangles")
    out = {"file": step_path.name, "sha1": sha, "bounds": [xmin, ymin, zmin, xmax, ymax, zmax], "bodies": bodies}
    cp.write_text(json.dumps(out))
    return out


def model_from_import(imported: dict, file_rel: str, previous: CadModel | None = None) -> CadModel:
    """A CadModel for a freshly imported file, keeping masses/offsets/removed flags of bodies with the same id."""
    prev = {b.id: b for b in (previous.bodies if previous else [])}
    bodies = []
    for b in imported["bodies"]:
        old = prev.get(b["id"])
        bodies.append(CadBody(id=b["id"], name=b["name"], volume=float(b["volume"]), centroid=list(b["centroid"]),
                              inertia_unit=list(b["inertia_unit"]),
                              mass=old.mass if old else 0.0, offset=list(old.offset) if old else [0.0, 0.0, 0.0],
                              removed=old.removed if old else False))
    m = CadModel(file=file_rel, bodies=bodies)
    if previous:
        m.axes, m.rotation_deg, m.origin, m.scale, m.visible = previous.axes, list(previous.rotation_deg), list(previous.origin), previous.scale, previous.visible
    return m


def mesh_payload(model: CadModel, imported: dict) -> dict:
    """Meshes and centroids in the structural frame (offsets NOT applied: the client positions each body)."""
    R = model.matrix() * model.scale
    o = np.asarray(model.origin, float)
    by_id = {b["id"]: b for b in imported["bodies"]}
    out = []
    for b in model.bodies:
        src = by_id.get(b.id)
        if not src:
            continue
        v = np.asarray(src["vertices"], float).reshape(-1, 3) @ R.T + o
        out.append({"id": b.id, "name": b.name, "centroid": [float(x) for x in model.to_frd(b.centroid)],
                    "vertices": [round(float(x), 5) for x in v.ravel()], "indices": src["indices"]})
    return {"ok": True, "file": model.file, "bodies": out}
