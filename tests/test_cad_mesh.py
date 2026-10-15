"""OBJ and STL import: a shape to look at, with no mass.

A mesh is not a solid. It can be triangulated, measured and drawn, but nothing in it says what it is made
of, so a model imported this way is marked `visual_only` and contributes no mass however the file is
edited afterwards. These check that, the two STL encodings, OBJ groups, and the unit guess — a mesh file
carries no units, and a 300-unit aircraft is one drawn in millimetres.
"""
from __future__ import annotations

import json
import struct

import numpy as np
import pytest

from airframe_designer.geometry.airframe import quad_x
from airframe_designer.geometry.cad import CadModel, import_any, import_mesh, model_from_import
from tests.test_cad import step_file                                   # noqa: F401  (the STEP fixture)

# a unit cube, 12 triangles, corner at the origin
CUBE_V = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
CUBE_F = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
          (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]


def write_obj(path, scale=1.0, groups=1):
    lines = []
    for g in range(groups):
        lines.append(f"o part{g + 1}")
        base = g * len(CUBE_V)
        for x, y, z in CUBE_V:
            lines.append(f"v {(x + 2 * g) * scale} {y * scale} {z * scale}")
        for a, b, c in CUBE_F:
            lines.append(f"f {base + a + 1} {base + b + 1} {base + c + 1}")
    path.write_text("\n".join(lines))
    return path


def write_stl(path, scale=1.0, binary=True):
    tris = [(np.array(CUBE_V[a]) * scale, np.array(CUBE_V[b]) * scale, np.array(CUBE_V[c]) * scale) for a, b, c in CUBE_F]
    if binary:
        out = bytearray(b"\0" * 80 + struct.pack("<I", len(tris)))
        for a, b, c in tris:
            out += struct.pack("<12fH", 0, 0, 0, *a, *b, *c, 0)
        path.write_bytes(bytes(out))
    else:
        s = ["solid cube"]
        for a, b, c in tris:
            s.append(" facet normal 0 0 0\n  outer loop")
            s += [f"   vertex {p[0]} {p[1]} {p[2]}" for p in (a, b, c)]
            s.append("  endloop\n endfacet")
        s.append("endsolid cube")
        path.write_text("\n".join(s))
    return path


@pytest.mark.parametrize("make", [lambda p: write_obj(p / "cube.obj"),
                                  lambda p: write_stl(p / "cube.stl", binary=True),
                                  lambda p: write_stl(p / "cube.stl", binary=False)])
def test_a_metre_cube_comes_back_as_a_cubic_metre(tmp_path, make):
    """Volume and centroid from the triangles themselves, so the table says something true."""
    d = import_any(make(tmp_path))
    assert d["visual_only"] is True
    assert len(d["bodies"]) == 1
    b = d["bodies"][0]
    assert b["volume"] == pytest.approx(1.0, rel=1e-6)                 # a 1 m cube
    assert b["centroid"] == pytest.approx([0.5, 0.5, 0.5], abs=1e-6)
    assert len(b["indices"]) == 36 and b["inertia_unit"] == [0.0] * 6   # triangles, no inertia


def test_a_mesh_wound_inside_out_still_has_a_positive_volume(tmp_path):
    """Exported meshes are often wound the other way. That is a normal direction, not a negative size."""
    lines = ["o flipped"] + [f"v {x} {y} {z}" for x, y, z in CUBE_V] + [f"f {a + 1} {c + 1} {b + 1}" for a, b, c in CUBE_F]
    p = tmp_path / "flipped.obj"
    p.write_text("\n".join(lines))
    b = import_mesh(p)["bodies"][0]
    assert b["volume"] == pytest.approx(1.0, rel=1e-6)
    assert b["centroid"] == pytest.approx([0.5, 0.5, 0.5], abs=1e-6)


def test_a_model_drawn_in_millimetres_is_read_as_millimetres(tmp_path):
    """300 units across is 300 mm, not 300 m: nobody builds a three-hundred-metre drone."""
    d = import_mesh(write_obj(tmp_path / "big.obj", scale=300.0))
    b = d["bodies"][0]
    assert b["volume"] == pytest.approx(0.027, rel=1e-6)               # 0.3 m cube
    assert b["centroid"] == pytest.approx([0.15, 0.15, 0.15], abs=1e-6)
    assert max(d["bounds"]) == pytest.approx(0.3, abs=1e-9)


def test_obj_groups_become_separate_bodies(tmp_path):
    d = import_mesh(write_obj(tmp_path / "two.obj", groups=2))
    assert [b["name"] for b in d["bodies"]] == ["part1", "part2"]
    assert all(len(b["indices"]) == 36 for b in d["bodies"])
    assert d["bodies"][1]["centroid"][0] == pytest.approx(2.5, abs=1e-6)   # the second cube is 2 along x


def test_the_import_is_cached_beside_the_file(tmp_path):
    p = write_obj(tmp_path / "cube.obj")
    first = import_mesh(p)
    cache = tmp_path / "cube.obj.bodies.json"
    assert cache.is_file() and json.loads(cache.read_text())["sha1"] == first["sha1"]
    assert import_mesh(p) == first                                     # second read comes from the cache


def test_a_visual_model_weighs_nothing_whatever_the_file_says(tmp_path):
    """The flag is the guard: even a hand-edited mass on a mesh body must not reach the aircraft."""
    d = import_mesh(write_obj(tmp_path / "cube.obj"))
    model = model_from_import(d, "airframes/cad/cube.obj")
    assert model.visual_only is True
    model.bodies[0].mass = 5.0                                         # as if typed into an older file
    assert model.mass_items() == []
    assert model.totals()["mass"] == 0.0

    af = quad_x()
    before = af.mass.mass
    af.cad = model
    af.mass.from_items = True
    af.resolve_mass()
    assert af.mass.mass == pytest.approx(before)        # the five kilograms on the mesh body reached nothing

    assert CadModel.from_dict(model.to_dict()).visual_only is True     # and it survives a save and load


def test_a_step_import_is_still_a_solid_with_mass(step_file):
    """The flag must not leak onto STEP models, which are the ones that can weigh something."""
    from airframe_designer.geometry.cad import import_step
    d = import_step(step_file)
    assert not d.get("visual_only")
    model = model_from_import(d, "x.step")
    assert model.visual_only is False
    model.bodies[0].mass = 2.0
    assert len(model.mass_items()) == 1


def test_an_empty_mesh_is_refused(tmp_path):
    p = tmp_path / "empty.obj"
    p.write_text("# nothing here\n")
    with pytest.raises(RuntimeError):
        import_mesh(p)


def test_a_step_surface_is_imported_as_a_body_with_no_volume(tmp_path):
    """A CAD export of an aircraft is mostly surfaces: skins, canopies, anything not modelled as a solid.

    They used to be dropped, so a file with two ribs and three skin surfaces imported as two ribs — a
    picture of an aircraft with most of the aircraft missing. They come in as bodies with no volume, which
    is what keeps them out of the mass.
    """
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt, gp_Pln, gp_Dir, gp_Ax3
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    from OCP.Interface import Interface_Static
    from airframe_designer.geometry.cad import import_step, model_from_import

    Interface_Static.SetCVal_s("write.step.unit", "M")       # metres in, metres out: the units are not the point here
    face = BRepBuilderAPI_MakeFace(gp_Pln(gp_Ax3(gp_Pnt(0, 0, 0.5), gp_Dir(0, 0, 1))),
                                   -0.5, 0.5, -0.25, 0.25).Face()
    box = BRepPrimAPI_MakeBox(gp_Pnt(-0.1, -0.1, -0.1), gp_Pnt(0.1, 0.1, 0.1)).Shape()
    w = STEPControl_Writer()
    for shape in (box, face):
        w.Transfer(shape, STEPControl_AsIs)
    p = tmp_path / "solid_and_surface.step"
    w.Write(str(p))

    d = import_step(p)
    solid = [b for b in d["bodies"] if b["volume"] > 0]
    surface = [b for b in d["bodies"] if b["volume"] == 0]
    assert len(solid) == 1 and len(surface) == 1, [b["name"] for b in d["bodies"]]
    assert solid[0]["volume"] == pytest.approx(0.008, rel=1e-6)        # a 200 mm cube
    assert len(surface[0]["indices"]) >= 6                              # it is drawn
    assert surface[0]["centroid"] == pytest.approx([0.0, 0.0, 0.5], abs=1e-6)
    assert np.allclose(surface[0]["inertia_unit"], 0.0)

    model = model_from_import(d, "x.step")
    model.bodies[0].mass = 3.0
    model.bodies[1].mass = 3.0                                          # a mass typed onto the surface
    assert len(model.mass_items()) == 1                                 # only the solid carries it
    assert model.totals()["mass"] == pytest.approx(3.0)
