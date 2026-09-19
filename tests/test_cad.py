"""STEP import: solids -> volume, centroid, inertia, mass items, JSON round trip and parameter paths."""
import math
import json
import numpy as np
import pytest

pytest.importorskip("OCP")

from airframe_designer.geometry import cad as cadmod
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry.paths import apply_variables, list_paths


@pytest.fixture(scope="module")
def step_file(tmp_path_factory):
    """Two named solids in millimetres: a 1 x 0.2 x 0.1 m box at the origin and a r=60 mm, h=200 mm cylinder at (300, 0, 100)."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDataStd import TDataStd_Name
    from OCP.Interface import Interface_Static
    doc = TDocStd_Document(TCollection_ExtendedString("t"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    box = BRepPrimAPI_MakeBox(gp_Pnt(-500, -100, -50), gp_Pnt(500, 100, 50)).Shape()
    cyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(300, 0, 100), gp_Dir(0, 0, 1)), 60, 200).Shape()
    for shape, name in ((box, "Fuselage"), (cyl, "Battery")):
        TDataStd_Name.Set_s(st.AddShape(shape, False), TCollection_ExtendedString(name))
    w = STEPCAFControl_Writer(); w.SetNameMode(True)
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    w.Transfer(doc, STEPControl_AsIs)
    p = tmp_path_factory.mktemp("cad") / "two_bodies.step"
    w.Write(str(p))
    return p


def test_import_measures_solids(step_file):
    r = cadmod.import_step(step_file)
    names = [b["name"] for b in r["bodies"]]
    assert names == ["Fuselage", "Battery"]
    box, cyl = r["bodies"]
    assert box["volume"] == pytest.approx(0.02, rel=1e-6)                    # metres, whatever the file unit
    assert cyl["volume"] == pytest.approx(math.pi * 0.06 ** 2 * 0.2, rel=1e-4)
    assert np.allclose(cyl["centroid"], [0.3, 0.0, 0.2], atol=1e-6)
    assert box["inertia_unit"][0] == pytest.approx((0.2 ** 2 + 0.1 ** 2) / 12 * 0.02, rel=1e-6)   # about the centroid
    assert cyl["inertia_unit"][0] == pytest.approx((3 * 0.06 ** 2 + 0.2 ** 2) / 12 * cyl["volume"], rel=1e-4)
    assert len(box["indices"]) == 12 * 3 and len(cyl["indices"]) > 0
    assert cadmod.cache_path(step_file).exists()
    assert cadmod.import_step(step_file) == r                                    # cache hit


def test_bodies_drive_the_cg(step_file):
    r = cadmod.import_step(step_file)
    m = cadmod.model_from_import(r, "cad/two_bodies.step")
    m.bodies[0].mass, m.bodies[1].mass = 20.0, 5.0
    af = Airframe.load("airframes/quad_x.json")
    af.cad = m; af.mass.from_items = True; af.resolve_mass()
    # CAD z up -> FRD z down: the battery sits at (0.3, 0, -0.2)
    assert af.mass.mass == pytest.approx(25.0)
    assert np.allclose(af.mass.cg, [0.06, 0.0, -0.04], atol=1e-6)
    ixx_expected = 20 * (0.2 ** 2 + 0.1 ** 2) / 12 + 5 * (3 * 0.06 ** 2 + 0.2 ** 2) / 12 + 20 * 0.04 ** 2 + 5 * 0.16 ** 2
    assert af.mass.inertia[0] == pytest.approx(ixx_expected, rel=1e-3)
    # dragging a body and removing one changes the CG; a zero-mass body does not count
    m.bodies[1].offset = [0.2, 0, 0]
    af.resolve_mass(); assert af.mass.cg[0] == pytest.approx(0.1)
    m.bodies[1].removed = True
    af.resolve_mass(); assert af.mass.mass == pytest.approx(20.0) and af.mass.cg[0] == pytest.approx(0.0)
    # round trip through JSON keeps everything (and 'pos' is exported for the UI)
    d = json.loads(json.dumps(af.to_dict()))
    assert d["cad"]["bodies"][1]["pos"] == pytest.approx([0.5, 0.0, -0.2])
    af2 = Airframe.from_dict(d)
    assert [b.mass for b in af2.cad.bodies] == [20.0, 5.0] and af2.cad.bodies[1].removed
    assert af2.mass.cg == pytest.approx(af.mass.cg)


def test_axes_presets_and_paths(step_file):
    r = cadmod.import_step(step_file)
    m = cadmod.model_from_import(r, "cad/two_bodies.step")
    m.bodies[1].mass = 1.0
    m.axes = "x_aft_z_up"
    assert np.allclose(m.body_pos(m.bodies[1]), [-0.3, 0.0, -0.2])
    m.axes = "frd"; m.origin = [1, 0, 0]; m.scale = 2.0
    assert np.allclose(m.body_pos(m.bodies[1]), [1.6, 0.0, 0.4])
    af = Airframe.load("airframes/quad_x.json"); af.cad = m; af.mass.from_items = True; af.resolve_mass()
    assert "cad.bodies[1].mass" in list_paths(af)
    af2 = apply_variables(af, {"cad.bodies[Battery].mass": 3.0, "cad.bodies[Fuselage].mass": 1.0, "cad.bodies[0].offset[2]": -0.5})
    assert af2.mass.mass == pytest.approx(4.0)
    assert af2.mass.cg[2] == pytest.approx((3 * 0.4 + 1 * (0 - 0.5)) / 4)
