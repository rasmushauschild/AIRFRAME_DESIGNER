import csv
import io
import zipfile
import numpy as np
import pytest
from airframe_designer.sim.animation import blender_rows, animation_zip
from airframe_designer.dynamics.quaternion import q_from_euler, q_to_rotmat


def test_rotation_and_cg_reproduce_point_trajectory():
    cg = np.array([1.2, -.3, .4])
    point = np.array([3., 2., -.7])
    pos = np.array([5., 7., -2.])
    C = np.diag([1., -1., -1.])
    for angles in [(0, 0, 0), (0, .6, 0), (.4, -.3, 1.2)]:
        q = q_from_euler(*angles)
        row = blender_rows([[100., *pos, *q]], cg)[0]
        transformed = np.array(row[1:4]) + q_to_rotmat(row[4:]) @ C @ point
        expected = C @ (pos + q_to_rotmat(q) @ (point - cg))
        np.testing.assert_allclose(transformed, expected, atol=1e-12)
        assert row[0] == 0


def test_quaternion_sign_and_simulation_timing():
    q = q_from_euler(0, .5, 0)
    rows = blender_rows([[10, 0, 0, -1, *q], [10.02, 0, 0, -2, *(-q)]], [0, 0, 0])
    assert rows[1][0] == pytest.approx(.02)
    np.testing.assert_allclose(rows[0][4:], rows[1][4:])
    assert rows[1][3] == 2


def test_archive_and_embedded_script():
    with zipfile.ZipFile(io.BytesIO(animation_zip([[1, 0, 0, -1, 1, 0, 0, 0]], [0, 0, 0], 'Test "aircraft"'))) as z:
        assert set(z.namelist()) == {'motion.csv', 'import_animation.py', 'README.txt'}
        rows = list(csv.reader(io.StringIO(z.read('motion.csv').decode())))
        assert len(rows) == 2
        assert rows[0] == ['time_s', 'x_m', 'y_m', 'z_m', 'qw', 'qx', 'qy', 'qz']
        compile(z.read('import_animation.py'), 'import_animation.py', 'exec')


@pytest.mark.parametrize('rows', [[], [[0, 0, 0, 0, 0, 0, 0, 0]], [[float('nan'), 0, 0, 0, 1, 0, 0, 0]], [[1, 0, 0, 0, 1, 0, 0, 0]] * 2])
def test_invalid_motion_rejected(rows):
    with pytest.raises(ValueError):
        blender_rows(rows, [0, 0, 0])
