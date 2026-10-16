"""Shared fixtures: project paths, preset airframes and the schema-1 reference airframe."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

TESTS_DIR = Path(__file__).resolve().parent
PROJECT_DIR = TESTS_DIR.parent
DATA_DIR = TESTS_DIR / "data"
AIRFRAMES_DIR = PROJECT_DIR / "airframes"
SCENARIOS_DIR = PROJECT_DIR / "scenarios"
SCHEMA1_FIXTURE = DATA_DIR / "atlas_07c_schema1.json"
PX4_BINARY = Path(os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))) / "build" / "px4_sitl_default" / "bin" / "px4"

# the package is not installed into the venv: make it importable from the project root
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: takes more than a few seconds")


def pytest_collection_modifyitems(config, items):
    if PX4_BINARY.is_file():
        return
    skip = pytest.mark.skip(reason=f"PX4 SITL build not found at {PX4_BINARY}")
    for item in items:
        if "px4" in item.keywords:
            item.add_marker(skip)


@pytest.fixture
def project_dir() -> Path:
    return PROJECT_DIR


@pytest.fixture
def venv_python() -> str:
    return sys.executable


@pytest.fixture
def quad():
    from airframe_designer.geometry.airframe import quad_x
    return quad_x()


@pytest.fixture
def plane():
    from airframe_designer.geometry.airframe import plane_quad
    return plane_quad()


@pytest.fixture
def schema1_dict() -> dict:
    with open(SCHEMA1_FIXTURE) as f:
        return json.load(f)


@pytest.fixture
def atlas08():
    from airframe_designer.geometry.airframe import Airframe
    return Airframe.load(DATA_DIR / "atlas_08.json")
