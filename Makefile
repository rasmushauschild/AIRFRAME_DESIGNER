PY := .venv/bin/python

.PHONY: setup ui test test-fast run-hover study firmware clean

setup:
	python3 -m venv .venv && .venv/bin/pip install -q -e ".[dev]"

ui:
	$(PY) -m airframe_designer ui

test-fast:
	$(PY) -m pytest -q -m "not px4"

test:
	$(PY) -m pytest -q

run-hover:
	$(PY) -m airframe_designer run --airframe airframes/atlas_08.json --scenario hover --out results/hover.json

study:
	$(PY) -m airframe_designer study --spec studies/atlas08_hover_tilt.json --workers 6

firmware:  ## build the PX4 SITL firmware with the ATLAS nose-lift module into firmware/atlas/build (source: $PX4_SOURCE_DIR or ~/PX4-Autopilot)
	cd firmware/atlas && cmake -S "$${PX4_SOURCE_DIR:-$$HOME/PX4-Autopilot}" -B build/px4_sitl_default -G Ninja \
	  -DCONFIG=px4_sitl_default -DEXTERNAL_MODULES_LOCATION="$$PWD" "-DPYTHON_EXECUTABLE=$$PWD/../../.venv/bin/python" \
	  && CCACHE_DIR=/tmp/atlas-nl-ccache cmake --build build/px4_sitl_default -j 6

clean:
	rm -rf results/*/ .pytest_cache; find . -name __pycache__ -type d -exec rm -rf {} +
