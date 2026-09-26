"""Verified parameter preparation for visible SITL scenarios (never persists to the model)."""
import math
from starlette.concurrency import run_in_threadpool

async def prepare_scenario_params(state, params):
    link = state.link  # Pin the SITL connection; never follow a reconnect to hardware.
    def check():
        if state.conn.mode != "sitl" or state.link is not link or link.status().get("armed"):
            raise ValueError("Scenario preparation cancelled: connection changed or vehicle armed")
        if not link.ctl_connected or state.simulator.paused:
            raise ValueError("Connect and resume SITL before starting the scenario")
    check()
    for name, value in params.items():
        if not isinstance(name, str) or not name or len(name)>16 or type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"Invalid scenario parameter: {name}")
    for name, value in params.items():
        check()
        result = await run_in_threadpool(link.set_param, name, value)
        check()
        if not result.get("ok"):
            raise ValueError(f"Scenario not started: {name}: {result.get('error', 'verification failed')}. Earlier settings may already be applied to SITL.")
        state.log(f"[live test] verified {name} = {result['value']}")
