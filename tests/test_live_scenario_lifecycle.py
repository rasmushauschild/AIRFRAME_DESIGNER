import asyncio,threading
from types import SimpleNamespace as NS
from unittest.mock import Mock
from airframe_designer.server.app import build_app
from airframe_designer.sim.scenario import load_scenario

def setup():
    sim=NS(lock=threading.RLock(),hooks=[],t=10.,paused=False,airframe=NS(name='ATLAS_BWB_CONCEPT_05',mass=NS(cg=[0,0,0])),reset=Mock())
    state=NS(simulator=sim,conn=NS(mode='sitl',reset_all=lambda:{'steps':[]}),link=NS(status=lambda:{'armed':False}),log=Mock())
    routes={r.path:r.endpoint for r in build_app(state).routes if hasattr(r,'endpoint')}
    return sim,routes

def test_reset_finishes_runner_and_removes_hook():
    sim,r=setup()
    assert asyncio.run(r['/api/sim/scenario/start']({'phases':[{'type':'wait','duration':100}]}))['ok']
    assert len(sim.hooks)==1
    asyncio.run(r['/api/px4/reset_all']())
    assert sim.hooks==[]
    assert asyncio.run(r['/api/sim/scenario/start']({'phases':[{'type':'wait','duration':100}]}))['ok']

def test_wrong_model_rejected_before_start():
    sim,r=setup()
    response=asyncio.run(r['/api/sim/scenario/start']({'airframe':'ATLAS_MVP_08B','phases':[{'type':'wait'}]}))
    assert response.status_code==400 and not sim.hooks

def test_no_scenario_asks_the_simulator_to_fly_the_ground():
    """Every bundled scenario hands the ground to the firmware, because the aircraft has no other help."""
    import json, pathlib as _p
    for f in sorted(_p.Path('scenarios').glob('*.json')):
        types={p.get('type') for p in (json.loads(f.read_text()).get('phases') or [])}
        assert not types & {'nose_lift','nose_land'}, f'{f.name} still asks the simulator to lift or land it'


def test_animation_requires_recorded_run():
    sim,r=setup()
    response=asyncio.run(r['/api/sim/scenario/animation']())
    assert response.status_code==404


def test_live_animation_download_uses_recorded_pose():
    import io,zipfile
    sim,r=setup()
    asyncio.run(r['/api/sim/scenario/start']({'phases':[{'type':'wait','duration':100}]}))
    hook=sim.hooks[0]
    runner=next(c.cell_contents for c in hook.__closure__ if hasattr(c.cell_contents,'metrics'))
    runner.metrics.motion=[[10,0,0,-1,1,0,0,0]]
    response=asyncio.run(r['/api/sim/scenario/animation']())
    assert response.status_code==200
    with zipfile.ZipFile(io.BytesIO(response.body)) as z:
        assert 'motion.csv' in z.namelist()
