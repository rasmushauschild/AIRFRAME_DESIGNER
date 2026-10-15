import asyncio,threading
from types import SimpleNamespace as NS
from unittest.mock import Mock
from airframe_designer.server.app import build_app
from airframe_designer.sim.scenario import load_scenario

def setup():
    sim=NS(lock=threading.RLock(),hooks=[],t=10.,paused=False,airframe=NS(name='ATLAS_BWB_CONCEPT_05',mass=NS(cg=[0,0,0])),stop_nose_lift=Mock(),reset=Mock())
    state=NS(simulator=sim,conn=NS(mode='sitl',reset_all=lambda:{'steps':[]}),link=NS(status=lambda:{'armed':False}),log=Mock())
    routes={r.path:r.endpoint for r in build_app(state).routes if hasattr(r,'endpoint')}
    return sim,routes

def test_reset_finishes_runner_and_removes_hook():
    sim,r=setup()
    assert asyncio.run(r['/api/sim/scenario/start']({'phases':[{'type':'wait','duration':100}]}))['ok']
    assert len(sim.hooks)==1
    asyncio.run(r['/api/px4/reset_all']())
    assert sim.hooks==[]
    sim.stop_nose_lift.assert_called_once()
    assert asyncio.run(r['/api/sim/scenario/start']({'phases':[{'type':'wait','duration':100}]}))['ok']

def test_wrong_model_rejected_before_start():
    sim,r=setup()
    response=asyncio.run(r['/api/sim/scenario/start']({'airframe':'ATLAS_MVP_08B','phases':[{'type':'wait'}]}))
    assert response.status_code==400 and not sim.hooks

def test_concept_scenario_selects_all_four_nose_fans():
    sc=load_scenario('scenarios/atlas_bwb_concept_05_full_flight.json')
    assert sc.airframe=='ATLAS_BWB_CONCEPT_05'
    assert all(p['motors']==[4,5,6,7] for p in sc.phases if p['type'] in ('nose_lift','nose_land'))


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
