"""Run the ATLAS_07D native PX4 demo on port 8081, SITL instance 1 only."""
from pathlib import Path
import sys
import time
BASE=Path(__file__).resolve().parent
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.batch.worker import BATCH_PX4_DEFAULTS, px4_param_types
import airframe_designer.px4.connection as connection
import airframe_designer.server.app as server
import airframe_designer.app as application
from airframe_designer.sim.simulator import Simulator
from landing_geometry import apply_landed_pitch
MODEL=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else BASE/'atlas_07d.native.json'
LIVE_SIM=None
af=Airframe.load(str(MODEL))
launch=connection.launch_px4
def seeded(px4_dir,model,log,**kw):
    current=LIVE_SIM.airframe if LIVE_SIM is not None else Airframe.load(str(MODEL))
    apply_landed_pitch(current)
    kw.update(params={**BATCH_PX4_DEFAULTS,**current.px4_params_sitl(), 'NLF_LAND_ANG':current.landed_pitch_deg},param_types=px4_param_types(px4_dir),fresh=True)
    return launch(px4_dir,model,log,**kw)
connection.launch_px4=seeded
server.UI_DIR=BASE/'ui'
class DemoSimulator(Simulator):
    def __init__(self,*args,**kwargs):
        frame = args[0] if args else kwargs['airframe']
        apply_landed_pitch(frame)
        kwargs.update(physics='jsbsim',physics_substeps=4,seed=1)
        super().__init__(*args,**kwargs)
        global LIVE_SIM
        LIVE_SIM=self
    def set_airframe(self, airframe, keep_state=True):
        apply_landed_pitch(airframe)
        return super().set_airframe(airframe, keep_state=keep_state)

class DemoConnectionManager(connection.ConnectionManager):
    def connect_sitl(self, launch=None):
        with self._lock:
            will_launch = self.args.launch_px4 if launch is None else launch
            if not will_launch:
                return super().connect_sitl(launch)
            # Quiesce the loop before replacing its link or resetting its clock.
            was_running = self.sim.running
            self.sim.stop()
            thread = self.sim._thread
            if thread is not None:
                thread.join(timeout=5)
                if thread.is_alive():
                    raise RuntimeError('Simulation loop did not stop; reconnect cancelled')
            try:
                self._close_link()
                self.stop_px4()
                self.sim.reset()
                with self.sim.lock:
                    self.sim.time_usec = 0
                    self.sim.step_count = 0
                    self.sim._last_hb = 0
                    self.sim._wall_start = time.perf_counter()
                    self.sim._sim_start = 0
                    self.sim._rtf_t0 = time.perf_counter()
                    self.sim._rtf_sim0 = 0
                    self.sim.nose_lift_last = None
                    self.sim.paused = False
                return super().connect_sitl(True)
            finally:
                if was_running:
                    self.sim.start()

    def reset_all(self):
        if self.mode != 'sitl' or not self.args.launch_px4:
            return super().reset_all()
        with self._lock:
            if time.time() < self._reset_busy_until:
                return {"ok": True, "steps": ["reset already in progress"]}
            self._reset_busy_until = time.time() + 8
            try:
                self.connect_sitl(True)
                return {"ok": True, "steps": ["simulation clock and vehicle reset", "owned PX4 SITL relaunched"]}
            except Exception:
                self._reset_busy_until = 0
                raise

application.ConnectionManager=DemoConnectionManager
application.Simulator=DemoSimulator
application.main(['--no-browser','--http','127.0.0.1:8081','--mode','sitl',
    '--px4-dir',str(BASE),'--px4-instance','1','--px4-rootfs',str(BASE/'live_px4'),
    '--airframe',str(MODEL)])
