"""Standard interactive simulator with native PX4 ground sequences."""
import time
from .geometry.landing import apply_landed_pitch
from .sim.simulator import Simulator
from .px4.connection import ConnectionManager
from .px4.sitl import launch_px4
from .batch.worker import BATCH_PX4_DEFAULTS, px4_param_types

class NativeSimulator(Simulator):
    def __init__(self,*args,**kwargs):
        frame = args[0] if args else kwargs['airframe']
        apply_landed_pitch(frame)
        kwargs.update(physics='jsbsim',physics_substeps=4,seed=1)
        super().__init__(*args,**kwargs)
    def set_airframe(self, airframe, keep_state=True):
        apply_landed_pitch(airframe)
        return super().set_airframe(airframe, keep_state=keep_state)

class NativeConnectionManager(ConnectionManager):
    def _launch_sitl(self, instance):
        current = self.sim.airframe
        apply_landed_pitch(current)
        return launch_px4(self.args.px4_dir, self.args.px4_model, self.log,
            instance=instance, rootfs=self.args.px4_rootfs,
            params={**BATCH_PX4_DEFAULTS, **current.px4_params_sitl(), 'NLF_LAND_ANG': current.landed_pitch_deg},
            param_types=px4_param_types(self.args.px4_dir), fresh=True)

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
