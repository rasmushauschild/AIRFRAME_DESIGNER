"""Standard interactive simulator with native PX4 ground sequences."""
import time
import threading
from .geometry.landing import apply_landed_pitch


def landed_pitch_if_possible(airframe, log=None) -> None:
    """Work out the resting pitch, and shrug if the legs cannot say.

    A leg being edited passes through shapes that have no resting pitch at all - two feet, three in a line -
    and an airframe you cannot save while you move a leg is worse than a stale angle. The last good value
    stays, and the reason is logged."""
    try:
        apply_landed_pitch(airframe)
    except ValueError as e:
        if log:
            log(f"[geometry] landed pitch unchanged ({airframe.landed_pitch_deg:g} deg): {e}")
from .sim.simulator import Simulator
from .px4.connection import ConnectionManager
from .px4.sitl import launch_px4
from .batch.worker import BATCH_PX4_DEFAULTS, px4_param_types

class NativeSimulator(Simulator):
    def __init__(self,*args,**kwargs):
        frame = args[0] if args else kwargs['airframe']
        landed_pitch_if_possible(frame)
        kwargs.update(physics_substeps=4,seed=1)
        super().__init__(*args,**kwargs)
    def set_airframe(self, airframe):
        before = self.airframe.px4_params_sitl()
        landed_pitch_if_possible(airframe, self.log)
        result = super().set_airframe(airframe)
        if before != self.airframe.px4_params_sitl():
            callback = getattr(self, "on_model_parameters_changed", None)
            if callback: callback()
        return result

class NativeConnectionManager(ConnectionManager):
    def __init__(self, simulator, args, log):
        super().__init__(simulator, args, log)
        self._model_timer = None
        self._model_revision = 0
        simulator.on_model_parameters_changed = self.queue_model_sync

    def queue_model_sync(self):
        if self.mode != "sitl" or not self.args.launch_px4:
            return
        with self._lock:
            self._model_revision += 1
            revision = self._model_revision
            if self._model_timer: self._model_timer.cancel()
            self._reset_busy_until = time.time() + 60
            self._model_timer = threading.Timer(1.0, self._sync_model, args=(revision,))
            self._model_timer.daemon = True
            self._model_timer.start()

    def _sync_model(self, revision):
        with self._lock:
            if revision != self._model_revision: return
            if self.link and self.link.armed:
                self._model_timer = threading.Timer(1.0, self._sync_model, args=(revision,))
                self._model_timer.daemon = True
                self._model_timer.start()
                return
            try:
                self.log("[PX4] Applying model changes automatically and resetting the simulator")
                self.connect_sitl(True)
                self._reset_busy_until = time.time() + 8
                self.log("[PX4] Current model parameters loaded; waiting for preflight checks")
            except Exception as exc:
                self.error = str(exc)
                self.log(f"[PX4] Automatic model update failed: {exc}")
                self._reset_busy_until = time.time() + 60

    def _launch_sitl(self, instance):
        current = self.sim.airframe
        landed_pitch_if_possible(current, self.log)
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
                    # whether the loop is running is the user's: the app opens stopped and Start is the
                    # only thing that starts it, so relaunching PX4 under it must not start it instead
                return super().connect_sitl(True)
            finally:
                if was_running:
                    self.sim.start()

    def reset_all(self):
        """Reset is a stop button. The vehicle is on the ground before this call returns, whatever else was
        going on - a sequence, a nose lift, an armed and flying aircraft, even another reset a second ago.

        Behind that, PX4 is relaunched: the simulation clock goes back to zero and no flight controller can
        follow time running backwards. That takes a few seconds, so it does not happen in the click. A second
        press resets the vehicle again at once and its relaunch replaces the one still in flight."""
        if self.mode != 'sitl' or not self.args.launch_px4:
            return super().reset_all()
        from .px4.link import mavlink
        self._reset_gen += 1
        gen = self._reset_gen
        link = self.link
        if link is not None and link.ctl_connected and link.armed:
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        self.sim.motor_override = None
        self.sim.reset()
        if self._model_timer:                 # the relaunch loads the current model anyway
            self._model_revision += 1
            self._model_timer.cancel()
            self._model_timer = None
        self._reset_busy_until = time.time() + 25
        threading.Thread(target=self._relaunch_after_reset, args=(gen,), daemon=True).start()
        return {"ok": True, "steps": ["vehicle reset", "PX4 SITL relaunching"]}

    def _relaunch_after_reset(self, gen):
        """The slow half of a reset, off the request thread; it gives up as soon as a newer reset exists."""
        if self._reset_gen != gen:
            return
        try:
            self.connect_sitl(True)
            if self._reset_gen == gen:
                self.log("[PX4] reset: SITL relaunched")
                self._reset_busy_until = time.time() + 1
        except Exception as exc:
            if self._reset_gen == gen:
                self.error = str(exc)
                self._reset_busy_until = 0
                self.log(f"[PX4] reset: could not relaunch SITL: {exc}")
