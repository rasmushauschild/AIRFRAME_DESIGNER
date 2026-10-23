# Re-running the takeoff-spin simulation

PX4 slots 0-9 were taken by other sessions, so these runs used slot 12 with a private copy of the PX4 SITL
startup scripts (the shared ~/PX4-Autopilot tree was not touched):

1. `px4priv/build/px4_sitl_default/` = symlinks to everything in `~/PX4-Autopilot/build/px4_sitl_default/` except `etc/`, which is a copy.
2. In that copy, `etc/init.d-posix/px4-rc.mavlink`: the line that sends instances above 9 to port 14549 is commented out.
3. In that copy, `etc/init.d-posix/rcS`: `PARAM_FILE` / `PARAM_BACKUP_FILE` point at `fs/parameters.bson` (where the simulator seeds them),
   and the SITL sensors' fixed calibration ids (`CAL_ACC0_ID 1310988` ... `CAL_MAG1_ID 197644`) are set on every boot.

`run.sh <name> [--set ...]` then flies `takeoff_spin.json` with `asbuilt.args` (the simulated aircraft matched to the real one) and
the board's flight parameters. Examples (all from the report):

    run.sh m_fans_cw   --set rotors[*].max_thrust=26 $(cat pin_km.args) --set "rotors[*].km=-0.0115"        # reproduces the spin
    run.sh m_declared  --set rotors[*].max_thrust=26                                                       # fans as declared
    run.sh m_fix_spin_front --set rotors[*].max_thrust=26 --set "rotors[*].km=-0.0115" \
        --set px4.CA_ROTOR{0..5,8}_KM=-0.0115 --set px4.CA_ROTOR6_KM=0.2331 --set px4.CA_ROTOR7_KM=-0.2561     # spin gone

`cmp.py` / `cmp2.py` score any PX4 log, real or simulated, from PX4's own "Takeoff detected".
