# SMALL_SCALE_V3: the last SITL before the first real flight (2026-10-19)

Everything needed to fly V3 in SITL exactly as it was flown on Rasmus's machine on 19 October 2026,
before LIVE_191026_FLIGHT1 (22:11 the same evening).

## The sessions

| Run | Started (-0700) | What happened |
|---|---|---|
| `runs/20261019-135834.json` | 13:58, 19.6 min | **Latest SITL session of V3.** Native ATLAS build, `SYS_HITL = 0`, real-flight wiring (motor 9 on AUX 1). SITL ran only 13:58:34-14:06:01 ("stopping PX4 SITL"); the "Horizontal velocity unstable" / "Heading estimate invalid" lines after that come from whatever the app connected to next, most likely the board. Never armed. |
| `runs/20261019-122316.json` | 12:23, 38.6 min | Last SITL session that armed and took off (0.83 m max). Motor 9 on MAIN 9. |

Both carry the same airframe. Each run file holds the airframe, the parameters PX4 was given and the
app log of the session. They are extracted beside it:

* `airframe_V3_<run>.json` - the airframe exactly as the app had it
* `px4_params_<run>.json` - the parameters PX4 was given (the real-flight style set: `PWM_MAIN/AUX_FUNC`,
  not `HIL_ACT_FUNC`). A fresh export of the airframe at the commit below agrees on every aircraft and
  controller parameter; only the output wiring keys and `SYS_HITL` differ. Replay this file, not a fresh export.

## The code

| Part | Where | Version |
|---|---|---|
| App | `designer/` (git worktree of AIRFRAME_DESIGNER) | `7eab64c` - `main` at 13:58 that day (`efd3df8` + a PORTAL-only airframe change) |
| PX4 source | `~/PX4-Autopilot-atlas` (git worktree of ~/PX4-Autopilot, with submodules) | `2a0e9109238bd0274608e3c25201be7c40becd0c` - the commit every archive manifest records |
| ATLAS module | `designer/firmware/atlas` (`src`, `msg`, `allocator_overlay`) | identical to the archive copies `20261019-162016_preset` and `20261019-193544_px4_fmu-v6x` |
| SITL binary | `designer/firmware/atlas/build/px4_sitl_default/bin/px4` | built from the two above |

`ControlAllocator.cpp` at `2a0e9109` hashes to `f000d3cc...`, the value `firmware/atlas/src/CMakeLists.txt` pins.
The ordinary `~/PX4-Autopilot` (v1.17.0) does not, which is why the build failed there.

## The archive

`archive/` holds three versions from rasmushauschild/PX4_FIRMWARE (commit in `archive/ARCHIVE_COMMIT`):
`20261019-162016_preset` and `20261019-175313_preset` (the afternoon's board snapshots) and
`20261019-193544_px4_fmu-v6x` (the image actually flashed to the Pixhawk that evening, `firmware.px4`).

PX4 at this commit reads its parameters from `fs/parameters.bson` in its working directory (the session log
says so: "selected parameter default file fs/parameters.bson"). PX4 v1.17 reads `parameters.bson` one level up.
The app at `7eab64c` writes the seed to `fs/`, which is right for this PX4 and wrong for v1.17.

## Rebuild

```sh
cd ~/AIRFRAME_REPRO/v3_sitl_20261019/designer/firmware/atlas
export PATH="$HOME/.venvs/px4-build/bin:$PATH" SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
cmake -S ~/PX4-Autopilot-atlas -B build/px4_sitl_default -G Ninja -DCONFIG=px4_sitl_default \
  -DEXTERNAL_MODULES_LOCATION="$PWD" -DPYTHON_EXECUTABLE=$HOME/.venvs/px4-build/bin/python -DCMAKE_OSX_SYSROOT=$SDKROOT
CCACHE_DIR=/tmp/atlas-nl-ccache cmake --build build/px4_sitl_default -j 6
```

* `~/.venvs/px4-build` holds PX4's Python build tools (kconfiglib, jinja2, empy); the project `.venv` does not.
* `SDKROOT` matters: the default SDK on this Mac (MacOSX27.0) has `.tbd` files the installed linker cannot
  read ("unknown architecture"), and the micro XRCE-DDS sub-build ignores `CMAKE_OSX_SYSROOT`.
* The PX4 history down to v1.18.0-beta1 must be present (`git fetch --shallow-since=2026-05-01 origin 2a0e9109...`
  in ~/PX4-Autopilot), or PX4 calls itself v0.0.0. With it: `v1.18.0-beta1-647-g2a0e910923`, the
  "1.18.0 beta" the board reports.

## Fly

From `designer/`, with the main project's venv:

```sh
/Users/stefanosfragkoulis/Documents/UtopiaLabs/new/AIRFRAME_DESIGNER/.venv/bin/python -m airframe_designer ui \
  --airframe ../airframe_V3_20261019-135834.json --px4-instance 6 --http 127.0.0.1:8086 --no-browser --mode sitl
```

Instances 0-9 only: PX4 sends every instance above 9 to UDP 14549, and this app listens on 14540 + instance.

## Reproduced 2026-10-22

Built SITL binary: sha256 `bab3a09e8dd6e871b61795c86341ec8d69daa446943032b90b3383a2656a47bd`, reports
`v1.18.0-beta1-647-g2a0e910923`. Booted with the 13:58 airframe, PX4 imported a seed of the same size and
make-up as Rasmus's ("2536 bytes, INT32:33, FLOAT:84") and held 99 of the 100 recorded parameters
(`PWM_AUX_FUNC1` does not exist in SITL, there or here). Takeoff button, PX4's own takeoff to 1.0 m,
recording in `runs/reproduced_20261022.json`:

| Hover after the climb | 2026-10-22 | 2026-10-19 12:23, flight 3 |
|---|---|---|
| height | 0.69 ± 0.01 m | 0.70 ± 0.01 m |
| max abs roll | 4.2° | 4.5° |
| pitch (PX4 frame) | 7.3 ± 1.4° | 7.3 ± 1.4° |
| max ground speed | 0.17 m/s | 0.17 m/s |
| power | 12 215 W | 12 219 W |

The first arm attempt after boot is refused ("Resolve system health failures first") until the estimator
has aligned, a few seconds; the 12:23 session shows the same refusal.
