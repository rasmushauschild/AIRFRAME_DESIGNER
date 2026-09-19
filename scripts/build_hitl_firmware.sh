#!/bin/bash
# Build PX4 firmware for a flight controller WITH the HIL output driver (pwm_out_sim) that HITL needs.
# Standard PX4 release firmware for most boards leaves it out (CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=n).
#
#   scripts/build_hitl_firmware.sh [board] [build|upload] [variant]
#     board    px4_fmu-v6x (default), px4_fmu-v5, px4_fmu-v6c, ...
#     upload   build, then flash over USB (unplug/replug when asked)
#     variant  board config variant; default: "multicopter" if the board has one, else "default".
#              (fmu-v6x "default" is 100% full and does not link with pwm_out_sim added; "multicopter"
#               drops fixed-wing/VTOL/airspeed code and fits.)
#   PX4_REF=v1.17.0 scripts/build_hitl_firmware.sh ...   build from that tag (submodules included) so the
#              firmware matches the release already on the board; the checkout is restored afterwards.
#   ATLAS_MODULES=$PWD/firmware/atlas scripts/build_hitl_firmware.sh ...   build the ATLAS nose-lift module and
#              allocator overlay in (the same modules as the SITL); do not combine with PX4_REF, the overlay is
#              pinned to the checked-out PX4 source.
#
# Needs the ARM toolchain PX4 uses:  brew tap osx-cross/arm; brew trust osx-cross/arm && brew install osx-cross/arm/arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13
set -euo pipefail
BOARD="${1:-px4_fmu-v6x}"
ACTION="${2:-build}"
VARIANT="${3:-${VARIANT:-}}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="$HERE/.venv/bin:$PATH"
# Homebrew's arm-gcc-bin@13 is keg-only unless linked; use it directly when present.
for d in /opt/homebrew/opt/arm-gcc-bin@13/bin /usr/local/opt/arm-gcc-bin@13/bin; do [ -d "$d" ] && export PATH="$d:$PATH"; done

if ! command -v arm-none-eabi-gcc >/dev/null; then
  echo "arm-none-eabi-gcc not found. Install the toolchain first:"
  echo "  brew tap osx-cross/arm; brew trust osx-cross/arm && brew install osx-cross/arm/arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13"
  exit 1
fi
cd "$PX4_DIR"
PREV_REF="$(git symbolic-ref -q --short HEAD || git rev-parse HEAD)"
restore_ref() {
  if [ -n "${PX4_REF:-}" ] && [ "$PX4_REF" != "$PREV_REF" ]; then
    echo "restoring checkout $PREV_REF"
    git checkout -q "$PREV_REF" && git submodule update -q --init --recursive
  fi
}
if [ -n "${PX4_REF:-}" ] && [ "$PX4_REF" != "$PREV_REF" ]; then
  echo "checking out $PX4_REF (was $PREV_REF)"
  git checkout -q "$PX4_REF" && git submodule update -q --init --recursive
fi
BOARD_DIR="boards/${BOARD/_//}"                    # px4_fmu-v6x -> boards/px4/fmu-v6x
if [ -z "$VARIANT" ]; then
  if [ -f "$BOARD_DIR/multicopter.px4board" ]; then VARIANT=multicopter; else VARIANT=default; fi
fi
CFG="$BOARD_DIR/$VARIANT.px4board"
TARGET="${BOARD}_${VARIANT}"
[ -f "$CFG" ] || { echo "no board config at $CFG"; exit 1; }
echo "building $TARGET from $(git describe --tags --always) with pwm_out_sim"

if grep -q "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" "$CFG"; then
  echo "pwm_out_sim already enabled in $CFG"
else
  echo "enabling pwm_out_sim in $CFG"
  echo "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" >> "$CFG"
fi

cleanup() { git checkout -q -- "$CFG" 2>/dev/null || true; restore_ref; }
trap cleanup EXIT          # leave the checkout clean; the built .px4 keeps the module regardless

OUT="${FIRMWARE_FILE:-build/$TARGET/$TARGET.px4}"     # FIRMWARE_FILE=<image.px4>: flash that image (archive restore)
if [ -n "${FIRMWARE_FILE:-}" ] && [ "$ACTION" = "upload" ]; then
  [ -f "$FIRMWARE_FILE" ] || { echo "no such firmware image: $FIRMWARE_FILE"; exit 1; }
  echo "flashing archived image $FIRMWARE_FILE (no build)"
fi
# ATLAS_MODULES=<dir> builds the same external modules the SITL uses (firmware/atlas: nose-lift module + allocator
# overlay) into the board firmware. The build directory is tied to that choice, so switching wipes it.
CACHE="build/$TARGET/CMakeCache.txt"
WANT="${ATLAS_MODULES:-}"
HAVE="$( [ -f "$CACHE" ] && sed -n 's/^EXTERNAL_MODULES_LOCATION:STRING=//p' "$CACHE" || true )"
if [ -z "${FIRMWARE_FILE:-}" ] && { [ "$ACTION" != "upload" ] || [ ! -f "$OUT" ]; }; then
  if [ "$WANT" != "$HAVE" ] && [ -d "build/$TARGET" ]; then
    echo "external modules changed ('$HAVE' -> '$WANT'): reconfiguring build/$TARGET from scratch"
    rm -rf "build/$TARGET"
  fi
  if [ -n "$WANT" ]; then
    echo "with ATLAS external modules from $WANT"
    EXTERNAL_MODULES_LOCATION="$WANT" make "$TARGET"
    date > "build/$TARGET/$TARGET.atlas"
  else
    make "$TARGET"
    rm -f "build/$TARGET/$TARGET.atlas"
  fi
fi
echo
echo "Firmware: $PX4_DIR/$OUT"
echo "Flash it with QGroundControl (Vehicle Setup > Firmware > Advanced > Custom firmware file),"
echo "or run:  scripts/build_hitl_firmware.sh $BOARD upload $VARIANT"

if [ "$ACTION" = "upload" ]; then
  # Call the uploader directly rather than `make upload`, which would re-run the build with whatever
  # sources are checked out. The uploader asks the running firmware to reboot into the bootloader.
  UPLOADER=""
  for c in Tools/px_uploader.py Tools/px_uploader/px_uploader.py platforms/nuttx/Debug/px_uploader.py; do
    [ -f "$c" ] && { UPLOADER="$c"; break; }
  done
  [ -n "$UPLOADER" ] || UPLOADER="$(git ls-files | grep -m1 'px_uploader.py$' || true)"
  if [ -z "$UPLOADER" ]; then   # newer trees dropped it; borrow the v1.17.0 copy
    mkdir -p build && git show v1.17.0:Tools/px_uploader.py > build/px_uploader.py && UPLOADER=build/px_uploader.py
  fi
  [ -n "$UPLOADER" ] || { echo "px_uploader.py not found in $PX4_DIR"; exit 1; }
  echo "Uploading $OUT over USB with $UPLOADER — if it waits, unplug and replug the board."
  python3 "$UPLOADER" --port "/dev/tty.usbmodemPX*,/dev/tty.usbmodem*,/dev/ttyACM*" "$OUT"
fi
