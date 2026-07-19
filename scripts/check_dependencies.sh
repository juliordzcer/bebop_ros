#!/usr/bin/env bash
#
# Detects (and, with --install, installs) everything this repository needs
# to `colcon build` and run. Intended usage, from a fresh clone:
#
#   cd ~/ws_bebop/src
#   git clone https://github.com/juliordzcer/bebop_ros.git
#   ./bebop_ros/scripts/check_dependencies.sh --install
#   cd ~/ws_bebop
#   colcon build
#
# Run without --install to only report what is missing (exit code 1 if
# anything is missing, 0 if everything is already present).
#
# What it checks:
#   1. ROS 2 (a sourced /opt/ros/<distro>)
#   2. rosdep-resolvable package.xml dependencies (rclpy, tf2_ros, joy, ...)
#   3. Gazebo Harmonic dev libraries (gz-sim/gz-transport/...). These are
#      NOT resolvable via rosdep on most distros (missing/incomplete rosdep
#      keys for gz-* on Ubuntu Noble at the time of writing), so they are
#      checked/installed explicitly here instead.
#   4. Python packages imported by the nodes (numpy, scipy, transforms3d, ...)

set -uo pipefail

DO_INSTALL=0
for arg in "$@"; do
  case "$arg" in
    --install) DO_INSTALL=1 ;;
    -h|--help)
      sed -n '2,20p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg (use --install or --help)" >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
# When cloned as <ws>/src/bebop_ros, this resolves to <ws>/src.
WS_SRC_DIR="$(dirname "$REPO_DIR")"

MISSING_APT=()
MISSING_PIP=()
WARNINGS=()

log()  { echo "[check_dependencies] $*"; }
ok()   { echo "  [ok]      $*"; }
miss() { echo "  [missing] $*"; }

# ---------------------------------------------------------------------------
# 1. ROS 2 distro
# ---------------------------------------------------------------------------
if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -d /opt/ros ]; then
    # Pick the newest installed distro directory that has a setup.bash.
    ROS_DISTRO="$(ls -1 /opt/ros 2>/dev/null | while read -r d; do
      [ -f "/opt/ros/$d/setup.bash" ] && echo "$d"
    done | sort | tail -n1)"
  fi
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "No ROS 2 installation found (no \$ROS_DISTRO and nothing under /opt/ros)."
  echo "Install ROS 2 first -- see the 'Requirements' section in README.md."
  exit 1
fi

log "Using ROS_DISTRO=$ROS_DISTRO"
# ROS 2's setup.bash is not `set -u` safe (references unset vars itself),
# so relax that check just for sourcing it.
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

# ---------------------------------------------------------------------------
# 2. rosdep-resolvable package.xml dependencies
# ---------------------------------------------------------------------------
log "Checking ROS package dependencies (rosdep) ..."
if ! command -v rosdep >/dev/null 2>&1; then
  MISSING_APT+=("python3-rosdep")
  WARNINGS+=("rosdep is not installed; cannot check package.xml dependencies yet.")
elif [ ! -d "$WS_SRC_DIR" ]; then
  WARNINGS+=("Could not locate the workspace src/ directory from $SCRIPT_DIR; skipping rosdep check.")
else
  if [ ! -d /etc/ros/rosdep/sources.list.d ] && [ ! -f "$HOME/.ros/rosdep/sources.cache" ]; then
    WARNINGS+=("rosdep does not look initialized. Run: sudo rosdep init && rosdep update")
  fi
  ROSDEP_OUT="$(rosdep install --from-paths "$WS_SRC_DIR" --ignore-src -y --simulate 2>&1)"
  ROSDEP_STATUS=$?
  if [ $ROSDEP_STATUS -eq 0 ] && ! echo "$ROSDEP_OUT" | grep -q "^apt-get install"; then
    ok "all rosdep-resolvable dependencies are already installed"
  elif [ $ROSDEP_STATUS -eq 0 ]; then
    miss "rosdep reports missing packages (will be installed via 'rosdep install' below)"
    NEED_ROSDEP_INSTALL=1
  else
    WARNINGS+=("rosdep could not resolve all keys (typo in a package.xml <depend>? see below):")
    WARNINGS+=("$ROSDEP_OUT")
  fi
fi

# ---------------------------------------------------------------------------
# 3. Gazebo Harmonic + build tools (not covered by rosdep)
# ---------------------------------------------------------------------------
log "Checking Gazebo Harmonic + build tools ..."
APT_PACKAGES=(
  build-essential
  cmake
  libgz-cmake3-dev
  libgz-sim8-dev
  libgz-transport13-dev
  libgz-msgs10-dev
  libgz-plugin2-dev
  libgz-math7-dev
  libgz-common5-dev
  "ros-${ROS_DISTRO}-ros-gz"
)

for pkg in "${APT_PACKAGES[@]}"; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    ok "$pkg"
  else
    miss "$pkg"
    MISSING_APT+=("$pkg")
  fi
done

if ! dpkg -s libgz-cmake3-dev >/dev/null 2>&1 && ! apt-cache policy libgz-cmake3-dev 2>/dev/null | grep -q "Candidate:"; then
  WARNINGS+=("The Gazebo (osrfoundation) apt repository does not look configured, so libgz-*-dev packages cannot be found/installed. See 'Install Gazebo Harmonic' in README.md to add it, then re-run this script.")
fi

# ---------------------------------------------------------------------------
# 4. Python packages
# ---------------------------------------------------------------------------
log "Checking Python packages ..."
PY_MODULES=(numpy scipy matplotlib transforms3d)
for mod in "${PY_MODULES[@]}"; do
  if python3 -c "import ${mod}" >/dev/null 2>&1; then
    ok "python3: $mod"
  else
    miss "python3: $mod"
    MISSING_PIP+=("$mod")
  fi
done

# ---------------------------------------------------------------------------
# Summary / install
# ---------------------------------------------------------------------------
echo
if [ ${#WARNINGS[@]} -gt 0 ]; then
  log "Warnings:"
  for w in "${WARNINGS[@]}"; do
    echo "  - $w"
  done
  echo
fi

NOTHING_MISSING=1
[ ${#MISSING_APT[@]} -gt 0 ] && NOTHING_MISSING=0
[ ${#MISSING_PIP[@]} -gt 0 ] && NOTHING_MISSING=0
[ "${NEED_ROSDEP_INSTALL:-0}" = "1" ] && NOTHING_MISSING=0

if [ "$NOTHING_MISSING" = "1" ]; then
  log "All dependencies are already installed. You can run: colcon build"
  exit 0
fi

if [ "$DO_INSTALL" != "1" ]; then
  log "Missing dependencies were found. Re-run with --install to install them,"
  log "or install them manually (see README.md)."
  exit 1
fi

log "Installing missing dependencies (sudo will prompt for your password) ..."

if [ "${NEED_ROSDEP_INSTALL:-0}" = "1" ]; then
  rosdep install --from-paths "$WS_SRC_DIR" --ignore-src -y
fi

if [ ${#MISSING_APT[@]} -gt 0 ]; then
  sudo apt-get update
  sudo apt-get install -y "${MISSING_APT[@]}"
fi

if [ ${#MISSING_PIP[@]} -gt 0 ]; then
  pip3 install --break-system-packages "${MISSING_PIP[@]}"
fi

log "Done. Re-run this script (without --install) to verify, then: colcon build"
