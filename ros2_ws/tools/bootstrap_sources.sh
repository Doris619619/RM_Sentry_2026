#!/usr/bin/env bash
set -euo pipefail

verify_offline_build=false
if [[ "${1:-}" == "--verify-offline-build" ]]; then
  verify_offline_build=true
elif [[ $# -ne 0 ]]; then
  echo "usage: $0 [--verify-offline-build]" >&2
  exit 2
fi

# The only networked setup step. It records fixed commits in third_party.repos;
# after it succeeds, colcon builds consume local sources and can run offline.
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repository_dir="$(cd "${workspace_dir}/.." && pwd)"
sdk_source="${repository_dir}/Livox-SDK2"
sdk_prefix="${workspace_dir}/.deps/livox-sdk2"

command -v vcs >/dev/null || { echo 'install python3-vcstool first'; exit 1; }
command -v rosdep >/dev/null || { echo 'install python3-rosdep first'; exit 1; }
test -f "${sdk_source}/CMakeLists.txt" || { echo "missing repository Livox-SDK2: ${sdk_source}"; exit 1; }


vcs import "${workspace_dir}/src" < "${workspace_dir}/third_party.repos"

verify_revision() {
  local source_dir="$1"
  local expected="$2"
  local actual
  actual="$(git -C "${source_dir}" rev-parse HEAD)"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "pinned source mismatch: ${source_dir} expected ${expected}, got ${actual}" >&2
    exit 1
  fi
}
verify_revision "${workspace_dir}/src/vendor/blasfeo" "ae6e2d1dea015862a09990b95905038a756ffc7d"
verify_revision "${workspace_dir}/src/vendor/hpipm" "255ffdf38d3a5e2c3285b29568ce65ae286e5faf"
verify_revision "${workspace_dir}/src/livox_ros_driver2" "dd6c8de14479197e314270af8133f8a4cbe16ff9"

# BLASFEO and HPIPM are non-ROS vendor dependencies, consumed only by the catkin wrappers.
touch "${workspace_dir}/src/vendor/blasfeo/COLCON_IGNORE" "${workspace_dir}/src/vendor/hpipm/COLCON_IGNORE"

# Keep the pinned upstream driver usable with ordinary Humble colcon.  This
# patch removes an unset historical HUMBLE_ROS branch; it neither changes the
# pinned revision nor downloads code during a build.
livox_patch="${workspace_dir}/tools/patches/livox_ros_driver2-humble-cmake.patch"
livox_source="${workspace_dir}/src/livox_ros_driver2"
if git -C "${livox_source}" apply --reverse --check "${livox_patch}" >/dev/null 2>&1; then
  echo "livox_ros_driver2 Humble patch already applied"
else
  git -C "${livox_source}" apply "${livox_patch}"
fi

# No sudo and no ldconfig: Livox-SDK2 is installed into a workspace-managed
# prefix, then passed to the driver through CMAKE_PREFIX_PATH.
cmake -S "${sdk_source}" -B "${workspace_dir}/.deps/livox-sdk2-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${sdk_prefix}"
# SDK2 builds static and shared variants concurrently internally; default to one
# top-level job so a clean Ubuntu VM does not exhaust memory. Override explicitly.
cmake --build "${workspace_dir}/.deps/livox-sdk2-build" --parallel "${RM_SENTRY_SDK_JOBS:-1}"
cmake --install "${workspace_dir}/.deps/livox-sdk2-build"

# This may use apt and therefore needs normal administrator permission on a new
# host.  It is deliberately separate from colcon's offline build phase.
rosdep install --from-paths "${workspace_dir}/src" --ignore-src \
  --rosdistro "${ROS_DISTRO:-humble}" -r -y

echo "bootstrap complete"
echo "For every build: export CMAKE_PREFIX_PATH=${sdk_prefix}:\${CMAKE_PREFIX_PATH:-}"
echo "Then source /opt/ros/humble/setup.bash and run colcon build; it performs no network fetch."

if [[ "${verify_offline_build}" == true ]]; then
  command -v unshare >/dev/null || { echo "unshare is required for offline-build verification" >&2; exit 1; }
  # A network namespace has no interfaces.  Any accidental fetch fails instead
  # of silently using the host network.
  unshare --user --map-root-user --net -- bash -lc "
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    export CMAKE_PREFIX_PATH='${sdk_prefix}:'"\${CMAKE_PREFIX_PATH:-}"
    cd '${workspace_dir}'
    colcon --log-base /tmp/rm-sentry-offline-log build --build-base /tmp/rm-sentry-offline-build --install-base /tmp/rm-sentry-offline-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
  "
fi
