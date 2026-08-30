#!/usr/bin/env bash
set -euo pipefail

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
