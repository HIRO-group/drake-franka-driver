# -*- python -*-

workspace(name = "drake_franka_driver")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# Python deps required by rules_ros2 (needs whl_filegroup -> rules_python ≥ 0.32)
http_archive(
    name = "rules_python",
    sha256 = "2f5c284fbb4e86045c2632d3573fc006facbca5d1fa02976e89dc0cd5488b590",
    strip_prefix = "rules_python-1.6.3",
    url = "https://github.com/bazel-contrib/rules_python/releases/download/1.6.3/rules_python-1.6.3.tar.gz",
)

# Register Python toolchains (recommended)
load("@rules_python//python:repositories.bzl", "py_repositories")
py_repositories()

# Create @rules_ros2_pip_deps from rules_ros2's lockfile
load("@rules_python//python:pip.bzl", "pip_parse")
pip_parse(
    name = "rules_ros2_pip_deps",
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

# ============================================================================
# Drake (pinned to v1.21.0)
# ============================================================================

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "v1.21.0",
    "6571295843aff8e11620340739bf5eab7a25130f8f06667b2d3e6df85567509a",
)

DRAKE_STRIP_PREFIX = "drake-1.21.0"

# Allow building against a local Drake checkout via env var
load("//:environ.bzl", "environ_repository")
environ_repository(
    name = "environ",
    vars = ["FRANKA_LOCAL_DRAKE_PATH"],
)
load("@environ//:environ.bzl", "FRANKA_LOCAL_DRAKE_PATH")

# Remote Drake if FRANKA_LOCAL_DRAKE_PATH is unset
http_archive(
    name = "drake" if not FRANKA_LOCAL_DRAKE_PATH else "drake_ignored",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = DRAKE_STRIP_PREFIX,
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

# Local Drake if FRANKA_LOCAL_DRAKE_PATH is set
local_repository(
    name = "drake" if FRANKA_LOCAL_DRAKE_PATH else "drake_ignored",
    path = FRANKA_LOCAL_DRAKE_PATH,
)

# Drake externals
load("@drake//tools/workspace:default.bzl", "add_default_workspace")
add_default_workspace()

register_toolchains("@rules_foreign_cc//toolchains:all")

load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")

# ---------------------------------------------------------------------------
# libfranka: use system install at /usr/local for BOTH v4 and v5
#   Headers: /usr/local/include/franka/*.h
#   Libs:    /usr/local/lib/libfranka.so*
# This matches your BUILD deps:
#   @libfranka_v4//:libfranka
#   @libfranka_v5//:libfranka
# ---------------------------------------------------------------------------

new_local_repository(
    name = "libfranka_v4",
    path = "/usr/local",
    build_file_content = """
cc_library(
    name = "libfranka",
    hdrs = glob(["include/franka/**/*.h"]),
    includes = ["include"],
    linkopts = [
        "-L/usr/local/lib",
        "-Wl,-rpath,/usr/local/lib",
        "-lfranka",
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
)
""",
)

new_local_repository(
    name = "libfranka_v5",
    path = "/usr/local",
    build_file_content = """
cc_library(
    name = "libfranka",
    hdrs = glob(["include/franka/**/*.h"]),
    includes = ["include"],
    linkopts = [
        "-L/usr/local/lib",
        "-Wl,-rpath,/usr/local/lib",
        "-lfranka",
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
)
""",
)

# If other parts of your repo still expect these "common" repos, keep them.
# They do not affect the @libfranka_v{4,5} override above.
load("//tools/workspace/libfranka_common_v4:repository.bzl", "libfranka_common_v4_repository")
load("//tools/workspace/libfranka_common_v5:repository.bzl", "libfranka_common_v5_repository")
load("//tools/workspace/poco:repository.bzl", "poco_repository")

libfranka_common_v4_repository(name = "libfranka_common_v4", mirrors = DEFAULT_MIRRORS)
libfranka_common_v5_repository(name = "libfranka_common_v5", mirrors = DEFAULT_MIRRORS)
poco_repository(name = "poco")

# ============================================================================
# ROS 2 Humble via mvukov/rules_ros2 (CycloneDDS only)
# ============================================================================

# Add bazel_features dependency (required by rules_ros2/rules_foreign_cc in Bazel 7.x)
http_archive(
    name = "bazel_features",
    sha256 = "ba1282c1aa1d1fffdcf994ab32131d7c7551a9bc960fbf05f42d55a1b930cbfb",
    strip_prefix = "bazel_features-1.15.0",
    urls = ["https://github.com/bazel-contrib/bazel_features/releases/download/v1.15.0/bazel_features-v1.15.0.tar.gz"],
)
load("@bazel_features//:deps.bzl", "bazel_features_deps")
bazel_features_deps()

git_repository(
    name = "com_github_mvukov_rules_ros2",
    remote = "https://github.com/mvukov/rules_ros2.git",
    commit = "4710585558693b62bdab089ca3d0f050b765c819",
)

# Materialize wheels into repos
load("@rules_ros2_pip_deps//:requirements.bzl", "install_deps")
install_deps()

# Bring in all repos for ROS 2 Humble
load(
    "@com_github_mvukov_rules_ros2//repositories:repositories.bzl",
    "ros2_workspace_repositories",
    "ros2_repositories",
)
ros2_workspace_repositories()
ros2_repositories()

# ============================================================================
# Notes
# ============================================================================
# Runtime middleware (CycloneDDS only):
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Optional CycloneDDS XML:
#   export CYCLONEDDS_URI=/path/to/cyclonedds_profile.xml
#
# After switching rules:
#   bazel clean --expunge && bazel sync
#
# Use these deps in BUILD files (mvukov label style):
#   "@ros2_rclcpp//:rclcpp"
#   "@ros2_rcutils//:rcutils"