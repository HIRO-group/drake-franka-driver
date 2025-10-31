# gflags_ext.bzl

def _gflags_repo_impl(repo_ctx):
    # Expose system-installed gflags as a Bazel repo.
    # Requires: sudo apt-get install -y libgflags-dev
    repo_ctx.file(
        "BUILD.bazel",
        content = """
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gflags",
    hdrs = glob(["usr/include/gflags/*.h", "usr/include/gflags/*.hpp"]),
    includes = ["usr/include"],
    linkopts = ["-lgflags"],
)
""",
        executable = False,
    )

gflags_repo = repository_rule(
    implementation = _gflags_repo_impl,
    local = True,  # read-only view of the host FS (we reference /usr/include)
)
