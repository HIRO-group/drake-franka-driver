# externals_ext.bzl
#
# Local repository rules for system-installed libraries.
# Only libfranka is enhanced to avoid "-lfranka not found" by importing
# the actual .so. LCM and gflags stay minimal and unchanged in spirit.

def _write_build(repo_ctx, lib_name, linkopt):
    repo_ctx.file(
        "BUILD.bazel",
        content = """
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "{lib}",
    includes = ["usr/include"],
    linkopts = ["{link}"]
)
""".format(lib = lib_name, link = linkopt)
    )

def _ensure_usr_include(repo_ctx):
    # Expose system headers at <repo>/usr/include so dependents get -I.../usr/include
    repo_ctx.symlink("/usr/include", "usr/include")

# ===========================
# LCM (unchanged behavior)
# ===========================
def _lcm_repo_impl(repo_ctx):
    _ensure_usr_include(repo_ctx)
    _write_build(repo_ctx, lib_name = "lcm", linkopt = "-llcm")

lcm_repo = repository_rule(
    implementation = _lcm_repo_impl,
    local = True
)

# ======================================================
# libfranka — import the real .so to avoid link search
# ======================================================

def _pick(repo_ctx, paths):
    for p in paths:
        if repo_ctx.path(p).exists:
            return p
    return None

def _libfranka_repo_impl(repo_ctx):
    # Default to /usr/local; can be overridden via MODULE.bazel attr `path`
    base = repo_ctx.attr.path
    if base == "":
        base = "/usr/local"

    inc_dir = base + "/include"
    if not repo_ctx.path(inc_dir).exists:
        fail("libfranka headers not found under {} (expected {}/franka/*.h)".format(
            inc_dir, inc_dir
        ))

    # Try common lib directories
    lib_dir = _pick(repo_ctx, [
        base + "/lib",
        base + "/lib/x86_64-linux-gnu",
        base + "/lib64"
    ])
    if lib_dir == None:
        lib_dir = "/usr/local/lib"

    # Point to a concrete shared object so the linker never needs -L
    so_path = _pick(repo_ctx, [
        lib_dir + "/libfranka.so",         # preferred soname symlink
        lib_dir + "/libfranka.so.0.9.2",   # matches your machine
        lib_dir + "/libfranka.so.0.9"
    ])
    if so_path == None:
        fail("libfranka shared library not found under {} (looked for libfranka.so*)".format(lib_dir))

    # Mirror headers and the .so into the repo
    repo_ctx.symlink(inc_dir, "include")
    repo_ctx.symlink(so_path, "libfranka.so")

    repo_ctx.file(
        "BUILD.bazel",
        content = """
package(default_visibility = ["//visibility:public"])

cc_import(
    name = "franka_so",
    shared_library = ":libfranka.so"
)

cc_library(
    name = "libfranka",
    hdrs = glob(["include/franka/**/*.h"]),
    includes = ["include"],
    deps = [":franka_so"],
    # Ensure runtime can find the real directory containing libfranka
    linkopts = ["-Wl,-rpath,{rpath}"]
)
""".format(rpath = lib_dir)
    )

libfranka_repo = repository_rule(
    implementation = _libfranka_repo_impl,
    attrs = {
        # Override in MODULE.bazel if installed under /usr:
        #   libfranka_repo_rule(name = "libfranka_v4", path = "/usr")
        "path": attr.string(default = "/usr/local")
    },
    local = True
)

# ===========================
# gflags (unchanged behavior)
# ===========================
def _gflags_repo_impl(repo_ctx):
    _ensure_usr_include(repo_ctx)
    _write_build(repo_ctx, lib_name = "gflags", linkopt = "-lgflags")

gflags_repo = repository_rule(
    implementation = _gflags_repo_impl,
    local = True
)
