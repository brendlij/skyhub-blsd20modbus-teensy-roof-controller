"""Injects build timestamp and git commit into the firmware as -D defines.

Consumed by src/FirmwareInfo.h, which falls back to "unknown" for both when
this script has not run (see the #ifndef guards there), so a build without
PlatformIO or without git still compiles.

Note: the timestamp changes on every build, which changes the compiler flags
and therefore forces a full rebuild each time. That is the price of an
honest build stamp; a full rebuild of this project takes a few seconds.
"""

import datetime
import subprocess

Import("env")  # noqa: F821 -- injected by PlatformIO/SCons


def git(*args):
    """Runs a git command, returning "" if git or the repo is unavailable."""
    try:
        out = subprocess.check_output(
            ["git", *args],
            cwd=env.subst("$PROJECT_DIR"),  # noqa: F821
            stderr=subprocess.DEVNULL,
        )
        return out.decode("utf-8", "replace").strip()
    except Exception:
        return ""


commit = git("rev-parse", "--short=7", "HEAD") or "unknown"

# Flag uncommitted changes: an image built from a dirty tree cannot be traced
# back to a commit, and SkyHub should be able to see that in INFO.
if commit != "unknown" and git("status", "--porcelain"):
    commit += "-dirty"

build_timestamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

env.Append(  # noqa: F821
    CPPDEFINES=[
        ("FW_BUILD_TIMESTAMP", env.StringifyMacro(build_timestamp)),  # noqa: F821
        ("FW_GIT_COMMIT", env.StringifyMacro(commit)),  # noqa: F821
    ]
)

print("build metadata: %s @ %s" % (commit, build_timestamp))
