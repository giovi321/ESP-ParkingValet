# PlatformIO pre-build hook: regenerate src/web_ui.h and src/build_info.h.
import os
import subprocess
import sys

try:
    Import("env")  # provided by PlatformIO/SCons
    project_dir = env["PROJECT_DIR"]
except Exception:
    project_dir = os.getcwd()

# 1. Web UI -> gzipped PROGMEM header
gen = os.path.join(project_dir, "tools", "gen_web_ui.py")
subprocess.run([sys.executable, gen], check=True)

# 2. Build identifier (git short SHA) -> src/build_info.h, so the firmware can
#    report exactly which commit it was built from.
sha = "unknown"
try:
    sha = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=project_dir, stderr=subprocess.DEVNULL,
    ).decode().strip() or "unknown"
except Exception:
    pass

header = '#pragma once\n#define BUILD_GIT_SHA "%s"\n' % sha
path = os.path.join(project_dir, "src", "build_info.h")
prev = ""
if os.path.exists(path):
    with open(path, "r", encoding="utf-8") as f:
        prev = f.read()
if prev != header:  # only rewrite when it changes, to keep incremental builds fast
    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
    print("[build_info] BUILD_GIT_SHA=%s" % sha)
