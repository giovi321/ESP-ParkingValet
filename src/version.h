#pragma once
// Firmware version + build-SHA, shared by the translation units that report them
// (main.cpp, web_server.cpp, mqttc.cpp). tools/pio_prebuild.py generates
// build_info.h with the real git SHA; this folds it in when present and provides
// the fallbacks. PARKINGCAM_VERSION normally comes from a -D build flag; the guard
// below leaves that definition intact and only supplies a fallback if it is absent.
#if defined(__has_include)
#  if __has_include("build_info.h")
#    include "build_info.h"
#  endif
#endif
#ifndef PARKINGCAM_VERSION
#define PARKINGCAM_VERSION "0.0.0"
#endif
#ifndef BUILD_GIT_SHA
#define BUILD_GIT_SHA "dev"
#endif
