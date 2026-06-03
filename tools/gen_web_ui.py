#!/usr/bin/env python3
"""Gzip web-src/index.html into src/web_ui.h as a PROGMEM byte array.

Run standalone:  python tools/gen_web_ui.py
Also invoked automatically as a PlatformIO pre-build step (see platformio.ini).
"""
import gzip
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "web-src", "index.html")
OUT = os.path.join(ROOT, "src", "web_ui.h")


def main() -> int:
    if not os.path.exists(SRC):
        print(f"[gen_web_ui] source not found: {SRC}", file=sys.stderr)
        return 1
    with open(SRC, "rb") as f:
        raw = f.read()
    gz = gzip.compress(raw, 9)

    # Skip regeneration if unchanged (keeps incremental builds fast).
    if os.path.exists(OUT):
        with open(OUT, "r", encoding="utf-8") as f:
            head = f.read(200)
        marker = f"// raw={len(raw)} gz={len(gz)}"
        if marker in head:
            print(f"[gen_web_ui] up to date ({len(raw)} -> {len(gz)} bytes)")
            return 0

    lines = []
    lines.append("#pragma once")
    lines.append(f"// raw={len(raw)} gz={len(gz)}  (generated from web-src/index.html; do not edit)")
    lines.append("#include <Arduino.h>")
    lines.append(f"const unsigned int index_html_gz_len = {len(gz)};")
    lines.append("const uint8_t index_html_gz[] PROGMEM = {")
    for i in range(0, len(gz), 16):
        chunk = gz[i:i + 16]
        lines.append("  " + ",".join(f"0x{b:02x}" for b in chunk) + ",")
    lines.append("};")
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[gen_web_ui] wrote {OUT}  ({len(raw)} -> {len(gz)} bytes gzipped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
