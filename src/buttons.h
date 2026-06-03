#pragma once
#include <Arduino.h>

// BOOT (GPIO0) long-press handling.
//
// Holding BOOT for >= LONGPRESS_MS at any time latches a "force AP config mode"
// flag in RTC memory and reboots. On the next boot, buttonsConsumeForcedAp()
// reports true exactly once so main() can start in AP/config mode.
//
// A power-on reset never counts as a forced-AP boot (the flag is only honored
// after a software restart), so a cold boot always behaves normally.

static const uint32_t LONGPRESS_MS = 5000;

void buttonsBegin();

// Service the button. Call frequently from loop(). Triggers the reboot itself.
void buttonsLoop();

// Returns true once if this boot was triggered by a BOOT long-press.
bool buttonsConsumeForcedAp();

// Latch the force-AP flag and reboot (used by the web UI "config mode" action).
void buttonsForceApMode();

// Optional: print transitions on a safe set of candidate GPIOs so the board's
// physical buttons can be identified. Enable with -DPARKINGCAM_BUTTON_DISCOVERY.
void buttonsDiscoveryScan(uint32_t durationMs = 30000);
